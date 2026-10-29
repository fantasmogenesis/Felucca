/* SPDX-License-Identifier: GPL-3.0-only */
/* FELUCCA_FG=1: the fm1-groovebox firmware on Felucca's platform. Felucca keeps the boot, the guards, USB (MIDI,
 * the update and the console), the flash driver, the LCD and the input scan; the music, the screen and the
 * controls are fg_box (motores/src/fg_box.c, built as its own objects from build/fg/). Here:
 *   audio   the ISR's first chunk of each half renders the half's 128 frames with fg_box_audio (the synths on the
 *           second core), MASTER scales it, the MIDI clock goes out;
 *   loop    buttons, keys and encoders to fg_box, its LEDs, its screen band by band (two band buffers: one is
 *           drawn while the other's DMA runs; a band that did not change is not sent), MIDI clock in, autosave;
 *   flash   16 project slots, the autosave and the user's sound bank, 16 KiB each, in Felucca's data area
 *           (0x97000..0xDF000; the OTA staging starts at 0xE0000): a header (magic, length, CRC-32) and the data.
 * Safe start: after a failed boot (bootguard: a crash or hang in the last boot's first 30 s) the audio stays
 * silent and the screen says so, so the FM-1 stays up for a reinstall. */
#include "fg_box.h"

#define FGX_VIS_HIST 1u                                /* cuadros de historia para repite/stutter/reversa del visual */
static fg_box_t fgx_box __attribute__((section(".pool")));
static uint8_t fgx_vis[28800u + 8640u + FGX_VIS_HIST * 7200u + 64u] __attribute__((section(".pool"), aligned(4)));
static uint16_t fgx_band[2][240u * 16u] __attribute__((aligned(4)));   /* (.bss: el pool está lleno) */
static int32_t fgx_buf[2u * HALF_FRAMES];
static uint32_t fgx_off, fgx_frame_ms, fgx_notes;
static uint8_t fgx_safe, fgx_bk, fgx_ready;
static uint32_t fgx_frames, fgx_frame_us_max, fgx_frame_us, fgx_vis_us, fgx_band_us, fgx_sent;

uint32_t fg_now(void) { return fm1_ticks(); }

/* MIDI clock in (USB and TRS): midi_enqueue (TIMER5) stamps each realtime byte; the audio takes them before the
 * half it renders and tells fg_box how long ago each arrived (fg_box_clock locks the sequencer to them) */
#define FGX_RT 32u
static uint32_t fgx_rt_t[FGX_RT];
static uint8_t fgx_rt_b[FGX_RT];
static volatile uint32_t fgx_rt_w, fgx_rt_r;
static void fgx_rt_push(uint32_t st)
{
    uint32_t w = fgx_rt_w;
    if (w - fgx_rt_r >= FGX_RT)
        return;
    fgx_rt_t[w % FGX_RT] = fm1_ticks();
    fgx_rt_b[w % FGX_RT] = (uint8_t)st;
    RING_PUBLISH();
    fgx_rt_w = w + 1u;
}
int fg_core1_run(void (*fn)(uint32_t), uint32_t arg) { return C1_RUN(fn, arg); }
int fg_core1_wait(void) { return C1_WAIT(); }
static uint32_t fgx_ms(void) { return fm1_ms; }

/* ------------------------------------------------------------- audio --- */
static void fgx_chunk(int32_t *out, uint32_t n)        /* audio ISR, via audio_block */
{
    uint32_t i, pk;
    int32_t mg = fx_usb_fixed ? MASTER_FULL : (int32_t)song.master_q12;
#if FELUCCA_UAC_MT
    uac_stem_off = fgx_off;                            /* uac_tap reads the tracks of this block (usb.c) */
#endif
    if (fgx_off == 0u) {
        if (fgx_ready && !fgx_safe) {
            uint32_t now = fm1_ticks();
            while (fgx_rt_r != fgx_rt_w) {             /* 24 MHz ticks -> samples: x 44100 / 24e6 = x 147 / 80000 */
                uint32_t r = fgx_rt_r % FGX_RT, d = now - fgx_rt_t[r];
                fg_box_clock(&fgx_box, fgx_rt_b[r], d > 24000000u ? 44100u : d * 147u / 80000u);
                RING_PUBLISH();
                fgx_rt_r++;
            }
            fg_box_audio(&fgx_box, fgx_buf, HALF_FRAMES);
            if (fgx_box.clk_start) { fgx_box.clk_start = 0; midi_out_event(0x0Fu | 0xFAu << 8); }
            if (fgx_box.clk_stop) { fgx_box.clk_stop = 0; midi_out_event(0x0Fu | 0xFCu << 8); }
            while (fgx_box.clk_pulses) { fgx_box.clk_pulses--; midi_out_event(0x0Fu | 0xF8u << 8); }
            while (fg_box_midi_out(&fgx_box, &pk)) midi_out_event(pk);   /* notas (canal = pista) */
        } else {
            for (i = 0; i < 2u * HALF_FRAMES; i++)
                fgx_buf[i] = 0;
        }
    }
    for (i = 0; i < 2u * n; i++)
        out[i] = (fgx_buf[2u * fgx_off + i] * mg) >> 12;
    fgx_off = (fgx_off + n) % HALF_FRAMES;
}

/* ------------------------------------------------------------- flash --- */
#define FGX_BASE 0x97000u
#define FGX_SLOT 0x4000u
#define FGX_MAGIC 0x31534746u                          /* "FGS1" */
typedef struct { uint32_t magic, len, crc; } fgx_hdr_t;
static uint32_t fgx_crc(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu, k;
    while (n--) {
        c ^= *p++;
        for (k = 0; k < 8u; k++)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}
static int fgx_write(unsigned slot, const void *d, uint32_t n)
{
    uint32_t off = FGX_BASE + slot * FGX_SLOT, s;
    fgx_hdr_t h;
    if (slot > FBX_UBANK_SLOT || n + sizeof h > FGX_SLOT || !flash_ok)
        return -1;
    h.magic = 0xFFFFFFFFu;                             /* the magic last: a cut write leaves no valid header */
    h.len = n;
    h.crc = fgx_crc(d, n);
    for (s = 0; s < n + sizeof h; s += 4096u)
        if (st_erase(off + s))
            return -2;
    if (st_prog(off + sizeof h, d, n))
        return -3;
    if (st_prog(off + 4u, &h.len, 8u))
        return -4;
    h.magic = FGX_MAGIC;
    if (st_prog(off, &h.magic, 4u))
        return -5;
    return 0;
}
static int fgx_head(unsigned slot, fgx_hdr_t *h)
{
    if (slot > FBX_UBANK_SLOT || !flash_ok || st_read(FGX_BASE + slot * FGX_SLOT, h, sizeof *h))
        return -1;
    return h->magic == FGX_MAGIC && h->len && h->len + sizeof *h <= FGX_SLOT ? 0 : -1;
}
static int fgx_read(unsigned slot, void *d, uint32_t cap, uint32_t *len)
{
    fgx_hdr_t h;
    if (fgx_head(slot, &h) || h.len > cap)
        return -1;
    if (st_read(FGX_BASE + slot * FGX_SLOT + sizeof h, d, h.len) || fgx_crc(d, h.len) != h.crc)
        return -2;
    *len = h.len;
    return 0;
}
static int fgx_has(unsigned slot)
{
    fgx_hdr_t h;
    return !fgx_head(slot, &h);
}

/* -------------------------------------------------------------- boot --- */
static void fgx_init(void)                             /* before audio_init */
{
    fgx_box.store_write = fgx_write;
    fgx_box.store_read = fgx_read;
    fgx_box.store_has = fgx_has;
    fgx_box.ms = fgx_ms;
    fg_box_init(&fgx_box, fgx_vis, 240, 240, FGX_VIS_HIST, 0x5eed1u ^ fm1_ticks());
    fgx_box.blk_ticks = HALF_FRAMES * FM1_TICKS_PER_US * 1000000u / 44100u;   /* (la polifonía mide el núcleo 1 contra esto) */
    fgx_safe = bootguard.failed != 0;
    if (fgx_safe)                                      /* nothing was loaded: writing would overwrite the autosave, the bank and the slots */
        fgx_box.store_write = 0;
#if FELUCCA_UAC_MT
    for (uint32_t t = 0; t < 8u; t++)
        uac_stem[t] = fgx_box.trk[t];
#endif
}
static void fgx_boot(void)                             /* after the timers: the flash and fm1_ms work */
{
    fgx_box.now = fm1_ms;
    if (!fgx_safe)
        fg_box_boot(&fgx_box);
    else {                                             /* the screen says why there is no sound */
        static const char L[] = "ARRANQUE SEGURO", V[] = "SIN SONIDO";
        uint32_t i;
        for (i = 0; i < sizeof L; i++) fgx_box.ov.label[i] = L[i];
        for (i = 0; i < sizeof V; i++) fgx_box.ov.value[i] = V[i];
        fgx_box.ov.bar = -1;
        fgx_box.ov.until = 0xFFFFFFFFu;
    }
    fgx_ready = 1;
}

/* -------------------------------------------------------------- loop --- */
static void fgx_led_half(uint32_t id)
{
    uint32_t p, r;
    for (p = 0; p < FM1_NCOL; p++)
        for (r = 1; r < 5u; r++)
            if (FM1_KEYMAP[r][p] == (int8_t)id) {
                fm1_led_mid[p] |= (uint8_t)(1u << r);
                fm1_led_dim[p] |= (uint8_t)(1u << r);
            }
}
static void fgx_leds(void)
{
    uint32_t bt, ky, i;
    if (fm1_led_anim_on())
        return;
    fg_box_leds(&fgx_box, &bt, &ky);
    for (i = 0; i < FM1_NCOL; i++) {
        fm1_led_dim[i] = 0;
        fm1_led_breath[i] = 0;
        fm1_led_mid[i] = 0;
    }
    for (i = 0; i < FB_NBTN; i++)
        fm1_led_key(panel.btn[i], (int)((bt >> i) & 1u));
    bt = fg_box_leds_half(&fgx_box);                    /* las páginas que no están elegidas: a media luz (el nivel medio */
    for (i = 0; i < FB_NBTN; i++)                       /* de la HAL más el brillo tenue, como los tiempos del DRUM) */
        if ((bt >> i) & 1u)
            fgx_led_half(panel.btn[i]);
    for (i = 0; i < FBX_KEYS; i++)
        fm1_led_key(14u + i, (int)((ky >> i) & 1u));
}

/* espejo (espejo/index.html): mientras la página lo pida (CC 119 en el canal 16 cada segundo), cada golpe y el
 * estado de cada cuadro salen por SysEx */
static uint32_t fgx_mirror_ms, fgx_mirror_on;
static void fgx_mirror(int state)
{
    static uint8_t m[80];
    unsigned n;
    fgx_box.mirror = (uint8_t)(fgx_mirror_on && fm1_ms - fgx_mirror_ms < 3000u);
    if (!fgx_box.mirror)
        return;
    while ((n = fg_box_mirror(&fgx_box, m, state)) != 0u) {
        if (ota_wire_send(m, n))
            break;
        if (m[4] == 2u)
            break;
    }
}

static void fgx_screen(void)
{
    uint32_t y, t0;
    if (fm1_ms - fgx_frame_ms < 33u)                   /* ~30 frames a second at most */
        return;
    fgx_frame_ms = fm1_ms;
    t0 = fm1_ticks();
    fg_box_frame(&fgx_box);
    fgx_mirror(1);
    fgx_vis_us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    fgx_band_us = 0;
    fgx_sent = 0;
    for (y = 0; y < 240u; y += 16u) {
        uint32_t t1 = fm1_ticks();
        int ch = fg_box_band(&fgx_box, y, fgx_band[fgx_bk]);
        fgx_band_us += (fm1_ticks() - t1) / FM1_TICKS_PER_US;
        if (ch) {
            lcd_blit(0, y, 240, 16, fgx_band[fgx_bk]);
            fgx_bk ^= 1u;
            fgx_sent++;
        }
    }
    fgx_frame_us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    if (fgx_frame_us > fgx_frame_us_max)
        fgx_frame_us_max = fgx_frame_us;
    fgx_frames++;
}

static void fgx_loop(void)
{
    uint32_t rel = 0, pr = fm1_input_edges(&rel), i, notes, ch;
    fg_box_tick(&fgx_box, fm1_ms);
    for (i = 0; i < 14u; i++) {                        /* matrix button ids -> printed labels */
        uint32_t lab = panel_btn_of(i);
        if (lab >= NB)
            continue;
        if ((rel >> i) & 1u)
            fg_box_button(&fgx_box, lab, 0);
        if ((pr >> i) & 1u)
            fg_box_button(&fgx_box, lab, 1);
    }
    notes = fm1_in.notes;
    ch = notes ^ fgx_notes;
    fgx_notes = notes;
    for (i = 0; i < FBX_KEYS; i++)
        if ((ch >> i) & 1u)
            fg_box_key(&fgx_box, i, (int)((notes >> i) & 1u));
    for (i = 0; i < NE; i++) {
        int32_t s = panel_enc(i);
        if (s)
            fg_box_enc(&fgx_box, i, s);
    }
    while (mi_r != mi_w) {                             /* MIDI in (USB and TRS): notes, CC, program change */
        uint32_t at = mi_r % MQ, pkt = midi_in_q[at], st = (pkt >> 8) & 0xFFu;
        RING_PUBLISH();
        mi_r++;
        if (st == 0xBFu && ((pkt >> 16) & 0x7Fu) == 119u) {   /* el espejo pide (o deja de pedir) */
            fgx_mirror_ms = fm1_ms;
            fgx_mirror_on = ((pkt >> 24) & 0x7Fu) != 0u;
        } else if (st >= 0x80u && st < 0xF0u)
            fg_box_midi_in(&fgx_box, (uint8_t)st, (uint8_t)(pkt >> 16), (uint8_t)(pkt >> 24));
    }
    if (midi_in_overflow) {
        mi_r = mi_w;
        midi_in_overflow = 0;
    }
    fgx_leds();
    fgx_mirror(0);                                     /* los golpes en cuanto llegan */
    fgx_screen();
    fg_box_idle(&fgx_box);
}
