/* SPDX-License-Identifier: GPL-3.0-only */
/* FELUCCA_FGBENCH=1 (fm1-groovebox bench/build.sh): the fg_* engines (build/fg/, their own objects) play a fixed
 * pattern in place of mix_block, to measure what 8 tracks cost on the FM-1 before building a UI on them. The audio
 * ISR still calls audio_block per CTL: the first chunk of each half renders the half's 128 frames (the engines'
 * block), the next three copy from it. MASTER scales it as it does mix_block. Console: `fg`.
 * Guards, so a bench that does not fit leaves the FM-1 up for the next install: silent for the first 2 s (USB and
 * the UI start first), and not at all after a crash, a hang or a failed boot (main.c boot_clean); 4 halves in a row
 * over 90 % of a half drop the worst-case scene, and if the CAJA scene still does, the bench goes silent.
 * Second test (2026-10-22): the four synths run on the second core (started at power-on in this build, whatever
 * MENU > 2ND CORE says; not after a failed boot), the drums and the mix on this one; a unit whose second core
 * misreads runs everything here, as Felucca does. */
#include "fg_bench.h"

#define FGB_BUDGET_TICKS (HALF_FRAMES * 544u)          /* a half at 44,117.6 Hz in TIMER4 ticks (audio.c DAC_TICKS) */
#define FGB_BUDGET_US (FGB_BUDGET_TICKS / FM1_TICKS_PER_US)
#define FGB_START_MS 2000u
#define FGB_OVER_TICKS (FGB_BUDGET_TICKS * 9u / 10u)

enum { FGBS_WAIT, FGBS_RUN, FGBS_NO_WORST, FGBS_STOP_OVER, FGBS_STOP_BOOT };
static const char *const FGB_STATE[] = {"esperando", "sonando", "sonando sin peor caso (no le cupo)",
                                        "detenida: la escena caja no cupo", "apagada: el arranque anterior fallo"};
static uint8_t boot_clean;                             /* main.c */
static int32_t fgb_buf[2u * HALF_FRAMES];
static uint32_t fgb_off, fgb_over;
static uint8_t fgb_state;

uint32_t fgb_now(void) { return fm1_ticks(); }
int fgb_core1_run(void (*fn)(uint32_t), uint32_t arg) { return C1_RUN(fn, arg); }
int fgb_core1_wait(void) { return C1_WAIT(); }

static void fgb_half(void)
{
    uint32_t t0, i;
    if (fgb_state == FGBS_WAIT && fm1_ms >= FGB_START_MS)
        fgb_state = boot_clean ? FGBS_RUN : FGBS_STOP_BOOT;
    if (fgb_state != FGBS_RUN && fgb_state != FGBS_NO_WORST) {
        for (i = 0; i < 2u * HALF_FRAMES; i++)
            fgb_buf[i] = 0;
        return;
    }
    t0 = fm1_ticks();
    fg_bench_render(fgb_buf, HALF_FRAMES);
    fgb_over = fm1_ticks() - t0 > FGB_OVER_TICKS ? fgb_over + 1u : 0u;
    if (fgb_over >= 4u) {
        fgb_over = 0;
        if (fg_bench_scene() == 1u && fgb_state == FGBS_RUN) {
            fg_bench_drop_worst();
            fgb_state = FGBS_NO_WORST;
        } else {
            fgb_state = FGBS_STOP_OVER;
        }
    }
}

static void fgb_chunk(int32_t *out, uint32_t n)        /* audio ISR, via audio_block */
{
    uint32_t i;
    int32_t mg = fx_usb_fixed ? MASTER_FULL : (int32_t)song.master_q12;
    if (fgb_off == 0u)
        fgb_half();
    for (i = 0; i < 2u * n; i++)
        out[i] = (fgb_buf[2u * fgb_off + i] * mg) >> 12;
    fgb_off = (fgb_off + n) % HALF_FRAMES;
}
