/*
 * NTN downlink propagation delay as a transmit delay line (header only, no dependency).
 *
 * The time-varying one-way delay d(t) of a LEO pass is applied INSIDE the sample stream, so the radio gets
 * contiguous timestamps: tx_rf() writes every slot at timestamp + ts_offset + base (base constant) and this
 * line outputs y[o] = x[o - v], v = round(d * fs - base), from a ring of past input samples.
 * v is an integer and changes only at block (slot / subframe) starts, by at most one sample per block: the one
 * sample repeated or skipped is then the first sample of the block, i.e. a cyclic-prefix sample, so every OFDM
 * symbol goes out intact and only its timing moves, as through a real channel. One sample per block is above
 * the largest LEO delay rate (~25 us/s: 0.77 sample per 0.5 ms slot at 61.44 MS/s) and turns a step of the pass
 * clock (resync with the clock server) into a ramp. A change above NTN_DL_JUMP samples is applied at once: it is a
 * restart of the looped pass (QEF window ends differ, e.g. 78 us) or the start, where a ramp would leave the delay
 * off the model for seconds (OTA 2026-11-25: 5 s). Resolution: +-0.5 sample (+-16 ns at 30.72 MS/s).
 * A fractional (interpolating) line was rejected: a 4-tap cubic is -12 dB off at the edge of a 20 MHz carrier
 * sampled at 30.72 MS/s.
 *
 * Replaces the per-slot integer shift of the write timestamp, which made consecutive slots overlap or leave a
 * one-sample gap at every step of the delay (rfsim 2026-11-25: 90k overlaps over one pass) and, over the air,
 * had no effect: the X310 ignores the timestamps of packets inside a running burst (PSS timing of a whole pass
 * moved by the clock drift only, 2026-11-25).
 *
 * shortcut: one block at a time from a single thread (the L1 TX thread), as tx_rf() calls it.
 */
#ifndef NR_NTN_DELAY_LINE_H
#define NR_NTN_DELAY_LINE_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NTN_DL_MAX_ANT 8
#define NTN_DL_JUMP 64 /* samples: larger delay changes are pass restarts, not slewed */

typedef struct {
  int nb_ant;
  uint64_t len;                   /* ring length (samples) */
  uint32_t *ring[NTN_DL_MAX_ANT]; /* past input samples (one I/Q int16 pair each), per antenna */
  uint64_t wr;                    /* input samples written so far */
  int64_t v;                      /* delay applied (samples), -1 before the first block */
} ntn_delay_line_t;

/* max_v: largest delay (samples) ever requested; max_block: largest block (samples). */
static inline int ntn_dl_init(ntn_delay_line_t *dl, int nb_ant, double max_v, uint32_t max_block)
{
  if (nb_ant < 1 || nb_ant > NTN_DL_MAX_ANT || max_v < 0)
    return -1;
  memset(dl, 0, sizeof(*dl));
  dl->nb_ant = nb_ant;
  dl->len = (uint64_t)ceil(max_v) + 2 * (uint64_t)max_block + 2;
  dl->v = -1;
  for (int a = 0; a < nb_ant; a++)
    if (!(dl->ring[a] = (uint32_t *)calloc(dl->len, sizeof(uint32_t))))
      return -1;
  return 0;
}

static inline void ntn_dl_free(ntn_delay_line_t *dl)
{
  for (int a = 0; a < dl->nb_ant; a++)
    free(dl->ring[a]);
  memset(dl, 0, sizeof(*dl));
}

/*
 * One block of n samples per antenna (in[a], out[a]: n I/Q int16 pairs; they may be the same buffer, the
 * whole block enters the ring before any output is written).
 * v_target: wanted delay (samples). Returns the integer delay applied to this block.
 */
static inline int64_t ntn_dl_process(ntn_delay_line_t *dl, void *const in[], void *const out[], uint32_t n, double v_target)
{
  int64_t v = (int64_t)llround(v_target < 0 ? 0 : v_target);
  const int64_t vmax = (int64_t)dl->len - 2 * (int64_t)n;
  if (v > vmax)
    v = vmax;
  if (dl->v >= 0 && llabs(v - dl->v) <= NTN_DL_JUMP) {
    if (v > dl->v + 1)
      v = dl->v + 1;
    if (v < dl->v - 1)
      v = dl->v - 1;
  }
  const uint64_t L = dl->len, w0 = dl->wr;
  for (int a = 0; a < dl->nb_ant; a++) {
    uint32_t *r = dl->ring[a];
    const uint32_t *x = (const uint32_t *)in[a];
    uint32_t *y = (uint32_t *)out[a];
    for (uint32_t j = 0; j < n; j++)
      r[(w0 + j) % L] = x[j];
    for (uint32_t j = 0; j < n; j++) {
      const int64_t p = (int64_t)(w0 + j) - v; /* before the first input sample: silence */
      y[j] = p < 0 ? 0 : r[(uint64_t)p % L];
    }
  }
  dl->wr += n;
  dl->v = v;
  return v;
}

#endif
