/*
 * Filter Library — Moving Average / FIR / Biquad / LPF
 *
 * MATLAB integration ready. See comments for coefficient generation.
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_FILTER_H
#define ALGO_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ALGO_RAMFUNC
#define ALGO_RAMFUNC __attribute__((section(".fast")))
#endif

#define ALGO_PI_F 3.14159265358979323846f

/* ── IEEE-754 finite check (bit-level, survives -ffast-math) ──────────── */

ALGO_RAMFUNC
static inline bool algo_flt_finite(float x)
{
    union { float f; uint32_t u; } v = { .f = x };
    return (v.u & 0x7F800000u) != 0x7F800000u;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Moving Average  —  滑动平均 (O(1) running sum, 乘法代替除法)
 *
 *  MATLAB:  y = movmean(x, N)
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct algo_ma algo_ma_t;

typedef struct {
    uint16_t window_size;
    float   *buffer;
} algo_ma_cfg_t;

typedef int   (*algo_ma_init_fn)(algo_ma_t *s, const algo_ma_cfg_t *c);
typedef float (*algo_ma_step_fn)(algo_ma_t *s, float x);
typedef void  (*algo_ma_reset_fn)(algo_ma_t *s);

struct algo_ma {
    struct {
        algo_ma_init_fn  init;
        algo_ma_step_fn  step;
        algo_ma_reset_fn reset;
    };

    float    *_buf;
    uint16_t  _size;
    float     _inv_size;
    uint16_t  _idx;
    float     _sum;
    bool      _filled;
    bool      _inited;
};

void algo_ma_ctor(algo_ma_t *s);

ALGO_RAMFUNC
static inline float algo_ma_step_fast(algo_ma_t *s, float x)
{
    if (!s->_inited) return 0.0f;

    if (!algo_flt_finite(x)) {
        return s->_filled ? (s->_sum * s->_inv_size)
                          : (s->_sum / (float)((s->_idx > 0U) ? s->_idx : 1U));
    }

    float old = s->_buf[s->_idx];
    s->_buf[s->_idx] = x;

    s->_idx++;
    if (s->_idx >= s->_size) s->_idx = 0;

    if (s->_filled) {
        s->_sum += x - old;
        return s->_sum * s->_inv_size;
    }

    s->_sum += x;
    if (s->_idx == 0U) {
        s->_filled = true;
    }
    return s->_sum / (float)((s->_idx > 0U) ? s->_idx : 1U);
}

/* ════════════════════════════════════════════════════════════════════════
 *  1st-Order Low-Pass  —  一阶 IIR 低通
 *
 *     alpha = Ts / (Ts + 1/(2*pi*fc))
 *     y[n]  = alpha * x[n] + (1-alpha) * y[n-1]
 *
 *  首次有效输入直接输出（priming），避免从 0 爬升。
 *
 *  MATLAB:  y = lowpass(x, fc, fs)
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct algo_lpf algo_lpf_t;

typedef struct {
    float cutoff_hz;
    float sample_rate_hz;
} algo_lpf_cfg_t;

typedef int   (*algo_lpf_init_fn)(algo_lpf_t *s, const algo_lpf_cfg_t *c);
typedef float (*algo_lpf_step_fn)(algo_lpf_t *s, float x);
typedef void  (*algo_lpf_reset_fn)(algo_lpf_t *s);

struct algo_lpf {
    struct {
        algo_lpf_init_fn  init;
        algo_lpf_step_fn  step;
        algo_lpf_reset_fn reset;
    };

    float _alpha;
    float _y;
    bool  _primed;
    bool  _inited;
};

void algo_lpf_ctor(algo_lpf_t *s);

ALGO_RAMFUNC
static inline float algo_lpf_step_fast(algo_lpf_t *s, float x)
{
    if (!s->_inited) return 0.0f;

    if (!algo_flt_finite(x)) return s->_y;

    if (!s->_primed) {
        s->_y = x;
        s->_primed = true;
        return x;
    }

    s->_y += s->_alpha * (x - s->_y);
    return s->_y;
}

/* ════════════════════════════════════════════════════════════════════════
 *  FIR  —  有限冲激响应
 *
 *     y[n] = Σ h[k]·x[n−k],  k = 0 … N−1
 *
 *  MATLAB:
 *     h = fir1(N-1, fc/(fs/2));                % 窗函数法
 *     h = firpm(N-1, [0 f1 f2 1], [1 1 0 0]);  % Parks-McClellan
 *     → const float fir_coeffs[N] = { … };
 *     → algo_fir_cfg_t  .coeffs = fir_coeffs
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct algo_fir algo_fir_t;

typedef struct {
    const float *coeffs;
    uint16_t     num_taps;
    float       *buffer;
} algo_fir_cfg_t;

typedef int   (*algo_fir_init_fn)(algo_fir_t *s, const algo_fir_cfg_t *c);
typedef float (*algo_fir_step_fn)(algo_fir_t *s, float x);
typedef void  (*algo_fir_reset_fn)(algo_fir_t *s);

struct algo_fir {
    struct {
        algo_fir_init_fn  init;
        algo_fir_step_fn  step;
        algo_fir_reset_fn reset;
    };

    const float *_coeffs;
    float       *_buf;
    uint16_t     _taps;
    uint16_t     _idx;
    float        _y;
    bool         _inited;
};

void algo_fir_ctor(algo_fir_t *s);

/* ════════════════════════════════════════════════════════════════════════
 *  Biquad  —  二阶 IIR (Direct Form II Transposed)
 *
 *     y[n]   = b0·x[n] + z1[n−1]
 *     z1[n]  = b1·x[n] − a1·y[n] + z2[n−1]
 *     z2[n]  = b2·x[n] − a2·y[n]
 *
 *  假定 a0 已归一化为 1（MATLAB 输出 [b,a] 后除以 a(1)）。
 *
 *  MATLAB:
 *     [b,a] = butter(2, fc/(fs/2));
 *     b = b / a(1);  a = a / a(1);
 *     → algo_biquad_coeffs_t = {b(1),b(2),b(3), a(2),a(3)}
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    float b0, b1, b2;
    float a1, a2;
} algo_biquad_coeffs_t;

typedef struct algo_biquad algo_biquad_t;

typedef struct {
    algo_biquad_coeffs_t coeffs;
} algo_biquad_cfg_t;

typedef int   (*algo_biquad_init_fn)(algo_biquad_t *s, const algo_biquad_cfg_t *c);
typedef float (*algo_biquad_step_fn)(algo_biquad_t *s, float x);
typedef void  (*algo_biquad_reset_fn)(algo_biquad_t *s);

struct algo_biquad {
    struct {
        algo_biquad_init_fn  init;
        algo_biquad_step_fn  step;
        algo_biquad_reset_fn reset;
    };

    float _b0, _b1, _b2;
    float _a1, _a2;
    float _z1, _z2;
    float _y;
    bool  _inited;
};

void algo_biquad_ctor(algo_biquad_t *s);

ALGO_RAMFUNC
static inline float algo_biquad_step_fast(algo_biquad_t *s, float x)
{
    if (!s->_inited) return 0.0f;

    if (!algo_flt_finite(x)) return s->_y;

    float y  = s->_b0 * x + s->_z1;
    s->_z1   = s->_b1 * x - s->_a1 * y + s->_z2;
    s->_z2   = s->_b2 * x - s->_a2 * y;
    s->_y    = y;
    return y;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Median  —  滑动窗中值滤波
 *
 *     y[n] = median{ x[n−k] },  k = 0 … N−1
 *
 *  对脉冲噪声（尖峰、毛刺）鲁棒性优于均值滤波。
 *  使用插入排序 O(N²·per step)，适用于小窗口 (N ≤ 31)。
 *
 *  MATLAB:  y = medfilt1(x, N)
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct algo_med algo_med_t;

typedef struct {
    uint16_t window_size;
    float   *buffer;
    float   *sort_buf;
} algo_med_cfg_t;

typedef int   (*algo_med_init_fn)(algo_med_t *s, const algo_med_cfg_t *c);
typedef float (*algo_med_step_fn)(algo_med_t *s, float x);
typedef void  (*algo_med_reset_fn)(algo_med_t *s);

struct algo_med {
    struct {
        algo_med_init_fn  init;
        algo_med_step_fn  step;
        algo_med_reset_fn reset;
    };

    float    *_buf;
    float    *_sort;
    uint16_t  _size;
    uint16_t  _idx;
    uint16_t  _count;
    float     _y;
    bool      _inited;
};

void algo_med_ctor(algo_med_t *s);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_FILTER_H */
