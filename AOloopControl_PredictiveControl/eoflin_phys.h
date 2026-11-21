// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    eoflin_phys.h
 * @brief   Unified Phase-5 helpers for mkEOFPF: Pass-2 Filtered-X DM kernel, windlayers
 *          Fourier-space physics-Gram Bayesian prior, and regime-weighted Gram library.
 */

#ifndef CACAO_AOLOOPCONTROL_PC_EOFLIN_PHYS_H
#define CACAO_AOLOOPCONTROL_PC_EOFLIN_PHYS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define EOFPF_MAX_DM_TAPS  32
#define EOFPF_MAX_WL_LAYER 8
#define EOFPF_REG_MAX_K    16
#define EOFPF_REG_DIM      4

    typedef struct
    {
        int      mode;      /* 0=none, 1=1D [L], 2=2D diag [M,L], 3=3D MIMO [M,M,L] */
        uint32_t n_dyn;     /* Active kernel modes */
        uint32_t n_taps;    /* Causal impulse tap count L */
        float   *h_diag;    /* [L * n_dyn] for mode 1/2 */
        float   *h_mimo;    /* [L * n_dyn * n_dyn] for mode 3 */
        float    centroid;  /* Diagonal impulse centroid latency [frames] */
    } EOFPF_DM_KERN;

    typedef struct
    {
        int      ready;       /* 1 if dmmodes dual Fourier basis is initialized */
        int      has_layers;  /* 1 if valid wind layers have populated unit Grams */
        uint32_t n_modes;     /* Modes covered by physics prior (<= eff_modes) */
        uint32_t B;           /* Block size */
        uint32_t P;           /* EOF tap order */
        uint32_t nblocks;     /* Number of blocks covered */
        uint32_t npad;        /* 2D FFT padded grid size (power of 2) */
        uint32_t nf;          /* Half-plane frequency count npad * (npad/2 + 1) */
        double   dx;          /* DM actuator pitch on pupil [m] */
        double   l0_m;        /* von Karman outer scale [m] */
        double  *fx;          /* [nf] spatial frequency x [cycles/m] */
        double  *fy;          /* [nf] spatial frequency y [cycles/m] */
        double  *psd;         /* [nf] Hermitian-weighted von Karman PSD */
        float   *wf_re;       /* [n_modes * nf] real part of dual-basis 2D FFT */
        float   *wf_im;       /* [n_modes * nf] imag part of dual-basis 2D FFT */
        double **sxx_unit;    /* [nblocks] unit-scale Sxx [D * D] */
        double **syx_unit;    /* [nblocks] unit-scale Syx [b * D] */
        double  *c0_unit;     /* [n_modes] unit-scale lag-0 diagonal */
        double  *r1_unit;     /* [n_modes] unit-scale lag-1 diagonal */
        double   vbar_x;      /* Last Cn2-weighted mean wind vx [m/s] */
        double   vbar_y;      /* Last Cn2-weighted mean wind vy [m/s] */
        double   v_std;       /* Last Cn2-weighted wind spread [m/s] */
        int      n_layers;    /* Active wind layer count */
    } EOFPF_PHYS_PRIOR;

    typedef struct
    {
        uint32_t nblocks;
        uint32_t B;
        uint32_t P;
        uint32_t eff_modes;
        uint32_t k_max;    /* Max regime anchors (1..EOFPF_REG_MAX_K) */
        uint32_t k_used;   /* Currently populated anchors */
        uint32_t cur_k;    /* Last assigned anchor index */
        double   desc[EOFPF_REG_MAX_K][EOFPF_REG_DIM];
        double   w_eff[EOFPF_REG_MAX_K];
        double   y2_sum[EOFPF_REG_MAX_K];
        double   zoh2_sum[EOFPF_REG_MAX_K];
        double  *G[EOFPF_REG_MAX_K]; /* Flat [sum_blk D_blk*D_blk] per anchor */
        double  *R[EOFPF_REG_MAX_K]; /* Flat [sum_blk D_blk*b_blk] per anchor */
        size_t  *g_off;              /* [nblocks] offset into G[k] */
        size_t  *r_off;              /* [nblocks] offset into R[k] */
        size_t   g_tot;
        size_t   r_tot;
    } EOFPF_GRAM_STATE;

    /* Pass-2 DM impulse kernel (Filtered-X) */
    int  eofpf_dm_kern_load(const char *path, uint32_t nbmode, EOFPF_DM_KERN *k);
    void eofpf_dm_kern_free(EOFPF_DM_KERN *k);
    void eofpf_dm_kern_convolve(const EOFPF_DM_KERN *k,
                                const float         *in_fm,
                                uint32_t             n_frames,
                                uint32_t             nbmode,
                                uint32_t             t_start,
                                uint32_t             t_end,
                                float                lat_fb,
                                float               *out_fm);

    /* windlayers physics-Gram Bayesian prior */
    int  eofpf_phys_init(EOFPF_PHYS_PRIOR *pp,
                         const char       *dmmodes_path,
                         uint32_t          eff_modes,
                         uint32_t          B,
                         uint32_t          P,
                         float             pup_diam,
                         float             dm_dx,
                         float             l0_m);
    void eofpf_phys_free(EOFPF_PHYS_PRIOR *pp);
    int  eofpf_phys_update_layers(EOFPF_PHYS_PRIOR *pp,
                                  const float      *wl_tbl,
                                  uint32_t          ncol,
                                  uint32_t          nrow,
                                  float             lat_fr,
                                  float             frate_hz,
                                  int              *wind_switched_out);
    void eofpf_phys_blend_block(const EOFPF_PHYS_PRIOR *pp,
                                uint32_t                blk,
                                const double           *G_emp,
                                const double           *R_emp,
                                double                  w_emp,
                                double                  n_prior,
                                double                 *G_out,
                                double                 *R_out,
                                double                 *prior_y2_out);

    /* Extended-memory & regime-weighted Gram accumulator */
    int  eofpf_gram_init(EOFPF_GRAM_STATE *gs,
                         uint32_t          eff_modes,
                         uint32_t          B,
                         uint32_t          P,
                         uint32_t          k_max);
    void eofpf_gram_free(EOFPF_GRAM_STATE *gs);
    void eofpf_gram_discount_all(EOFPF_GRAM_STATE *gs, double factor);
    uint32_t eofpf_gram_select_anchor(EOFPF_GRAM_STATE *gs,
                                      const double      desc_now[EOFPF_REG_DIM],
                                      double            rlim);
    void eofpf_gram_accum_block(EOFPF_GRAM_STATE *gs,
                                uint32_t          anchor,
                                uint32_t          blk,
                                const double     *dG,
                                const double     *dR,
                                double            decay);
    void eofpf_gram_blend_block(const EOFPF_GRAM_STATE *gs,
                                uint32_t                blk,
                                const double            desc_now[EOFPF_REG_DIM],
                                double                  kappa,
                                double                  bw,
                                double                 *G_out,
                                double                 *R_out,
                                double                 *w_out);

#ifdef __cplusplus
}
#endif

#endif /* CACAO_AOLOOPCONTROL_PC_EOFLIN_PHYS_H */
