// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    pyWFSgridmatch.c
 * @brief   Match a Pyramid WFS zonal response matrix (`zrespM`) to a 4-pupil affine DM grid
 *
 * Locates the 4 response peaks of each active DM actuator across the 4 WFS quadrants and solves
 * the exact weighted linear least-squares affine mapping:
 *   x_q(u, v) = xc_q + Axx * u + Axy * v
 *   y_q(u, v) = yc_q + Ayx * u + Ayy * v
 * where (u, v) are centered DM coordinates, q in {0=BL, 1=TL, 2=BR, 3=TR}.
 * Optionally publishes the resulting [4 x 4] nominal geometry stream (`[xc, yc, rout, 0]`) for
 * `pyWFStlm` and `pyrwfs.mkzones`.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "fps_cli_sync.h"
#include "processinfo.h"

#include "pyramidWFStools.h"
#include "pyrwfs_tlm_core.h"

/* ================================================================
 * 1.  PARAMETERS
 * ============================================================= */

static char     inimname[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static char     outgeomname[FUNCTION_PARAMETER_STRMAXLEN] = "";
static double   spotsize                                 = 2.0;
static uint32_t dmxsize                                  = 0;
static uint32_t dmysize                                  = 0;
static float    actthr                                   = 0.10f;

static float o_center_x   = 0.0f;
static float o_center_y   = 0.0f;
static float o_sep_x      = 0.0f;
static float o_sep_y      = 0.0f;
static float o_pup_angle  = 0.0f;
static float o_actpitch_x = 0.0f;
static float o_actpitch_y = 0.0f;
static float o_act_angle  = 0.0f;
static float o_rout_est   = 0.0f;
static float o_fit_rms    = 0.0f;
static int32_t o_nact_used = 0;

#define FPS_PARAMS_GRIDMATCH(X)                                                                  \
    X(".in_name", inimname, FPTYPE_STREAMNAME, 1,                                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_PRIMARY_CLI_INPUT, "input zonal RM cube (zrespM)")           \
    X(".spotsize", &spotsize, FPTYPE_FLOAT64, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "spot smoothing & centroid radius [pix]")                                                  \
    X(".dmxsize", &dmxsize, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "DM x size [actuators]")                                                                   \
    X(".dmysize", &dmysize, FPTYPE_UINT32, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,           \
      "DM y size [actuators]")                                                                   \
    X(".actthr", &actthr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,            \
      "active actuator power threshold fraction")                                                \
    X(".outgeom", outgeomname, FPTYPE_STRING, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "optional output [4x4] nomgeom stream")                                                    \
    X(".out.center_x", &o_center_x, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "4-pupil center X [pix]")                                                                  \
    X(".out.center_y", &o_center_y, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "4-pupil center Y [pix]")                                                                  \
    X(".out.sep_x", &o_sep_x, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "pupil half-separation X [pix]")                                                           \
    X(".out.sep_y", &o_sep_y, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                          \
      "pupil half-separation Y [pix]")                                                           \
    X(".out.pup_angle", &o_pup_angle, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "pupil square angle [rad]")                                                                \
    X(".out.actpitch_x", &o_actpitch_x, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "DM actuator pitch X on WFS [pix/act]")                                                    \
    X(".out.actpitch_y", &o_actpitch_y, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "DM actuator pitch Y on WFS [pix/act]")                                                    \
    X(".out.act_angle", &o_act_angle, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                  \
      "DM grid rotation angle [rad]")                                                            \
    X(".out.rout_est", &o_rout_est, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "estimated outer pupil radius [pix]")                                                      \
    X(".out.fit_rms", &o_fit_rms, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                      \
      "weighted spot fit RMS residual [pix]")                                                    \
    X(".out.nact_used", &o_nact_used, FPTYPE_INT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "number of illuminated actuators fitted")

MILK_PARAMS_DEFINE(pyWFSgridmatch, FPS_PARAMS_GRIDMATCH);

/* ================================================================
 * 2.  EXACT 6x6 LINEAR SOLVER (GAUSSIAN ELIMINATION WITH PIVOTING)
 * ============================================================= */

static int solve_6x6(double A[6][6], double b[6], double x[6])
{
    double M[6][7];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            M[i][j] = A[i][j];
        }
        M[i][6] = b[i];
    }

    for (int col = 0; col < 6; col++)
    {
        int    piv = col;
        double am  = fabs(M[col][col]);
        for (int r = col + 1; r < 6; r++)
        {
            double v = fabs(M[r][col]);
            if (v > am)
            {
                am  = v;
                piv = r;
            }
        }
        if (am < 1e-12)
        {
            return -1;
        }
        if (piv != col)
        {
            for (int j = col; j < 7; j++)
            {
                double tmp = M[col][j];
                M[col][j]  = M[piv][j];
                M[piv][j]  = tmp;
            }
        }
        double diag = M[col][col];
        for (int r = col + 1; r < 6; r++)
        {
            double f = M[r][col] / diag;
            for (int j = col; j < 7; j++)
            {
                M[r][j] -= f * M[col][j];
            }
        }
    }

    for (int i = 5; i >= 0; i--)
    {
        double s = M[i][6];
        for (int j = i + 1; j < 6; j++)
        {
            s -= M[i][j] * x[j];
        }
        x[i] = s / M[i][i];
    }
    return 0;
}

/* ================================================================
 * 3.  COMPUTE ENTRY POINT
 * ============================================================= */

static MILK_HOT errno_t pyWFSgridmatch_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    IMGID zrmimg = imgid_make_from_name(inimname);
    resolveIMGID(&zrmimg, ERRMODE_WARN, dcimg, dcnimg);
    if (zrmimg.ID == -1)
    {
        return RETURN_FAILURE;
    }

    if (zrmimg.md->naxis < 3 || dmxsize == 0u || dmysize == 0u ||
        zrmimg.md->size[2] < dmxsize * dmysize)
    {
        printf("[pyWFSgridmatch] ERROR: invalid zrespM dims (%ux%ux%u) for DM %ux%u\n",
               zrmimg.md->size[0], zrmimg.md->size[1],
               (zrmimg.md->naxis >= 3) ? zrmimg.md->size[2] : 0u, dmxsize, dmysize);
        return RETURN_FAILURE;
    }

    errno_t ret = RETURN_SUCCESS;

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        const uint32_t xsize  = zrmimg.md->size[0];
        const uint32_t ysize  = zrmimg.md->size[1];
        const size_t   xysize = (size_t) xsize * ysize;
        const size_t   nact   = (size_t) dmxsize * dmysize;

        float  *imarray    = (float *) malloc(xysize * sizeof(float));
        float  *imcarray   = (float *) malloc(xysize * sizeof(float));
        double *pixval2sum = (double *) malloc(nact * sizeof(double));
        float  *spotpos_x  = (float *) calloc(nact * 4u, sizeof(float));
        float  *spotpos_y  = (float *) calloc(nact * 4u, sizeof(float));
        float  *spotval    = (float *) calloc(nact * 4u, sizeof(float));

        if (!imarray || !imcarray || !pixval2sum || !spotpos_x || !spotpos_y || !spotval)
        {
            free(imarray);
            free(imcarray);
            free(pixval2sum);
            free(spotpos_x);
            free(spotpos_y);
            free(spotval);
            ret           = RETURN_FAILURE;
            processloopOK = 0;
            break;
        }

        int ksizeint = (int) lround(spotsize);
        if (ksizeint < 1)
        {
            ksizeint = 1;
        }
        double c_rad = (spotsize + 1.0 > 2.5) ? (spotsize + 1.0) : 2.5;

        /* 1. Compute actuator signal power across slices */
        double pixval2sum_max = 0.0;
        for (size_t slice = 0; slice < nact; slice++)
        {
            double       acc = 0.0;
            const float *src = zrmimg.im->array.F + slice * xysize;
            for (size_t p = 0; p < xysize; p++)
            {
                double v = (double) src[p];
                acc += v * v;
            }
            pixval2sum[slice] = acc;
            if (acc > pixval2sum_max)
            {
                pixval2sum_max = acc;
            }
        }

        double thr_val = (double) actthr * pixval2sum_max;
        int    nused   = 0;
        double umax_r  = 0.0;

        /* 2. Detect 4 quadrant response spot centroids per illuminated DM actuator */
        const double dm_cx = 0.5 * (double) (dmxsize - 1u);
        const double dm_cy = 0.5 * (double) (dmysize - 1u);

        for (uint32_t dmjj = 0; dmjj < dmysize; dmjj++)
        {
            for (uint32_t dmii = 0; dmii < dmxsize; dmii++)
            {
                size_t slice = (size_t) dmjj * dmxsize + dmii;
                if (pixval2sum[slice] <= thr_val)
                {
                    continue;
                }

                const float *src = zrmimg.im->array.F + slice * xysize;
                for (size_t p = 0; p < xysize; p++)
                {
                    imarray[p] = src[p] * src[p];
                }

                /* Box-filter squared response */
                for (uint32_t jj = 0; jj < ysize; jj++)
                {
                    int jj1min = ((int) jj - ksizeint > 0) ? ((int) jj - ksizeint) : 0;
                    int jj1max =
                        ((int) jj + ksizeint + 1 < (int) ysize) ? ((int) jj + ksizeint + 1) : (int) ysize;
                    for (uint32_t ii = 0; ii < xsize; ii++)
                    {
                        int ii1min = ((int) ii - ksizeint > 0) ? ((int) ii - ksizeint) : 0;
                        int ii1max = ((int) ii + ksizeint + 1 < (int) xsize)
                                         ? ((int) ii + ksizeint + 1)
                                         : (int) xsize;
                        float acc = 0.0f;
                        for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                        {
                            const float *row = imarray + (size_t) jj1 * xsize;
                            for (int ii1 = ii1min; ii1 < ii1max; ii1++)
                            {
                                acc += row[ii1];
                            }
                        }
                        imcarray[(size_t) jj * xsize + ii] = acc;
                    }
                }

                for (int q = 0; q < PYRWFS_NPUP; q++)
                {
                    uint32_t iistart = (q == 0 || q == 1) ? 0u : (xsize / 2u);
                    uint32_t iistop  = (q == 0 || q == 1) ? (xsize / 2u) : xsize;
                    uint32_t jjstart = (q == 0 || q == 2) ? 0u : (ysize / 2u);
                    uint32_t jjstop  = (q == 0 || q == 2) ? (ysize / 2u) : ysize;

                    float    vpeak  = -1.0f;
                    uint32_t iipeak = iistart;
                    uint32_t jjpeak = jjstart;
                    for (uint32_t jj = jjstart; jj < jjstop; jj++)
                    {
                        const float *row = imcarray + (size_t) jj * xsize;
                        for (uint32_t ii = iistart; ii < iistop; ii++)
                        {
                            if (row[ii] > vpeak)
                            {
                                vpeak  = row[ii];
                                iipeak = ii;
                                jjpeak = jj;
                            }
                        }
                    }

                    /* Sub-pixel photocenter on imarray around peak */
                    int    rpad   = (int) ceil(c_rad) + 1;
                    int    ii1min = ((int) iipeak - rpad > (int) iistart) ? ((int) iipeak - rpad)
                                                                          : (int) iistart;
                    int    ii1max = ((int) iipeak + rpad + 1 < (int) iistop)
                                        ? ((int) iipeak + rpad + 1)
                                        : (int) iistop;
                    int    jj1min = ((int) jjpeak - rpad > (int) jjstart) ? ((int) jjpeak - rpad)
                                                                          : (int) jjstart;
                    int    jj1max = ((int) jjpeak + rpad + 1 < (int) jjstop)
                                        ? ((int) jjpeak + rpad + 1)
                                        : (int) jjstop;
                    double xsum   = 0.0;
                    double ysum   = 0.0;
                    double wsum   = 0.0;
                    for (int jj1 = jj1min; jj1 < jj1max; jj1++)
                    {
                        double       dy  = (double) jj1 - (double) jjpeak;
                        const float *row = imarray + (size_t) jj1 * xsize;
                        for (int ii1 = ii1min; ii1 < ii1max; ii1++)
                        {
                            double dx = (double) ii1 - (double) iipeak;
                            if (dx * dx + dy * dy <= c_rad * c_rad)
                            {
                                double w = (double) row[ii1];
                                xsum += w * (double) ii1;
                                ysum += w * (double) jj1;
                                wsum += w;
                            }
                        }
                    }
                    if (wsum > 0.0)
                    {
                        spotpos_x[slice * 4u + (size_t) q] = (float) (xsum / wsum);
                        spotpos_y[slice * 4u + (size_t) q] = (float) (ysum / wsum);
                        spotval[slice * 4u + (size_t) q]   = (float) wsum;
                    }
                }

                double ur = hypot((double) dmii - dm_cx, (double) dmjj - dm_cy);
                if (ur > umax_r)
                {
                    umax_r = ur;
                }
                nused++;
            }
        }

        /* 3. Exact weighted linear least-squares fit:
         *    theta = [c_0, c_1, c_2, c_3, A_u, A_v] separately for X and Y */
        double Nx[6][6], Ny[6][6];
        double bx[6], by[6];
        memset(Nx, 0, sizeof(Nx));
        memset(Ny, 0, sizeof(Ny));
        memset(bx, 0, sizeof(bx));
        memset(by, 0, sizeof(by));

        for (uint32_t dmjj = 0; dmjj < dmysize; dmjj++)
        {
            double v = (double) dmjj - dm_cy;
            for (uint32_t dmii = 0; dmii < dmxsize; dmii++)
            {
                double u     = (double) dmii - dm_cx;
                size_t slice = (size_t) dmjj * dmxsize + dmii;
                for (int q = 0; q < PYRWFS_NPUP; q++)
                {
                    double w = (double) spotval[slice * 4u + (size_t) q];
                    if (w <= 0.0)
                    {
                        continue;
                    }
                    double phi[6] = { 0.0, 0.0, 0.0, 0.0, u, v };
                    phi[q]        = 1.0;
                    double sx     = (double) spotpos_x[slice * 4u + (size_t) q];
                    double sy     = (double) spotpos_y[slice * 4u + (size_t) q];

                    for (int a = 0; a < 6; a++)
                    {
                        if (phi[a] == 0.0)
                        {
                            continue;
                        }
                        bx[a] += w * phi[a] * sx;
                        by[a] += w * phi[a] * sy;
                        for (int b = 0; b < 6; b++)
                        {
                            if (phi[b] != 0.0)
                            {
                                double contrib = w * phi[a] * phi[b];
                                Nx[a][b] += contrib;
                                Ny[a][b] += contrib;
                            }
                        }
                    }
                }
            }
        }

        double tx[6], ty[6];
        if (nused < 4 || solve_6x6(Nx, bx, tx) != 0 || solve_6x6(Ny, by, ty) != 0)
        {
            printf("[pyWFSgridmatch] ERROR: underdetermined DM-to-WFS grid fit (nused=%d)\n", nused);
            ret = RETURN_FAILURE;
        }
        else
        {
            /* Compute weighted RMS fit residual */
            double err2 = 0.0;
            double wtot = 0.0;
            for (uint32_t dmjj = 0; dmjj < dmysize; dmjj++)
            {
                double v = (double) dmjj - dm_cy;
                for (uint32_t dmii = 0; dmii < dmxsize; dmii++)
                {
                    double u     = (double) dmii - dm_cx;
                    size_t slice = (size_t) dmjj * dmxsize + dmii;
                    for (int q = 0; q < PYRWFS_NPUP; q++)
                    {
                        double w = (double) spotval[slice * 4u + (size_t) q];
                        if (w <= 0.0)
                        {
                            continue;
                        }
                        double px = tx[q] + tx[4] * u + tx[5] * v;
                        double py = ty[q] + ty[4] * u + ty[5] * v;
                        double ex = (double) spotpos_x[slice * 4u + (size_t) q] - px;
                        double ey = (double) spotpos_y[slice * 4u + (size_t) q] - py;
                        err2 += w * (ex * ex + ey * ey);
                        wtot += w;
                    }
                }
            }

            double cx      = 0.25 * (tx[0] + tx[1] + tx[2] + tx[3]);
            double cy      = 0.25 * (ty[0] + ty[1] + ty[2] + ty[3]);
            double sep_x   = 0.25 * ((tx[2] + tx[3]) - (tx[0] + tx[1]));
            double sep_y   = 0.25 * ((ty[1] + ty[3]) - (ty[0] + ty[2]));
            double pup_ang = 0.5 * (atan2((ty[2] + ty[3]) - (ty[0] + ty[1]),
                                          (tx[2] + tx[3]) - (tx[0] + tx[1])) +
                                    atan2(-((tx[1] + tx[3]) - (tx[0] + tx[2])),
                                          (ty[1] + ty[3]) - (ty[0] + ty[2])));
            double pitch_x = hypot(tx[4], ty[4]);
            double pitch_y = hypot(tx[5], ty[5]);
            double act_ang = atan2(ty[4], tx[4]);
            double rout    = (umax_r + 0.5) * 0.5 * (pitch_x + pitch_y);

            o_center_x   = (float) cx;
            o_center_y   = (float) cy;
            o_sep_x      = (float) sep_x;
            o_sep_y      = (float) sep_y;
            o_pup_angle  = (float) pup_ang;
            o_actpitch_x = (float) pitch_x;
            o_actpitch_y = (float) pitch_y;
            o_act_angle  = (float) act_ang;
            o_rout_est   = (float) rout;
            o_fit_rms    = (wtot > 0.0) ? (float) sqrt(err2 / wtot) : 0.0f;
            o_nact_used  = nused;

            printf("[pyWFSgridmatch] nact=%d center=(%.2f, %.2f) sep=(%.2f, %.2f) "
                   "pitch=(%.3f, %.3f) rot=%.4f rad rout=%.2f rms=%.3f pix\n",
                   nused, cx, cy, sep_x, sep_y, pitch_x, pitch_y, act_ang, rout, o_fit_rms);

            if (outgeomname[0] != '\0')
            {
                IMGID og = stream_connect_create_2Df32(outgeomname, 4u, PYRWFS_NPUP);
                if (og.ID != -1)
                {
                    og.md->write = 1;
                    for (int q = 0; q < PYRWFS_NPUP; q++)
                    {
                        float *row = og.im->array.F + (size_t) q * 4u;
                        row[0]     = (float) tx[q];
                        row[1]     = (float) ty[q];
                        row[2]     = (float) rout;
                        row[3]     = 0.0f;
                    }
                    processinfo_update_output_stream(processinfo, og.im, zrmimg.im);
                }
            }
        }

        free(pixval2sum);
        free(imcarray);
        free(imarray);
        free(spotpos_x);
        free(spotpos_y);
        free(spotval);

        processloopOK = 0;
    }
    MILK_PROCINFO_END(self)

    if (dcfpsptr != NULL)
    {
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    DEBUG_TRACE_FEXIT();
    return ret;
}

const MILK_RUNNABLE runnable_pyramidWFStools_pyWFSgridmatch = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "pyWFSgridmatch",
                 .cmdkey           = "pyWFSgridmatch",
                 .description      = "Match pyramid WFS zrespM to 4-pupil DM affine grid",
                 .description_long = "Fits an exact weighted linear least-squares 4-pupil + DM "
                                     "affine grid model to a pyramid WFS zonal response matrix "
                                     "and optionally exports [4x4] nominal pupil geometry." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(pyWFSgridmatch),
    .compute = pyWFSgridmatch_compute,
};
EXPORT_FPSEXEC(cacao, "pywfsgridmatch", runnable_pyramidWFStools_pyWFSgridmatch);
