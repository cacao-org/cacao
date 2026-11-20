// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    PyWFSsim.c
 * @brief   cheap pyramid WFS simulation: DM slopes -> four pupil images
 *
 * Input DM map Nx x Ny, output 2Nx x 2Ny, one pixel per actuator, four quadrants touching.
 * The DM map is OPD in microns. Forward differences gx, gy [um per actuator] (0 at the edge
 * and next to an unlit actuator) are converted to tilt in lambda/D, taking the grid as the
 * pupil diameter: tx = gx Nx / wl_um, ty = gy Ny / wl_um. They saturate as
 * s = u / sqrt(1 + u^2), u = t / modamp. Quadrant (qx, qy), ex = 2qx - 1, ey = 2qy - 1:
 *   out[i + qx Nx, j + qy Ny] = lit[i, j] / (4 Npup) (1 + ex sx[i, j]) (1 + ey sy[i, j])
 * The output is >= 0 and sums to 1; piston is unseen. No diffraction, noise or flux scaling.
 */
#include <float.h>
#include <math.h>

#include "milk_runnable.h"

#include "ImageStreamIO/ImageStruct.h"

#include "COREMOD_memory/COREMOD_memory.h"

// Local variables
static char  dmstream[FUNCTION_PARAMETER_STRMAXLEN];
static char  wfssignal_out[FUNCTION_PARAMETER_STRMAXLEN];
static float modamp = 1.0f;
static float wl_um  = 1.0f;
static char  pupmask[FUNCTION_PARAMETER_STRMAXLEN];

#define FPS_PARAMS(X)                                                                       \
    X(".dmstream", dmstream, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_TRIGGER_STREAM,           \
      "input DM map (trigger)")                                                             \
    X(".wfssignal", wfssignal_out, FPTYPE_STREAMNAME, 1,                                    \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output pupil images, 2Nx x 2Ny, total 1") \
    X(".modamp", &modamp, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,                          \
      "modulation radius, tilt at which the response saturates [lambda/D]")                 \
    X(".wlum", &wl_um, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT, "wavelength [um]")          \
    X(".pupmask", pupmask, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "pupil mask Nx x Ny, lit where > 0; empty: all lit")

MILK_PARAMS_DEFINE(PyWFSsim, FPS_PARAMS);


// modamp, wlum: writable while running, refused when <= 0 (checked after this callback)
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
        long fpi_modamp = functionparameter_GetParamIndex(milk_data.fpsptr, ".modamp");
        long fpi_wlum   = functionparameter_GetParamIndex(milk_data.fpsptr, ".wlum");

        if (fpi_modamp > -1)
        {
            milk_data.fpsptr->parray[fpi_modamp].fpflag |= (FPFLAG_WRITERUN | FPFLAG_MINLIMIT);
            milk_data.fpsptr->parray[fpi_modamp].val.f32[1] = FLT_MIN;
        }
        if (fpi_wlum > -1)
        {
            milk_data.fpsptr->parray[fpi_wlum].fpflag |= (FPFLAG_WRITERUN | FPFLAG_MINLIMIT);
            milk_data.fpsptr->parray[fpi_wlum].val.f32[1] = FLT_MIN;
        }
    }
    return RETURN_SUCCESS;
}


static inline float pywfs_sat(float u)
{
    return u / sqrtf(1.0f + u * u);
}

// One output frame from the DM map; lit[] is Nx x Ny, 0 or 1, Npup lit pixels
static void pywfs_frame(const float   *dm,
                        const uint8_t *lit,
                        float         *out,
                        uint32_t       Nx,
                        uint32_t       Ny,
                        uint64_t       Npup)
{
    const float    norm = 1.0f / (4.0f * (float) Npup);
    const float    kx   = (float) Nx / (wl_um * modamp); // um per actuator -> u
    const float    ky   = (float) Ny / (wl_um * modamp);
    const uint32_t Ox   = 2 * Nx; // output row length

    for (uint32_t j = 0; j < Ny; j++)
    {
        for (uint32_t i = 0; i < Nx; i++)
        {
            uint64_t k  = (uint64_t) j * Nx + i;
            float    gx = 0.0f;
            float    gy = 0.0f;
            if (i + 1 < Nx && lit[k] && lit[k + 1])
            {
                gx = dm[k + 1] - dm[k];
            }
            if (j + 1 < Ny && lit[k] && lit[k + Nx])
            {
                gy = dm[k + Nx] - dm[k];
            }
            float sx = pywfs_sat(gx * kx);
            float sy = pywfs_sat(gy * ky);
            float v  = lit[k] ? norm : 0.0f;

            uint64_t lo  = (uint64_t) j * Ox + i;        // qy = 0
            uint64_t hi  = (uint64_t) (j + Ny) * Ox + i; // qy = 1
            out[lo]      = v * (1.0f - sx) * (1.0f - sy);
            out[lo + Nx] = v * (1.0f + sx) * (1.0f - sy);
            out[hi]      = v * (1.0f - sx) * (1.0f + sy);
            out[hi + Nx] = v * (1.0f + sx) * (1.0f + sy);
        }
    }
}


static MILK_HOT errno_t PyWFSsim_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    IMGID dmimg = stream_connect(dmstream);
    if (dmimg.ID == -1 || dmimg.md->datatype != _DATATYPE_FLOAT)
    {
        PRINT_ERROR("%s: DM stream missing or not float32", dmstream);
        imgid_free(&dmimg);
        return RETURN_FAILURE;
    }
    uint32_t Nx   = dmimg.md->size[0];
    uint32_t Ny   = dmimg.md->size[1];
    uint64_t Nact = (uint64_t) Nx * Ny;

    // Mask resolved once; name set but stream missing: warning, all lit
    IMGID mask = (pupmask[0] != '\0') ? stream_connect(pupmask) : imgid_make_from_name("");
    if (mask.ID != -1 &&
        (mask.md->size[0] != Nx || mask.md->size[1] != Ny || mask.md->datatype != _DATATYPE_FLOAT))
    {
        PRINT_ERROR("%s: pupil mask must be float32 %u x %u, as the DM", pupmask, Nx, Ny);
        imgid_free(&dmimg);
        imgid_free(&mask);
        return RETURN_FAILURE;
    }
    uint8_t *lit  = malloc(Nact);
    uint64_t Npup = 0;
    if (lit == NULL)
    {
        abort();
    }
    for (uint64_t k = 0; k < Nact; k++)
    {
        lit[k] = (mask.ID == -1) || (mask.im->array.F[k] > 0.0f);
        Npup += lit[k];
    }
    imgid_free(&mask);
    if (Npup == 0)
    {
        PRINT_ERROR("%s: no lit pixel in the pupil mask", pupmask);
        free(lit);
        imgid_free(&dmimg);
        return RETURN_FAILURE;
    }

    IMGID outim = stream_connect_create_2D(wfssignal_out, 2 * Nx, 2 * Ny, _DATATYPE_FLOAT);

    MILK_PROCINFO_INIT(self)

    // One frame before the first wait: the output shows the current DM
    pywfs_frame(dmimg.im->array.F, lit, outim.im->array.F, Nx, Ny, Npup);
    processinfo_update_output_stream(processinfo, outim.im, NULL);

    MILK_PROCINFO_LOOPSTART(self)
    {
        pywfs_frame(dmimg.im->array.F, lit, outim.im->array.F, Nx, Ny, Npup);
        processinfo_update_output_stream(processinfo, outim.im, dmimg.im);
    }
    MILK_PROCINFO_END(self)

    free(lit);
    imgid_free(&dmimg);
    imgid_free(&outim);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_IOtools_PyWFSsim = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "pywfssim",
                 .cmdkey           = "pywfssim",
                 .description      = "cheap pyramid WFS simulation",
                 .description_long = "Map the x and y slopes of a DM onto four pupil images, "
                                     "2Nx x 2Ny float32, total 1, with a smooth saturation." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(PyWFSsim),
    .compute   = PyWFSsim_compute,
    .confcheck = customCONFcheck,
};
EXPORT_FPSEXEC(cacao, "cacaoiot-pywfssim", runnable_AOloopControl_IOtools_PyWFSsim);
