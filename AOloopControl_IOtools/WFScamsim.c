/**
 * @file    WFScamsim.c
 * @brief   camera simulation for WFS
 *
 */

#include "CommandLineInterface/CLIcore.h"


#include "statistic/statistic.h"

// Local variables pointers

// input signal stream
static char *wfssignal_in;
static long  fpi_wfssignal_in;


// output WFS image
static char *wfsim_out;
static long  fpi_wfsim_out;



// compute flag: dark subtract
static int64_t *compdarkadd;
static long     fpi_compdarkadd;

// dark frame
static char *wfsdark;
static long  fpi_wfsdark;



static float *fluxtotal;
static long   fpi_fluxtotal;

// Camera gain e-/ADU
static float *camgain;
static long   fpi_camgain;


// compute flag: apply photon noise
static int64_t *compphnoise;
static long     fpi_compphnoise;

// Readout noise
// negative value if no RON
static float *camRON;
static long   fpi_camRON;








static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STREAM,
        ".wfssignal",
        "Wavefront sensor input signal",
        "aol9_wfssignal",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfssignal_in,
        &fpi_wfssignal_in
    },
    {
        CLIARG_STREAM,
        ".wfscamim",
        "Wavefront sensor ouput image",
        "aol9_wfsim",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsim_out,
        &fpi_wfsim_out
    },
    {
        CLIARG_ONOFF,
        ".compdarkadd",
        "subtract dark",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compdarkadd,
        &fpi_compdarkadd
    },
    {
        CLIARG_STREAM,
        ".camdark",
        "camera dark frame",
        "aol9_wfsdark",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &wfsdark,
        &fpi_wfsdark
    },
    {
        CLIARG_FLOAT32,
        ".fluxtotal",
        "total output flux [phe-], <0 if no scaling",
        "1000.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &fluxtotal,
        &fpi_fluxtotal
    },
    {
        CLIARG_FLOAT32,
        ".camgain",
        "camera gain [e- / ADU]",
        "2.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &camgain,
        &fpi_camgain
    },
    {
        CLIARG_ONOFF,
        ".compphnoise",
        "compute photon noise",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &compphnoise,
        &fpi_compphnoise
    },
    {
        CLIARG_FLOAT32,
        ".camRON",
        "camera readout noise [e-] (neg = 0)",
        "-1.0",
        CLIARG_OUTPUT_DEFAULT,
        (void **) &camRON,
        &fpi_camRON
    }
};






// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_compdarkadd].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_fluxtotal].fpflag   |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_camgain].fpflag     |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_compphnoise].fpflag |= FPFLAG_WRITERUN;
        data.fpsptr->parray[fpi_camRON].fpflag      |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}

// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    return RETURN_SUCCESS;
}

static CLICMDDATA CLIcmddata =
{
    "WFScamsim", "simulate WFS camera", CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}








static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();


    IMGID wfssignalimg = mkIMGID_from_name(wfssignal_in);
    resolveIMGID(&wfssignalimg, ERRMODE_ABORT);

    uint32_t sizexWFS = wfssignalimg.size[0];
    uint32_t sizeyWFS = wfssignalimg.size[1];

    uint64_t sizeWFS = (uint64_t) sizexWFS;
    sizeWFS *= sizeyWFS;

    IMGID wfsdarkimg = mkIMGID_from_name(wfsdark);

    resolveIMGID(&wfsdarkimg, ERRMODE_WARN);

    IMGID imcamtmpimg = makeIMGID_2D("imcamtmp", sizexWFS, sizeyWFS);
    createimagefromIMGID(&imcamtmpimg);

    // Create output
    //
    IMGID wfsoutimg;
    {
        printf("CONNECTING / CREATING output stream\n");
        wfsoutimg =
            stream_connect_create_2D(wfsim_out, sizexWFS, sizeyWFS, _DATATYPE_UINT16);
    }



    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {

        // scale flux
        // ensure there is no negative value
        //
        if(*fluxtotal < 0.0)
        {
            // do not scale
            memcpy(imcamtmpimg.im->array.F, wfssignalimg.im->array.F, sizeof(float)* sizexWFS);
        }
        else
        {
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                if(wfssignalimg.im->array.F[ii] > 0.0)
                {
                    imcamtmpimg.im->array.F[ii] = (*fluxtotal) * wfssignalimg.im->array.F[ii];
                }
                else
                {
                    imcamtmpimg.im->array.F[ii] = 0.0;
                }
            }
        }

        // add photon noise
        //
        if(*fluxtotal >= 0.0)
        {
            if(data.fpsptr->parray[fpi_compphnoise].fpflag & FPFLAG_ONOFF)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] = poisson(imcamtmpimg.im->array.F[ii]);
                }
            }
        }

        // add readout noise
        //
        if(*fluxtotal >= 0.0)
        {
            if(*camRON > 0.0)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] += (*camRON) * gauss();
                }
            }
        }

        // convert to ADU
        //
        if(*fluxtotal >= 0.0)
        {
            for(uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                imcamtmpimg.im->array.F[ii] /= (*camgain);
            }
        }


        // add dark
        //
        if(*fluxtotal >= 0.0)
        {
            if(wfsdarkimg.ID != -1)
            {
                for(uint64_t ii = 0; ii < sizeWFS; ii++)
                {
                    imcamtmpimg.im->array.F[ii] += wfsdarkimg.im->array.F[ii];
                }
            }
        }

        // write to output camera
        //
        for(uint64_t ii = 0; ii < sizeWFS; ii++)
        {

            float tmpval = imcamtmpimg.im->array.F[ii];
            if(tmpval < 0.0)
            {
                wfsoutimg.im->array.UI16[ii] = 0;
            }
            else if (tmpval > 65535)
            {
                wfsoutimg.im->array.UI16[ii] = 65535;
            }
            else
            {
                wfsoutimg.im->array.UI16[ii] = (uint16_t) tmpval;
            }
        }

        DEBUG_TRACEPOINT(" ");


        processinfo_update_output_stream(processinfo, wfsoutimg.ID);
    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

INSERT_STD_FPSCLIfunctions

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_IOtools__WFScamsim()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
