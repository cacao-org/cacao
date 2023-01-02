/**
 * @file    measure_linear_respm.c
 * @brief   Measure linear response to perturbation
 *
 */

#include <stdlib.h>
#include <stdio.h>

#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"


// holds info for each poke
typedef struct
{

    // Cycle number
    uint64_t iter;

    // Did we poke DM during this time interval ?
    int poke;

    // Does frame count toward accumulated signal ?
    int accum;

    // frame index within poke mode acquisition
    // -1 if not part of signal
    int aveindex;

    // frame counter within poke mode acquisition, starts negative
    // becomes positive when accumulating signal
    int pokeframeindex;

    // Poke mode being measured
    int PokeIndexMEAS;

    // Current poke mode on input
    int PokeIndexCTRL;

    // Poke mode being measured, index in poke cube
    int PokeIndexMEAS_Mapped;

    // Current poke mode on DM, index in poke cube
    int PokeIndexCTRL_Mapped;


    struct timespec ts;

} PokeInfo;

static PokeInfo *pkinfarray;






// Local variables pointers


// input stream
static char *streamin;
static long  fpi_streamin;


// output stream
static char *streamout;
static long  fpi_streamout;


// input mode cube
static char *inmodeC;
static long  fpi_inmodeC;

// output mode cube
static char *outmodeC;
static long  fpi_outmodeC;


static float *pokeampl;
long          fpi_pokeampl;



// TIMING

// Toggles
static uint64_t *autotiming;
static long fpi_autotiming;

// WFS frame rate [Hz]
static float *WFSfrequ;
long fpi_WFSfrequ;

// Hardware latency in unit of WFS frame
static float *hardwlatfr;
long fpi_hardwlatfr;

static uint32_t *delayfr;
static long fpi_delayfr;
static uint32_t *delayRM1us;
static long fpi_delayRM1us;

static uint32_t *NBave;
static uint32_t *NBexcl;
static uint32_t *NBinnerCycle;





static CLICMDARGDEF farg[] =
{
    {
        CLIARG_STREAM,
        ".streamin",
        "input (perturbation) stream",
        "NULL",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &streamin,
        &fpi_streamin
    },
    {
        CLIARG_STREAM,
        ".streamout",
        "output (signal) stream",
        "NULL",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &streamout,
        &fpi_streamout
    },
    {
        CLIARG_FITSFILENAME,
        ".inmodes",
        "input modes",
        "NULL",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inmodeC,
        &fpi_inmodeC
    },
    {
        CLIARG_FITSFILENAME,
        ".outmodes",
        "output modes",
        "NULL",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &outmodeC,
        &fpi_outmodeC
    },
    {
        CLIARG_FLOAT32,
        ".ampl",
        "RM poke amplitude",
        "0.01",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &pokeampl,
        &fpi_pokeampl
    },
    // ============= TIMING =========================
    {
        CLIARG_FLOAT32,
        ".timing.WFSfrequ",
        "WFS frame rate [Hz]",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &WFSfrequ,
        &fpi_WFSfrequ
    },
    {
        CLIARG_FLOAT32,
        ".timing.hardwlatfr",
        "hardware latency [fr]",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &hardwlatfr,
        &fpi_hardwlatfr
    },
    {
        CLIARG_ONOFF,
        ".timing.autoTiming",
        "Auto Timing",
        "ON",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &autotiming,
        &fpi_autotiming
    },
    {
        CLIARG_UINT32,
        ".timing.delayfr",
        "frame delay, whole part",
        "2",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayfr,
        &fpi_delayfr
    },
    {
        CLIARG_UINT32,
        ".timing.delayRM1us",
        "Sub-frame delay [us]",
        "100",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &delayRM1us,
        &fpi_delayRM1us
    },
    {
        CLIARG_UINT32,
        ".timing.NBave",
        "Number of frames averaged for a single poke measurement",
        "5",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBave,
        NULL
    },
    {
        CLIARG_UINT32,
        ".timing.NBexcl",
        "Number of frames excluded",
        "1",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBexcl,
        NULL
    },
    {
        CLIARG_UINT32,
        ".timing.NBinnerCycle",
        "Number of inner cycles (how many consecutive times should a single +/- "
        "poke be repeated)",
        "10",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &NBinnerCycle,
        NULL
    }
};


// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{

    if(data.fpsptr != NULL)
    {
        data.fpsptr->parray[fpi_streamin].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;
        data.fpsptr->parray[fpi_streamout].fpflag |=
            FPFLAG_STREAM_RUN_REQUIRED | FPFLAG_CHECKSTREAM;

        data.fpsptr->parray[fpi_inmodeC].fpflag |=
            FPFLAG_FILE_RUN_REQUIRED;

        data.fpsptr->parray[fpi_pokeampl].fpflag |= FPFLAG_WRITERUN;
    }

    return RETURN_SUCCESS;
}


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{

    if(data.fpsptr != NULL)
    {
        if(data.fpsptr->parray[fpi_autotiming].fpflag & FPFLAG_ONOFF)  // ON state
        {
            printf("UPDATE TIMING >>>>>>>>>\n");


            {
                char *envvar = "CACAO_HARDWLAT_FRAME";
                if(!getenv(envvar))
                {
                    fprintf(stderr, "The environment variable %s was not found.\n", envvar);
                }
                else
                {
                    float tmpv = atof(getenv(envvar));
                    data.fpsptr->parray[fpi_hardwlatfr].val.i64[0]    = tmpv;
                }
            }



            {
                char *envar = "CACAO_WFS_FRATE";
                if(!getenv(envar))
                {
                    fprintf(stderr, "The environment variable %s was not found.\n", envar);
                }
                else
                {
                    float tmpv = atof(getenv(envar));
                    data.fpsptr->parray[fpi_WFSfrequ].val.i64[0]    = tmpv;
                }
            }



            // RMdelay = hardwlaten - 0.5 - excl/2
            double RMdelay = (*hardwlatfr) - 0.5 - 0.5 * (*NBexcl);

            int RMdelayfr =
                ((int)((*hardwlatfr) - 0.5 - 0.5 * (*NBexcl) +  10.0)) + 1 - 10;

            int delayRM1us = (int)((1.0 * RMdelayfr - RMdelay) / (*WFSfrequ) * 1000000.0);

            if(RMdelay < 0)
            {
                RMdelayfr    = 0;
                delayRM1us   = 0;
            }

            printf("RMdelayfr  = %d\n", RMdelayfr);
            printf("delayRM1us = %d\n", delayRM1us);


            data.fpsptr->parray[fpi_delayfr].val.i64[0]    = RMdelayfr;
            data.fpsptr->parray[fpi_delayRM1us].val.i64[0] = delayRM1us;

            // set back to OFF
            data.fpsptr->parray[fpi_autotiming].fpflag &= ~FPFLAG_ONOFF;
        }
    }

    return RETURN_SUCCESS;
}




static CLICMDDATA CLIcmddata =
{
    "measlinresp", "measure linear response of one stream to another", CLICMD_FIELDS_DEFAULTS
};



// detailed help
static errno_t help_function()
{


    return RETURN_SUCCESS;
}





static errno_t PokeUpdate(
    PokeInfo pkinf,
    PokeInfo *pkinfarray,
    int pokecnt,
    int NBpokeTotal
)
{

    if(pokecnt > NBpokeTotal - 1)
    {
        printf("ERROR: pokecnt %d   / %d\n", pokecnt, NBpokeTotal);
        exit(0);
    }


    pkinfarray[pokecnt].iter = pkinf.iter;
    pkinfarray[pokecnt].poke = pkinf.poke;
    pkinfarray[pokecnt].accum = pkinf.accum;
    pkinfarray[pokecnt].aveindex = pkinf.aveindex;
    pkinfarray[pokecnt].pokeframeindex = pkinf.pokeframeindex;
    pkinfarray[pokecnt].PokeIndexMEAS = pkinf.PokeIndexMEAS;
    pkinfarray[pokecnt].PokeIndexCTRL = pkinf.PokeIndexCTRL;
    pkinfarray[pokecnt].PokeIndexMEAS_Mapped = pkinf.PokeIndexMEAS_Mapped;
    pkinfarray[pokecnt].PokeIndexCTRL_Mapped = pkinf.PokeIndexCTRL_Mapped;

    struct timespec poke_ts;
    clock_gettime(CLOCK_REALTIME, &poke_ts);
    pkinfarray[pokecnt].ts.tv_sec = poke_ts.tv_sec;
    pkinfarray[pokecnt].ts.tv_nsec = poke_ts.tv_nsec;

    return RETURN_SUCCESS;
}






/**
 * ## Purpose
 *
 * Acquire WFS response to a series of DM patterns.
 * Called by AOloopControl_acquireCalib_Measure_WFS_linResponse()
 *
 * ## Description
 *
 *
 * ### Overview
 *
 * The function's main input is a set of actuator poke maps (imginmodeC),
 * and its main output is the corresponding output sensor response
 * (outCname).
 *
 *
 * ### Timing
 *
 * Timing offset between actuator and sensor is specified as a delay in unit
 * of sensor frames. The delay is the sum of an integer number of sensor
 * frames (timing_delayfr) and an additional time delay in microsecond (timing_delayRM1us).
 *
 * timing_NBave consecutive sensor frame(s) are averaged for each poke. timing_NBexcl frame(s)
 * are ignored between pokes to allow for actuator and sensor finite time
 * responses and timing jitter.
 *
 *
 * ### Number of cycles
 *
 * The measurement is repeated timing_NBcycle times for averaging. If SequInitMode
 * bitmask is zero, the same sequence of pokes is ran timing_NBcycle times.
 * Alternatively, if bit 0 is set to 1, then pairs are swapped every 4 pokes. If bit
 * 1 is set to one, then adjacent pairs are swapped between cycles.
 *
 * The swapping patterns (described in detail below with examples) are essential to
 * average out and cancel temporal bleeding between modes. If the actuator's temporal
 * response extends to multiple frames, it is important to avoid repeating the same sequence
 * of pokes, and to shuffle the poke order.
 *
 * ### Example poke sequences
 *
 * Numbers shows are index in the input poke cube
 *
 * not bit set
 *            00 01 02 03 04 05 06 07 08 09 10 11 12 ...
 * cycle 0 :  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 ...
 * cycle 1 :  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 ...
 * -- each cycle is the same sequence of pokes ---
 *
 * Following sequential pokes for index 07 :
 *
 * ... 05 06 07 08 09 ...
 * ... 05 06 07 08 09 ...
 *
 *
 * bit 0 set to 1 (0x01)
 *            00 01 02 03 04 05 06 07 08 09 10 11 12 ...
 * swaps       <->         <->         <->         <->
 * cycle 0 :  01 00 02 03 05 04 06 07 09 08 10 11 13 ...
 * cycle 1 :  01 00 02 03 05 04 06 07 09 08 10 11 13 ...
 * cycle 2 :  01 00 02 03 05 04 06 07 09 08 10 11 13 ...
 *
 * Following sequential pokes for index 07 :
 *
 * cycle 0 : ... 04 06 07 09 08 ...
 * cycle 1 : ... 04 06 07 09 08 ...
 * cycle 2 : ... 04 06 07 09 08 ...
 *
 *
 *
 *
 * bit 1 set (0x02)
 *            00 01 02 03 04 05 06 07 08 09 10 11 12 ...
 * swaps          <->   <->   <->   <->   <->   <->
 * cycle 0 :  00 02 01 04 03 06 05 08 07 10 09 12 11 ...
 * swaps       <->   <->   <->   <->   <->   <->
 * cycle 1 :  02 00 04 01 06 03 08 05 10 07 12 09 14 ...
 * swaps          <->   <->   <->   <->   <->   <->
 * cycle 2 :  02 04 00 06 01 08 03 10 05 12 07 14 09
 *
 * Even poke indices more to the left, while odd ones move to the right.
 * (except for poke 00, which moves to the right).
 *
 *
 * Following sequential pokes for index 07 :
 *
 * cycle 0 : ... 05 08 07 10 09 ...
 * cycle 1 : ... 05 10 07 12 09 ...
 * cycle 2 : ... 05 12 07 14 09 ...
 *
 * before 07 : 08 10 12 14 16 18 ...
 * k odd : (k+1)+2n
 * k even: (k-3)-2n
 *
 * after  07 : 10 12 14 16 18 20 ...
 * k odd : (k+3)+2n
 * k even: (k-2)-2n
 *
 *
 * bits 0 and 1 set (0x03)
 *
 *                 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22
 * 0x01 swaps       <->         <->         <->         <->         <->         <->
 *                 01 00 02 03 05 04 06 07 09 08 10 11 13 12 14 15 17 16 18 19 21 20 22
 * 0x02 swaps          <->   <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 0         01 02 00 05 03 06 04 09 07 10 08 13 11 14 12 17 15 18 16 21 19 22 20
 *
 * 0x02 swaps       <->   <->  <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 1         02 01 05 00 06 03 09 04 10 07 13 08 12 11 17 12 18 15 21 16 22 19
 *
 * 0x02 swaps          <->   <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 2         02 05 01 06 00 09 03 10 04 13 07 12 08 17 11 18 12 21 15 22 16
 *
 * 0x02 swaps       <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 3         05 02 06 01 09 00 10 03 13 04 12 07 17 08 18 11 21 12 22 15
 *
 * 0x02 swaps          <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 4         05 06 02 09 01 10 00 13 03 12 04 17 07 18 08 21 11 22 12
 *
 * 0x02 swaps       <->   <->   <->   <->   <->   <->   <->   <->   <->   <->
 * cycle 5         06 05 09 02 10 01 13 00 12 03 17 04 18 07 21 08 22 11
 *
 * Following sequential pokes for index 07 :
 *
 * cycle 0 : ... 04 09 07 10 08 ...
 * cycle 1 : ... 04 10 07 13 08 ...
 * cycle 2 : ... 04 13 07 12 08 ...
 * cycle 3 : ... 04 12 07 17 08 ...
 * cycle 4 : ... 04 17 07 18 08 ...
 * cycle 5 : ... 04 18 07 21 08 ...
 *
 * before 07 : (09 10 13 12) (17 18
 * (k+2 k+3 k+6 k+5) +8 each 4
 *
 * after  07 : (10 13 12 17) (18 21
 * (k+3 k+6 k+5 k+10) +8 each 4
 *
 *
 *
 * ### Optional output
 *
 * ## Arguments
 *
 *
 * @param imgin                Input stream to be poked
 * @param imgout               Output stream to be measured
 * @param imginmodeC           Poke pattern, modes to be sensed
 * @param timing_delayfr       Integer delay [frame]
 * @param timing_delayRM1us    Fractional delay [us]
 * @param timing_NBave         Number of frames averaged per poke state
 * @param timing_NBexcl        Number of frames excluded
 * @param timing_NBcycle       Number of cycles averaged (outer)
 * @param SequInitMode         Sequence initialization mode bitmask
 *                             0x01  swap pairs every 4 indices once
 *                             0x02 adjacent pairs are swapped between cycles
 * @param outCname             Output respone cube
 * @param outdir               Output directory where files are written
 * @return errno_t
 *
 *
 * *
 * SIGINT signal will stop acquisition immediately
 * USR2 signal completes current cycles and stops acquisition
 *
 * @note TODO: Issue DM command to be sent at a specified time in the future.
 */


static errno_t Measure_Linear_Response_Modal(
    IMGID       imgin,
    IMGID       imgout,
    IMGID       imginmodeC,
    float       ampl,
    long        timing_delayfr,
    long        timing_delayRM1us,
    uint32_t    timing_NBave,
    uint32_t    timing_NBexcl,
    uint32_t    SequInitMode,
    char       *outCname,
    char       *outdir
)
{
    // Save all intermediate results
    int SAVE_RMACQU_ALL = 1;


    // Convenient notations

    // Input space dimensions
    //
    uint32_t sizexin = imgin.md->size[0];
    uint32_t sizeyin = imgin.md->size[1];
    uint64_t sizexyin = sizexin;
    sizexyin *= sizeyin;

    // Output space dimensions
    //
    uint32_t sizexout = imgout.md->size[0];
    uint32_t sizeyout = imgout.md->size[1];
    uint64_t sizexyout = sizexout;
    sizexyout *= sizeyout;

    long NBmode       = imginmodeC.md->size[2];


    // Current poke info, counters etc
    PokeInfo pkinf;


    // duplicaate each mode to positive an negative amplitude
    //
    long NBmode2 = NBmode * 2;
    IMGID imginmodeC2 = makeIMGID_3D("pokemodeC2", sizexin, sizeyin, NBmode2);
    imageID IDinmodeC2 = createimagefromIMGID(&imginmodeC2);

    for(int mode = 0; mode < NBmode; mode++)
    {
        int mode2 = 2 * mode;
        for(uint64_t ii = 0; ii < sizexyin; ii++)
        {
            imginmodeC2.im->array.F[mode2 * sizexyin + ii] = ampl *
                    imginmodeC.im->array.F[mode * sizexyin + ii];
        }
        mode2 = 2 * mode + 1;
        for(uint64_t ii = 0; ii < sizexyin; ii++)
        {
            imginmodeC2.im->array.F[mode2 * sizexyin + ii] = -ampl *
                    imginmodeC.im->array.F[mode * sizexyin + ii];
        }
    }






    printf("    input  space size : %u %u\n", sizexin, sizeyin);
    printf("    output space size : %u %u\n", sizexout, sizeyout);
    printf("    input modes size  : %u %u %u\n", imginmodeC.md->size[0],
           imginmodeC.md->size[1], imginmodeC.md->size[2]);

    printf("    timing_delayfr    : %ld\n", timing_delayfr);
    printf("    timimg_delayRM1us : %ld us\n", timing_delayRM1us);
    printf("    timing_NBave      : %u\n", timing_NBave);
    printf("    timing_NBexcl     : %u\n", timing_NBexcl);
    printf("    SequInitMode      : %u\n", SequInitMode);






    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    long timing_NBcycle = processinfo->loopcntMax;
    printf("    timing_NBcycle    : %ld\n", timing_NBcycle);


    /*
    * If timing_NBcycle is set to zero, then the process should run in an infinite loop.
    * The process will then run until receiving SIGINT.
    */
    uint64_t NBiter = 10000; // runs until SIGINT signal received
    if(timing_NBcycle < 1)
    {
        NBiter = LONG_MAX; // runs until SIGINT signal received
    }
    else
    {
        NBiter = timing_NBcycle;
    }




    // Timing info for pokes
    //
    // The total number of pokes (1 poke = 1 frame)
    // consecutive identical pokes are counted as separate
    //
    uint64_t NBpokeTotal;
    NBpokeTotal  = timing_NBave + timing_NBexcl;  // for each DM mode
    NBpokeTotal *= NBmode2;                       // multiplied by number of modes
    NBpokeTotal += timing_delayfr;                // to allow for latency at startup
    NBpokeTotal += 4; // ???

//    (4 + timing_delayfr + (timing_NBave + timing_NBexcl) * NBmode2) * NBiter + 4;


    uint64_t pokecnt = 0;




    // create one temporary array per time step
    imageID *IDoutCstep = (imageID *) malloc(sizeof(imageID) * timing_NBave);


    // Cumulative
    for(uint32_t AveStep = 0; AveStep < timing_NBave; AveStep++)
    {
        char imname[STRINGMAXLEN_IMGNAME];
        WRITE_IMAGENAME(imname, "imoutStep%03u", AveStep);
        create_3Dimage_ID(imname,
                          imgout.md->size[0],
                          imgout.md->size[1],
                          NBmode2,
                          &(IDoutCstep[AveStep]));
    }



    // initialize arrays to zero
    //
    // Output array is created and initialized to hold the WFS response to each poke mode.
    //
    IMGID imgoutC = makeIMGID_3D(outCname, sizexout, sizeyout, NBmode2);
    imageID IDoutC = createimagefromIMGID(&imgoutC);

    for(uint32_t PokeIndex = 0; PokeIndex < NBmode2; PokeIndex++)
    {
        // Mode to be poked

        for(uint64_t ii = 0; ii < imgout.md->size[0]*imgout.md->size[1]; ii++)
        {
            imgoutC.im->array.F[PokeIndex * sizexyout + ii] = 0.0;
            for(uint32_t AveStep = 0; AveStep < timing_NBave; AveStep++)
            {
                data.image[IDoutCstep[AveStep]].array.F[PokeIndex * sizexyout + ii] = 0.0;
            }
        }
    }


    /*
    LATENCY CONVENTION

    hardwlat = 0.5 frame : DM poke synced with readout

    */

    pkinfarray = (PokeInfo *) malloc(sizeof(PokeInfo) * NBpokeTotal);



    // Poke sequence defines the sequence of mode poked for each iteration
    uint32_t *array_PokeSequ = (uint32_t *) malloc(sizeof(uint32_t) * NBmode2);
    // It is first initiated to poke consecutive modes
    for(uint32_t PokeIndex = 0; PokeIndex < NBmode2; PokeIndex++)
    {
        array_PokeSequ[PokeIndex] = PokeIndex;
    }

    if(SequInitMode & 0x01)
    {
        // swap pairs every 4 indices
        for(uint32_t PokeIndex = 0; PokeIndex < NBmode2 - 1; PokeIndex += 4)
        {
            uint32_t index0 = PokeIndex;
            uint32_t index1 = PokeIndex + 1;

            while(index1 > (uint32_t)(NBmode2 - 1))
            {
                index1 -= NBmode2;
            }

            // swap sequence pairs
            uint32_t tmpPokeMode;
            tmpPokeMode            = array_PokeSequ[index0];
            array_PokeSequ[index0] = array_PokeSequ[index1];
            array_PokeSequ[index1] = tmpPokeMode;
        }
    }

    // for swap 0x02, permut offset keep track of parity
    // permut_offset toggles from 0 to 1, starting here at 0
    int permut_offset = 0;




    char *ptr0      = (char *) imginmodeC2.im->array.F;
    size_t framesize   = sizeof(float) * sizexin * sizeyin;

    int semindexout = ImageStreamIO_getsemwaitindex(imgout.im, 0);
    printf("Using semaphore %d\n", semindexout);


    pkinf.iter = 0;
    pkinf.pokeframeindex = 0;
    pkinf.PokeIndexMEAS  = 0;     // Poked mode index being measured
    pkinf.PokeIndexCTRL = 0;  // Poke mode being actuated



    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        pokecnt = 0;

        printf("ITERATION %lu\n", pkinf.iter);
        fflush(stdout);

        printf("    %5d  pokecnt %8lu / %8lu\n", __LINE__, pokecnt, NBpokeTotal);
        fflush(stdout);

        processinfo_WriteMessage_fmt(processinfo, "it %lu/%lu",
                                     pkinf.iter,
                                     processinfo->loopcntMax
                                    );




        // swap pokes pairs
        //
        if(SequInitMode & 0x02)
        {
            printf("SWAPPING MODE 2\n");
            fflush(stdout);
            permut_offset++;
            if(permut_offset == 2)
            {
                permut_offset = 0;
            }

            for(uint32_t PokeIndex = permut_offset; PokeIndex < NBmode2; PokeIndex += 2)
            {
                uint32_t index0 = PokeIndex;
                uint32_t index1 = PokeIndex + 1;

                if(index1 > (uint32_t)(NBmode2 - 1))
                {
                    index1 -= NBmode2;
                }

                // swap sequence pairs
                uint32_t tmpPokeMode;
                tmpPokeMode            = array_PokeSequ[index0];
                array_PokeSequ[index0] = array_PokeSequ[index1];
                array_PokeSequ[index1] = tmpPokeMode;
            }
        }




        processinfo_WriteMessage_fmt(processinfo, "%lu init first poke", pkinf.iter);

        pkinf.pokeframeindex = 0;
        pkinf.PokeIndexMEAS  = 0; // Poked mode index
        pkinf.PokeIndexCTRL = 0;

        pkinf.PokeIndexMEAS_Mapped  = array_PokeSequ[pkinf.PokeIndexMEAS];
        pkinf.PokeIndexCTRL_Mapped = array_PokeSequ[pkinf.PokeIndexCTRL];

        if((pkinf.PokeIndexCTRL_Mapped > (uint32_t)(NBmode2 - 1)))
        {
            printf("ERROR: PokeIndexCTRL_Mapped = %u is outside range 0 - %ld\n",
                   pkinf.PokeIndexCTRL_Mapped,
                   NBmode2);
            exit(0);
        }

        usleep(timing_delayRM1us);





        // POKE first pattern
        // (pokecnt increments by 1)
        //
        pkinf.aveindex = -1; // not part of signal

        imgin.md->write = 1;
        memcpy((void *) imgin.im->array.F,
               (void *)(ptr0 + pkinf.PokeIndexCTRL_Mapped * framesize),
               sizeof(float) * sizexyin);
        imgin.md->cnt1 = pkinf.PokeIndexCTRL_Mapped;
        processinfo_update_output_stream(processinfo, imgin.ID);

        PokeUpdate(pkinf, pkinfarray, pokecnt, NBpokeTotal);

        printf("    %5d  pokecnt %8lu / %8lu\n", __LINE__, pokecnt, NBpokeTotal);
        fflush(stdout);

        pokecnt++;



        ImageStreamIO_semwait(imgout.im, semindexout);


        // Is this needed ??
        //COREMOD_MEMORY_image_set_sempost_byID(imgin.ID, -1);
        //imgin.md->cnt0++;





        // WAIT FOR LOOP DELAY, PRIMING
        // POKE first pattern
        // (pokecnt increments by 1+timing_delayfr)


        // Read delayfr frames (priming)
        //
        for(int delayindex = 0; delayindex < timing_delayfr; delayindex++)
        {
            // pokecnt++; ???

            // Read_cam_frame(loop, 1, normalize, 0, 0);
            ImageStreamIO_semwait(imgout.im, semindexout);

            pkinf.pokeframeindex++;
            if(pkinf.pokeframeindex == (int) timing_NBave)
            {
                pkinf.pokeframeindex = -timing_NBexcl;
                pkinf.PokeIndexCTRL++;

                if(pkinf.PokeIndexCTRL > NBmode2 - 1)
                {
                    pkinf.PokeIndexCTRL -= NBmode2;
                }

                pkinf.PokeIndexCTRL_Mapped = array_PokeSequ[pkinf.PokeIndexCTRL];

                if((pkinf.PokeIndexCTRL_Mapped > (uint32_t)(NBmode2 - 1)))
                {
                    printf(
                        "ERROR: PokeIndexCTRLMapped = %u is outside range 0 - "
                        "%ld\n",
                        pkinf.PokeIndexCTRL_Mapped,
                        NBmode2);
                    exit(0);
                }


                usleep(timing_delayRM1us);

                // POKE
                imgin.md->write = 1;
                memcpy((void *) imgin.im->array.F,
                       (void *)(ptr0 + pkinf.PokeIndexCTRL_Mapped * framesize),
                       sizeof(float) * sizexyin);
                imgin.md->cnt1 = pkinf.PokeIndexCTRL_Mapped;
                processinfo_update_output_stream(processinfo, imgin.ID);

                pkinf.poke = 1;

                PokeUpdate(pkinf, pkinfarray, pokecnt, NBpokeTotal);
                pokecnt ++;

            }
        }


        printf("    %5d  pokecnt %8lu / %8lu\n", __LINE__, pokecnt, NBpokeTotal);
        printf("    PokeIndex = %u\n", pkinf.PokeIndexMEAS);
        fflush(stdout);






        // First inner loop increment poke mode
        //
        while((pkinf.PokeIndexMEAS < NBmode2) && (data.signal_INT == 0))
        {
            // INTEGRATION

            processinfo_WriteMessage_fmt(processinfo, "iter %lu/%lu poke %u/%ld",
                                         pkinf.iter,
                                         processinfo->loopcntMax,
                                         pkinf.PokeIndexMEAS,
                                         NBmode2);

            for(int aveindex = 0; aveindex < timing_NBave + timing_NBexcl; aveindex++)
            {
                PokeUpdate(pkinf, pkinfarray, pokecnt, NBpokeTotal);
                pokecnt++;



                // Capture signal
                //
                ImageStreamIO_semwait(imgout.im, semindexout);
                //
                //
                //
                if(aveindex < timing_NBave)
                {
                    {
                        char *ptr = (char *) data.image[IDoutCstep[aveindex]].array.F;
                        ptr += sizeof(float) * pkinf.PokeIndexMEAS_Mapped * sizexyout;
                        memcpy(ptr,
                               imgout.im->array.F,
                               sizeof(float) * sizexyout);
                    }
                    pkinf.accum = 1;
                }





                pkinf.pokeframeindex++;
                // wait for NBexcl excluded frames. We poke after delayRM1us
                if(pkinf.pokeframeindex == (int) timing_NBave)
                {
                    pkinf.pokeframeindex = -timing_NBexcl;
                    pkinf.PokeIndexCTRL++;

                    if(pkinf.PokeIndexCTRL > NBmode2 - 1)
                    {
                        pkinf.PokeIndexCTRL -= NBmode2;
                    }

                    pkinf.PokeIndexCTRL_Mapped = array_PokeSequ[pkinf.PokeIndexCTRL];

                    usleep(timing_delayRM1us);

                    //printf(">>>>>>>>>>>>>>>>>>> %d %d %d <<<<<<<<<<<<<<<<<\n", __LINE__, PokeIndex1, PokeIndex1Mapped);
                    //fflush(stdout);

                    // POKE
                    imgin.md->write = 1;
                    memcpy((void *)(imgin.im->array.F),
                           (void *)(ptr0 + pkinf.PokeIndexCTRL_Mapped * framesize),
                           sizeof(float) * sizexyin);
                    imgin.md->cnt1 = pkinf.PokeIndexCTRL_Mapped;
                    processinfo_update_output_stream(processinfo, imgin.ID);



                    pkinf.poke = 1;
                }

            }

            pkinf.PokeIndexMEAS++;
            pkinf.PokeIndexMEAS_Mapped = array_PokeSequ[pkinf.PokeIndexMEAS];
        }


        printf("    %5d  pokecnt %8lu / %8lu\n", __LINE__, pokecnt, NBpokeTotal);
        fflush(stdout);





        usleep(timing_delayRM1us);

        // zero DM channel
        {
            /* A temporary array is created to hold the input commands */
            float *arrayf = (float *) calloc(sizexyin, sizeof(float));
            for(uint64_t ii = 0; ii < sizexyin; ii++)
            {
                arrayf[ii] = 0.0;
            }
            imgin.md->write = 1;
            memcpy((void *)(imgin.im->array.F),
                   (void *)(arrayf),
                   sizeof(float) * sizexyin);
            imgin.md->cnt1 = 0;
            processinfo_update_output_stream(processinfo, imgin.ID);
            free(arrayf);
        }



        pkinf.poke = 1;

        printf("Combining results ... ");
        fflush(stdout);

        for(uint32_t AveStep = 0; AveStep < timing_NBave; AveStep++)
        {
            // sum over all values AveStep
            for(pkinf.PokeIndexMEAS_Mapped = 0; pkinf.PokeIndexMEAS_Mapped < NBmode2;
                    pkinf.PokeIndexMEAS_Mapped++)
            {
                for(uint64_t ii = 0; ii < sizexyout; ii++)
                {
                    imgoutC.im->array.F[pkinf.PokeIndexMEAS_Mapped * sizexyout + ii] +=
                    data.image[IDoutCstep[AveStep]].array.F[pkinf.PokeIndexMEAS_Mapped * sizexyout +
                                                            ii];
                }
            }
        }

        printf("DONE\n");
        fflush(stdout);

        DEBUG_TRACEPOINT(" ");



        if(SAVE_RMACQU_ALL == 1)
        {

            // Save all intermediate result

            EXECUTE_SYSTEM_COMMAND("mkdir -p %s", outdir);

            FILE *fplog;

            char tmpfname[STRINGMAXLEN_FULLFILENAME];
            WRITE_FULLFILENAME(tmpfname, "%s/RMacqulog.txt", outdir);
            fplog = fopen(tmpfname, "w");
            fprintf(fplog, "%-20s  %ld\n", "delayfr", timing_delayfr);
            fprintf(fplog, "%-20s  %ld\n", "delayRM1us", timing_delayRM1us);
            fprintf(fplog, "%-20s  %u\n", "NBave", timing_NBave);
            fprintf(fplog, "%-20s  %u\n", "NBexcl", timing_NBexcl);
            fprintf(fplog, "%-20s  %s\n", "IDpokeC_name", imgin.md->name);
            fprintf(fplog, "%-20s  %ld\n", "NBcycle", timing_NBcycle);
            fprintf(fplog, "%-20s  %u\n", "SequInitMode", SequInitMode);
            fclose(fplog);

            // Save individual time step within averaging for high temporal
            // resolution
            for(uint32_t AveStep = 0; AveStep < timing_NBave; AveStep++)
            {

                // save to disk IDoutCstep[AveStep]

                char imname[STRINGMAXLEN_IMGNAME];
                WRITE_IMAGENAME(imname, "imoutStep%03u", AveStep);

                char tmpfname[STRINGMAXLEN_FULLFILENAME];
                WRITE_FULLFILENAME(tmpfname,
                                   "%s/wfsresp.tstep%03u.iter%03lu.fits",
                                   outdir,
                                   AveStep,
                                   pkinf.iter);
                printf("SAVING %s -> %s ... ", imname, tmpfname);
                fflush(stdout);
                save_fits(imname, tmpfname);
                printf("DONE\n");
                fflush(stdout);
            }
        }

        pkinf.iter++;



        // compile and save
        {
            char tmpoutfname[STRINGMAXLEN_FULLFILENAME];
            WRITE_FULLFILENAME(tmpoutfname, "%s/mode_linresp.fits", outdir);

            for(uint32_t PokeIndex = 0; PokeIndex < NBmode2; PokeIndex++)
            {
                for(uint64_t ii = 0; ii < sizexyout; ii++)
                {
                    imgoutC.im->array.F[PokeIndex * sizexyout + ii] /= timing_NBave * pkinf.iter *
                    ampl;
                }
            }

            save_fits(imgoutC.name, tmpoutfname);

        }


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END



    // print poke log
    {
        char tmpfname[STRINGMAXLEN_FULLFILENAME];
        if(strlen(outdir) > 0)
        {
            WRITE_FULLFILENAME(tmpfname, "%s/RMpokelog.txt", outdir);
        }
        else
        {
            WRITE_FULLFILENAME(tmpfname, "RMpokelog.txt");
        }

        FILE *fp = fopen(tmpfname, "w");
        for(pokecnt = 0; pokecnt < NBpokeTotal; pokecnt++)
        {
            fprintf(fp,
                    "%6lu %3lu    %1u %1u     %6u  %6d     %4u %4u   %4u %4u     "
                    "%3ld %3u %3u\n",
                    pokecnt,
                    pkinfarray[pokecnt].iter,
                    pkinfarray[pokecnt].poke,
                    pkinfarray[pokecnt].accum,
                    pkinfarray[pokecnt].aveindex,
                    pkinfarray[pokecnt].pokeframeindex,
                    pkinfarray[pokecnt].PokeIndexMEAS,
                    pkinfarray[pokecnt].PokeIndexCTRL,
                    pkinfarray[pokecnt].PokeIndexMEAS_Mapped,
                    pkinfarray[pokecnt].PokeIndexCTRL_Mapped,
                    NBmode2,
                    timing_NBexcl,
                    timing_NBave);
        }
        fclose(fp);
    }



    {
        char tmpfname[STRINGMAXLEN_FULLFILENAME];

        printf("Writing poke timing to file ... ");
        fflush(stdout);

        WRITE_FULLFILENAME(tmpfname, "%s/RMpokeTiming.txt", outdir);
        FILE *fp            = fopen(tmpfname, "w");
        double ftime0 = pkinfarray[0].ts.tv_sec + 1.0e-9 * pkinfarray[0].ts.tv_nsec;
        double ftime;
        for(uint64_t ii = 0; ii < pokecnt; ii++)
        {
            ftime = pkinfarray[ii].ts.tv_sec + 1.0e-9 * pkinfarray[ii].ts.tv_nsec;
            fprintf(fp,
                    "%5lu  %16ld.%09ld  %5d  %12.9lf\n",
                    ii,
                    pkinfarray[ii].ts.tv_sec,
                    pkinfarray[ii].ts.tv_nsec,
                    pkinfarray[ii].PokeIndexMEAS,
                    ftime - ftime0);
            ftime0 = ftime;
        }
        fclose(fp);
    }


    free(IDoutCstep);

    free(array_PokeSequ);

    free(pkinfarray);



    return EXIT_SUCCESS;
}









static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // connect to input space
    IMGID imgin = mkIMGID_from_name(streamin);
    resolveIMGID(&imgin, ERRMODE_ABORT);
    printf("input  space size : %u %u\n", imgin.md->size[0], imgin.md->size[1]);

    // connect to output space
    IMGID imgout = mkIMGID_from_name(streamout);
    resolveIMGID(&imgout, ERRMODE_ABORT);
    printf("output space size : %u %u\n", imgout.md->size[0], imgout.md->size[1]);

    load_fits(inmodeC, "inmodeC", LOADFITS_ERRMODE_WARNING, NULL);
    IMGID imginmodeC = mkIMGID_from_name("inmodeC");
    resolveIMGID(&imginmodeC, ERRMODE_ABORT);
    printf("input modes size : %u %u %u\n", imginmodeC.md->size[0],
           imginmodeC.md->size[1], imginmodeC.md->size[2]);


    // TODO Check that DM size matches poke file

    Measure_Linear_Response_Modal(
        imgin,
        imgout,
        imginmodeC,
        *pokeampl,
        *delayfr,
        *delayRM1us,
        *NBave,
        *NBexcl,
        1,
        outmodeC,
        "measlinrespmdir"
    );

    list_image_ID();

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}




INSERT_STD_FPSCLIfunctions




errno_t CLIADDCMD_AOloopControl__measure_linear_resp()
{
    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
