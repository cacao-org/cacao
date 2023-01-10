/**
 * @file    measure_linear_respm.c
 * @brief   Measure linear response to perturbation
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_iofits/COREMOD_iofits.h"


// Holds info for each poke frame
// A poke frame starts with a stream read.
// The input stream may is poked during the pokeframe.
// If so, it is done with a time delay from the start of the pokeframe (= stream read)
//
// The PokeInfo structure contains the following information
//
// Measurement :
// - [PokeIndexMEAS]        Mode index being measured (-1 if none). This increments during the sequence.
// - [PokeIndexMEAS_Mapped] Original mode to to which PokeIndexMEAS maps. This generally does not increase monotically.
// - [aveindex]             Average index. If averaging 3 frames per measurement, this will increment from 0 to 2. If >2, discard frame
//
// Control
// - [PokeIndexCTRL]        Mode to be poked during the poke frame. This increments during the sequence.
// - [PokeIndexCTRL_Mapped] Original mode to to which PokeIndexCTRL maps. This generally does not increase monotically.
// - [pokedelayns]          Delay [ns] from poke frame start to poking
//
// Timing log
// - [tstart]               Start of poke frame
// - [tpoke]                Time at which poke was completed
//
// The sequence of PokeIndexMEAS and PokeIndexCTRL values follows the same pattern, but offset in time to account for latency between the streams.
// The integer offset bettween the sequences, and the pokedelayns value, are derived from the framerate and latency between streams.
// We follow here the latency convention that if the poke responds instantaneously and the read is at full duty cycle, the latency is 0.5 frame.
//
// [lantencyfr] Latency in unit of frame
// [framerateHz]  Frame rate in Hz
// [Nexcl]      Number of excluded frames
//
// We define the start of a poke cycle as the arrival of the first frame (aveindex=0)
// This is followed by a number of measurement frames to be included (aveindex=1, 2, ... NBave-1) and excluded (aveindex = NBave ... NBave-Nexcl)
//
// The time offset between poke actuation and start of poke cycle is :
// dt1 [fr] = (latencyfr-0.5) + Nexcl/2
// positive value indicates the DM poke occurs before cycle start
//
// dt1 is then split into an integer offset [RMdelayfr] and the pokedelayns [delayMR1ns] :
//
// RMdelayfr = ceil(dt1)
// delayMR1ns = (RMdelayfr-dt1) * (1/framerateHz)*1000000.0
//


typedef struct
{
    // Poke mode being measured
    int PokeIndexMEAS;

    // Current poke mode on input
    int PokeIndexCTRL;

    // Poke mode being measured, index in poke cube
    int PokeIndexMEAS_Mapped;

    // Current poke mode on DM, index in poke cube
    int PokeIndexCTRL_Mapped;

    // frame index within poke mode acquisition
    // -1 if not part of signal
    int aveindex;

    // Time delay between start of pokeframe and poke actuation command, unit: nanosec
    int pokedelayns;

    struct timespec tstart;
    struct timespec tpoke;


    /*

        // Cycle number
        uint64_t iter;

        // Did we poke DM during this time interval ?
        int poke;

        // Does frame count toward accumulated signal ?
        int accum;



        // frame counter within poke mode acquisition, starts negative
        // becomes positive when accumulating signal
        int pokeframeindex;

        struct timespec ts;
    */
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

// Frame rate [Hz]
static float *timing_framerateHz;
long fpi_timing_framerateHz;

// Latency in unit of WFS frame
static float *timing_latencyfr;
long fpi_timing_latencyfr;

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
        (void **) &timing_framerateHz,
        &fpi_timing_framerateHz
    },
    {
        CLIARG_FLOAT32,
        ".timing.hardwlatfr",
        "hardware latency [fr]",
        "1000",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &timing_latencyfr,
        &fpi_timing_latencyfr
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
 * frames (timing_delayfr) and an additional time delay in microsecond (timing_delayRM1ns).
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
 * @param framerateHz          Frame rate [Hz]
 * @param latencyfr            Latency [fr]
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
    float       framerateHz,
    float       latencyfr,
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

    printf("    framerateHz       : %f\n", framerateHz);
    printf("    latencyfr         : %f\n", latencyfr);
    printf("    timing_NBave      : %u\n", timing_NBave);
    printf("    timing_NBexcl     : %u\n", timing_NBexcl);
    printf("    SequInitMode      : %u\n", SequInitMode);



    // Compute timing parameters
    // Time offset between poke actuation and start of poke cycle is

    float dtfr = (latencyfr - 0.5) + 0.5 * timing_NBexcl;
    // positive value indicates the DM poke occurs before cycle start

    int RMdelayfr = 0;
    int delayMR1ns = 0;
    if(dtfr > 0.0)
    {
        // dtfr is then split into an integer offset [RMdelayfr] and the pokedelayns [delayMR1ns] :
        RMdelayfr = ceil(dtfr);
        delayMR1ns = (int)((1.0 * RMdelayfr - dtfr) * (1.0 / framerateHz) * 1.0e9 +
                           0.5);
    }

    printf("dtfr       = %f\n", dtfr);
    printf("RMdelayfr  = %d\n", RMdelayfr);
    printf("delayMR1ns = %d\n", delayMR1ns);


    // number of poke frames
    uint64_t NBpokeframe;
    NBpokeframe  = timing_NBave + timing_NBexcl;  // for each mode
    NBpokeframe *= NBmode2;                       // multiplied by number of modes
    NBpokeframe += RMdelayfr;                // to allow for latency at startup

    pkinfarray = (PokeInfo *) malloc(sizeof(PokeInfo) * NBpokeframe);


    // initialization
    for(uint64_t pokeframe = 0; pokeframe < NBpokeframe; pokeframe ++)
    {
        // poke first mode
        pkinfarray[pokeframe].PokeIndexCTRL = 0;
        pkinfarray[pokeframe].pokedelayns   = delayMR1ns;

        // don't measure anything
        pkinfarray[pokeframe].PokeIndexMEAS = -1;
    }

    for(int pokemode = 0; pokemode < NBmode2; pokemode++)
    {
        for(int aveindex = 0; aveindex < timing_NBave + timing_NBexcl; aveindex++)
        {
            int pokeCTRLindex = pokemode * (timing_NBave + timing_NBexcl) + aveindex;
            int pokeMEASindex = pokeCTRLindex + RMdelayfr;

            pkinfarray[pokeCTRLindex].PokeIndexCTRL = pokemode;
            pkinfarray[pokeMEASindex].PokeIndexMEAS = pokemode;
            pkinfarray[pokeMEASindex].aveindex = aveindex;
        }
    }








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
    IMGID imgoutC2 = makeIMGID_3D("tmpmoderespraw", sizexout, sizeyout, NBmode2);
    imageID IDoutC2 = createimagefromIMGID(&imgoutC2);

    for(uint32_t PokeIndex = 0; PokeIndex < NBmode2; PokeIndex++)
    {
        // Mode to be poked

        for(uint64_t ii = 0; ii < imgout.md->size[0]*imgout.md->size[1]; ii++)
        {
            imgoutC2.im->array.F[PokeIndex * sizexyout + ii] = 0.0;
            for(uint32_t AveStep = 0; AveStep < timing_NBave; AveStep++)
            {
                data.image[IDoutCstep[AveStep]].array.F[PokeIndex * sizexyout + ii] = 0.0;
            }
        }
    }





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


    int iter = 0;



    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART
    {
        printf("ITERATION %d\n", iter);
        fflush(stdout);

        processinfo_WriteMessage_fmt(processinfo, "it %lu/%lu",
                                     iter,
                                     processinfo->loopcntMax
                                    );




        // swap pokes pairs
        //
        if(SequInitMode & 0x02)
        {
            printf("SWAPPING, MODE 2\n");
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







        for(uint64_t pokeframe = 0; pokeframe < NBpokeframe; pokeframe ++)
        {

            processinfo_WriteMessage_fmt(processinfo, "it %lu/%lu pokeframe %6lu / %6lu",
                                         iter,
                                         processinfo->loopcntMax,
                                         pokeframe,
                                         NBpokeframe
                                        );

            pkinfarray[pokeframe].PokeIndexCTRL_Mapped =
                array_PokeSequ[pkinfarray[pokeframe].PokeIndexCTRL];

            if(pkinfarray[pokeframe].PokeIndexMEAS != -1)
            {
                pkinfarray[pokeframe].PokeIndexMEAS_Mapped =
                    array_PokeSequ[pkinfarray[pokeframe].PokeIndexMEAS];
            }
            else
            {
                pkinfarray[pokeframe].PokeIndexMEAS_Mapped = -1;
            }



            // Read stream
            //
            //printf("%5lu  waiting for frame\n", pokeframe);

            ImageStreamIO_semwait(imgout.im, semindexout);
            clock_gettime(CLOCK_REALTIME, &pkinfarray[pokeframe].tstart);



            // Wait for time delay
            //
            //printf("%5lu  waiting for delay %d ns\n", pokeframe, pkinfarray[pokeframe].pokedelayns);
            {
                long nsec           = pkinfarray[pokeframe].pokedelayns;
                long nsec_remaining = nsec % 1000000000;
                long sec            = nsec / 1000000000;

                struct timespec timesleep;
                timesleep.tv_sec  = sec;
                timesleep.tv_nsec = nsec_remaining;

                nanosleep(&timesleep, NULL);
            }


            //printf("%5lu Poking\n", pokeframe);
            // Poke
            //
            imgin.md->write = 1;
            memcpy((void *)(imgin.im->array.F),
                   (void *)(ptr0 + pkinfarray[pokeframe].PokeIndexCTRL_Mapped * framesize),
                   sizeof(float) * sizexyin);
            imgin.md->cnt1 = pkinfarray[pokeframe].PokeIndexCTRL_Mapped;
            processinfo_update_output_stream(processinfo, imgin.ID);
            clock_gettime(CLOCK_REALTIME, &pkinfarray[pokeframe].tpoke);


            // Collect signal
            //
            if((pkinfarray[pokeframe].aveindex < timing_NBave)
                    && (pkinfarray[pokeframe].PokeIndexMEAS_Mapped != -1))
            {
                {
                    char *ptr = (char *)
                                data.image[IDoutCstep[pkinfarray[pokeframe].aveindex]].array.F;
                    ptr += sizeof(float) * pkinfarray[pokeframe].PokeIndexMEAS_Mapped * sizexyout;
                    memcpy(ptr,
                           imgout.im->array.F,
                           sizeof(float) * sizexyout);
                }
            }
        }


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
                    imgoutC2.im->array.F[pkinf.PokeIndexMEAS_Mapped * sizexyout + ii] +=
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
            fprintf(fplog, "%-20s  %f\n", "framerateHz", framerateHz);
            fprintf(fplog, "%-20s  %f\n", "latencyfr", latencyfr);
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
                                   "%s/wfsresp.tstep%03u.iter%04d.fits",
                                   outdir,
                                   AveStep,
                                   iter);
                printf("SAVING %s -> %s ... ", imname, tmpfname);
                fflush(stdout);
                save_fits(imname, tmpfname);
                printf("DONE\n");
                fflush(stdout);
            }
        }





        // print poke log
        {
            char tmpfname[STRINGMAXLEN_FULLFILENAME];
            if(strlen(outdir) > 0)
            {
                WRITE_FULLFILENAME(tmpfname, "%s/RMpokelog.iter%04d.txt", outdir, iter);
            }
            else
            {
                WRITE_FULLFILENAME(tmpfname, "RMpokelog.iter%04d.txt", iter);
            }

            FILE *fp = fopen(tmpfname, "w");
            for(int pokeframe = 0; pokeframe < NBpokeframe; pokeframe++)
            {
                fprintf(fp,
                        "%6d %3d    %4d %4d   %4d %4d     %3ld %3u %3u\n",
                        pokeframe,
                        pkinfarray[pokeframe].aveindex,
                        pkinfarray[pokeframe].PokeIndexMEAS,
                        pkinfarray[pokeframe].PokeIndexCTRL,
                        pkinfarray[pokeframe].PokeIndexMEAS_Mapped,
                        pkinfarray[pokeframe].PokeIndexCTRL_Mapped,
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

            WRITE_FULLFILENAME(tmpfname, "%s/RMpokeTiming.iter%04d.txt", outdir, iter);
            FILE *fp            = fopen(tmpfname, "w");
            double ftime0 = pkinfarray[0].tstart.tv_sec + 1.0e-9 * pkinfarray[0].tstart.tv_nsec;
            double ftime;
            for(uint64_t ii = 0; ii < NBpokeframe; ii++)
            {
                ftime = pkinfarray[ii].tstart.tv_sec + 1.0e-9 * pkinfarray[ii].tstart.tv_nsec;
                fprintf(fp,
                        "%5lu  %16ld.%09ld  %16ld.%09ld  %5d  %12.9lf\n",
                        ii,
                        pkinfarray[ii].tstart.tv_sec,
                        pkinfarray[ii].tstart.tv_nsec,
                        pkinfarray[ii].tpoke.tv_sec,
                        pkinfarray[ii].tpoke.tv_nsec,
                        pkinfarray[ii].PokeIndexMEAS,
                        ftime - ftime0);
                ftime0 = ftime;
            }
            fclose(fp);
        }



        iter++;



        // compile and save
        {
            char tmpoutfname[STRINGMAXLEN_FULLFILENAME];
            WRITE_FULLFILENAME(tmpoutfname, "%s/mode_linresp_raw.fits", outdir);

/*            for(uint32_t PokeIndex = 0; PokeIndex < NBmode2; PokeIndex++)
            {
                for(uint64_t ii = 0; ii < sizexyout; ii++)
                {
                    imgoutC2.im->array.F[PokeIndex * sizexyout + ii] /= timing_NBave * iter * ampl;
                }
            }
*/
            save_fits(imgoutC2.name, tmpoutfname);


            WRITE_FULLFILENAME(tmpoutfname, "%s/mode_linresp.fits", outdir);
            IMGID imgmoderespC = makeIMGID_3D("moderespC", sizexout, sizeyout, NBmode);
            imageID IDinmoderespC = createimagefromIMGID(&imgmoderespC);

            for(int mode = 0; mode < NBmode; mode++)
            {
                for(uint64_t ii = 0; ii < sizexyout; ii++)
                {
                    float posval = imgoutC2.im->array.F[(mode * 2) * sizexyout + ii];
                    float negval = imgoutC2.im->array.F[(mode * 2 + 1) * sizexyout + ii];
                    imgmoderespC.im->array.F[ mode * sizexyout + ii ] = (posval - negval) / 2 / (timing_NBave * iter * ampl);

                }

            }
            save_fits(imgmoderespC.name, outCname);
        }






    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END







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
        *timing_framerateHz,
        *timing_latencyfr,
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
