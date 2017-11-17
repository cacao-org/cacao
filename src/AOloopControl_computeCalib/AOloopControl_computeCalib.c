/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    26 Aug 2017
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>

#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t) {
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    uint64_t time;
    time = mach_absolute_time();
    double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
    double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
    t->tv_sec = seconds;
    t->tv_nsec = nseconds;
    return 0;
}
#else
#include <time.h>
#endif


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>


#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"
#include "linopt_imtools/linopt_imtools.h"
#include "statistic/statistic.h"
#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "AOloopControl_acquireCalib/AOloopControl_acquireCalib.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"


#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif


/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

#define MAX_MBLOCK 20

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */

extern int PIXSTREAM_NBSLICES;
extern long aoconfID_pixstream_wfspixindex;;

extern int *DM_active_map;
extern int *WFS_active_map;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c

extern int AOloopcontrol_meminit;











// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 5. COMPUTING CALIBRATION
 *  Compute control matrix, modes */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_mkSlavedAct */
int_fast8_t AOloopControl_computeCalib_mkSlavedAct_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)+CLI_checkarg(3,3)==0) {
        AOloopControl_computeCalib_mkSlavedAct(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.string);
        return 0;
    }
    else	return 1;
}

/** @brief CLI function for AOloopControl_mkloDMmodes */
int_fast8_t AOloopControl_computeCalib_mkloDMmodes_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,2)==0) {
        AOloopControl_computeCalib_mkloDMmodes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkCM */
int_fast8_t AOloopControl_computeCalib_mkCM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,1)==0) {
        AOloopControl_computeCalib_mkCM(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkModes */
int_fast8_t AOloopControl_computeCalib_mkModes_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,2)+CLI_checkarg(11,2)+CLI_checkarg(12,1)==0) {
        AOloopControl_computeCalib_mkModes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numl, data.cmdargtoken[11].val.numl, data.cmdargtoken[12].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkModes_Simple */
int_fast8_t AOloopControl_computeCalib_mkModes_Simple_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)==0) {
        AOloopControl_computeCalib_mkModes_Simple(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_computeCM */
int_fast8_t AOloopControl_computeCalib_computeCM_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,4)+CLI_checkarg(3,3)+CLI_checkarg(4,1)+CLI_checkarg(5,2)+CLI_checkarg(6,1)==0) {
        AOloopControl_computeCalib_compute_ControlMatrix(LOOPNUMBER, data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, "evecM", data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numf);
        save_fits("evecM","!evecM.fits");
        delete_image_ID("evecM");
    } else return 1;
}

/** @brief CLI function for AOloopControl_loadCM */
int_fast8_t AOloopControl_computeCalib_loadCM_cli() {
    if(CLI_checkarg(1,3)==0) {
        AOloopControl_computeCalib_loadCM(LOOPNUMBER, data.cmdargtoken[1].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkHadamardModes */
int_fast8_t AOloopControl_computeCalib_mkHadamardModes_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)==0) {
        AOloopControl_computeCalib_mkHadamardModes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_Hadamard_decodeRM */
int_fast8_t AOloopControl_computeCalib_Hadamard_decodeRM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,3)==0) {
        AOloopControl_computeCalib_Hadamard_decodeRM(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkCalib_map_mask */
int_fast8_t AOloopControl_computeCalib_mkCalib_map_mask_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,3)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,1)+CLI_checkarg(11,1)==0) {
        AOloopControl_computeCalib_mkCalib_map_mask(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numf, data.cmdargtoken[11].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_Process_zrespM */
int_fast8_t AOloopControl_computeCalib_Process_zrespM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,3)==0) {
        AOloopControl_computeCalib_Process_zrespM(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_ProcessZrespM */
int_fast8_t AOloopControl_computeCalib_ProcessZrespM_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,3)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,1)+CLI_checkarg(6,2)==0) {
        AOloopControl_computeCalib_ProcessZrespM_medianfilt(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_compute_CombinedControlMatrix */
int_fast8_t AOloopControl_computeCalib_compute_CombinedControlMatrix_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(5,3)+CLI_checkarg(6,3)==0) {
        AOloopControl_computeCalib_compute_CombinedControlMatrix(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.string);
        return 0;
    }
    else return 1;
}

















/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools functions */


void __attribute__ ((constructor)) libinit_AOloopControl_computeCalib()
{
	init_AOloopControl_computeCalib();
//	printf(" ...... Loading module %s\n", __FILE__);
}

            
int_fast8_t init_AOloopControl_computeCalib()
{

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "cacao   - AO loop control compute calibration");
    data.NBmodule++;





/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


    RegisterCLIcommand("aolmkH", __FILE__, AOloopControl_computeCalib_mkHadamardModes_cli, "make Hadamard poke sequence", "<DM pixel mask> <output fname [string]>", "aolmkH dm50mask h50pokec", "long AOloopControl_computeCalib_mkHadamardModes(char *DMmask_name, char outname)");

    RegisterCLIcommand("aolHaddec", __FILE__, AOloopControl_computeCalib_Hadamard_decodeRM_cli, "decode Hadamard matrix", "<input RM> <Hadamard matrix> <DMpix index frame> <output RM>", "aolHaddec imRMh Hmat pixiind imRM", "long AOloopControl_computeCalib_Hadamard_decodeRM(char *inname, char *Hmatname, char *indexname, char *outname)");

    RegisterCLIcommand("aolmkslact",__FILE__, AOloopControl_computeCalib_mkSlavedAct_cli, "create slaved actuators map based on proximity", "<maskRM> <distance> <outslact>", "aolmkslact DMmaskRM 2.5 DMslavedact", "long AOloopControl_computeCalib_mkSlavedAct(char *IDmaskRM_name, float pixrad, char *IDout_name)");

    RegisterCLIcommand("aolmklodmmodes",__FILE__, AOloopControl_computeCalib_mkloDMmodes_cli, "make low order DM modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode>", "aolmklodmmodes modes 50 50 5.0 0.8 1", "long AOloopControl_computeCalib_mkloDMmodes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode)");

    RegisterCLIcommand("aolRM2CM",__FILE__, AOloopControl_computeCalib_mkCM_cli, "make control matrix from response matrix", "<RMimage> <CMimage> <SVDlim>", "aolRM2CM respM contrM 0.1", "long AOloopControl_computeCalib_mkCM(char *respm_name, char *cm_name, float SVDlim)");

    RegisterCLIcommand("aolmkmodes", __FILE__, AOloopControl_computeCalib_mkModes_cli, "make control modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode> <block> <SVDlim>", "aolmkmodes modes 50 50 5.0 0.8 1 2 0.01", "long AOloopControl_computeCalib_mkModes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode, int BlockNB, float SVDlim)");

    RegisterCLIcommand("aolmkmodesM", __FILE__, AOloopControl_computeCalib_mkModes_Simple_cli, "make control modes in modal DM mode", "<input WFS modes> <NBmblock> <block> <SVDlim>", "aolmkmodesM wfsallmodes 5 2 0.01", "long AOloopControl_computeCalib_mkModes_Simple(char *IDin_name, long NBmblock, long Cmblock, float SVDlim)");

    RegisterCLIcommand("aolRMmkmasks",__FILE__, AOloopControl_computeCalib_mkCalib_map_mask_cli, "make sensitivity maps and masks from response matrix", "<zrespm fname [string]> <output WFS response map fname [string]>  <output DM response map fname [string]> <percentile low> <coefficient low> <percentile high> <coefficient high>", "aolRMmkmasks .. 0.2 1.0 0.5 0.3 0.05 1.0 0.65 0.3", "int AOloopControl_computeCalib_mkCalib_map_mask(long loop, char *zrespm_name, char *WFSmap_name, char *DMmap_name, float dmmask_perclow, float dmmask_coefflow, float dmmask_perchigh, float dmmask_coeffhigh, float wfsmask_perclow, float wfsmask_coefflow, float wfsmask_perchigh, float wfsmask_coeffhigh)");

    RegisterCLIcommand("aolproczrm",__FILE__, AOloopControl_computeCalib_Process_zrespM_cli, "process zonal resp mat, WFS ref -> DM and WFS response maps", "<input zrespm fname [string]> <input WFS ref fname [string]> <output zrespm [string]> <output WFS response map fname [string]>  <output DM response map fname [string]>", "aolproczrm zrespmat0 wfsref0 zrespm wfsmap dmmap", "int AOloopControl_computeCalib_Process_zrespM(long loop, char *IDzrespm0_name, char *IDwfsref_name, char *IDzrespm_name, char *WFSmap_name, char *DMmap_name)");

    RegisterCLIcommand("aolcleanzrm",__FILE__, AOloopControl_computeCalib_ProcessZrespM_cli, "clean zonal resp mat, WFS ref, DM and WFS response maps", "<zrespm fname [string]> <output WFS ref fname [string]>  <output WFS response map fname [string]>  <output DM response map fname [string]> <RM ampl [um]>", "aolcleanzrm zrm wfsref wfsmap dmmap 0.05", "int AOloopControl_computeCalib_ProcessZrespM(long loop, char *zrespm_name, char *WFSref0_name, char *WFSmap_name, char *DMmap_name, double ampl)");

    RegisterCLIcommand("aolcompcmatc",__FILE__, AOloopControl_computeCalib_compute_CombinedControlMatrix_cli, "compute combined control matrix", "<modal control matrix> <modes> <wfs mask> <dm mask> <combined cmat> <combined cmat, only active elements>", "aolcompcmatc cmat fmodes wfsmask dmmask cmatc cmatcact", "long AOloopControl_computeCalib_compute_CombinedControlMatrix(char *IDcmat_name, char *IDmodes_name, char* IDwfsmask_name, char *IDdmmask_name, char *IDcmatc_name, char *IDcmatc_active_name)");

    RegisterCLIcommand("aolcmmake",__FILE__, AOloopControl_computeCalib_computeCM_cli, "make control matrix", "<NBmodes removed> <RespMatrix> <ContrMatrix> <beta> <nbremovedstep> <eigenvlim>", "aolcmmake 8 respm cmat", "int AOloopControl_computeCalib_compute_ControlMatrix(long loop, long NB_MODE_REMOVED, char *ID_Rmatrix_name, char *ID_Cmatrix_name, char *ID_VTmatrix_name, double Beta, long NB_MODE_REMOVED_STEP, float eigenvlim)");



    RegisterCLIcommand("aolloadcm", __FILE__, AOloopControl_computeCalib_loadCM_cli, "load new control matrix from file", "<fname>", "aolloadcm cm32.fits", "long AOloopControl_computeCalib_loadCM(long loop, const char *CMfname)");




    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}











/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION                                                 */
/* =============================================================================================== */
/* =============================================================================================== */



// output:
// Hadamard modes (outname)
// Hadamard matrix ("Hmat.fits")
// pixel indexes ("Hpixindex.fits", float, to be converted to long)
long AOloopControl_computeCalib_mkHadamardModes(const char *DMmask_name, const char *outname)
{
    long IDout;
    long xsize, ysize, xysize;
    //    long IDdisk;
    long cnt;

    long Hsize;
    int n2max;

    long *indexarray;
    long index;
    long IDtest;
    int *Hmat;
    long k, ii, jj, n, n2, i, j;
    long IDindex;
    uint32_t *sizearray;

    long IDmask;



    IDmask = image_ID(DMmask_name);
    xsize = data.image[IDmask].md[0].size[0];
    ysize = data.image[IDmask].md[0].size[1];
    xysize = xsize*ysize;

    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
    sizearray[0] = xsize;
    sizearray[1] = ysize;
    IDindex = create_image_ID("Hpixindex", 2, sizearray, _DATATYPE_FLOAT, 0, 0);
    free(sizearray);

    cnt = 0;
    for(ii=0; ii<xysize; ii++)
        if(data.image[IDmask].array.F[ii]>0.5)
            cnt++;

    Hsize = 1;
    n2max = 0;
    while(Hsize<cnt)
    {
        Hsize *= 2;
        n2max++;
    }
    n2max++;

    printf("Hsize n2max = %ld  %d\n", Hsize, n2max);
    fflush(stdout);

    for(ii=0; ii<xysize; ii++)
        data.image[IDindex].array.F[ii] = -10.0;

    index = 0;

    indexarray = (long*) malloc(sizeof(long)*Hsize);
    for(k=0; k<Hsize; k++)
        indexarray[k] = -1;
    for(ii=0; ii<xysize; ii++)
        if((data.image[IDmask].array.F[ii]>0.5)&&(index<Hsize))
        {

            indexarray[index] = ii;
            // printf("(%ld %ld)  ", index, ii);

            data.image[IDindex].array.F[ii] = 1.0*index;

            index++;
        }
    save_fits("Hpixindex", "!Hpixindex.fits.gz");

    Hmat = (int*) malloc(sizeof(int)*Hsize*Hsize);



    // n = 0

    ii = 0;
    jj = 0;
    Hmat[jj*Hsize+ii] = 1;
    n2=1;
    for(n=1; n<n2max; n++)
    {
        for(ii=0; ii<n2; ii++)
            for(jj=0; jj<n2; jj++)
            {
                Hmat[ jj*Hsize + (ii+n2)] = Hmat[ jj*Hsize + ii];
                Hmat[ (jj+n2)*Hsize + (ii+n2)] = -Hmat[ jj*Hsize + ii];
                Hmat[ (jj+n2)*Hsize + ii] = Hmat[ jj*Hsize + ii];
            }
        n2 *= 2;
    }

    printf("n2 = %ld\n", n2);
    fflush(stdout);

    IDtest = create_2Dimage_ID("Htest", Hsize, Hsize);

    for(ii=0; ii<Hsize; ii++)
        for(jj=0; jj<Hsize; jj++)
            data.image[IDtest].array.F[jj*Hsize+ii] = Hmat[jj*Hsize+ii];

    save_fits("Htest", "!Hmat.fits.gz");


    IDout = create_3Dimage_ID(outname, xsize, ysize, Hsize);
    for(k=0; k<Hsize; k++)
    {
        for(index=0; index<Hsize; index++)
        {
            ii = indexarray[index];
            data.image[IDout].array.F[k*xysize+ii] = Hmat[k*Hsize+index];
        }
    }

    free(Hmat);

    free(indexarray);


    return(IDout);
}




long AOloopControl_computeCalib_Hadamard_decodeRM(const char *inname, const char *Hmatname, const char *indexname, const char *outname)
{
    long IDin, IDhad, IDout, IDindex;
    long NBact, NBframes, sizexwfs, sizeywfs, sizewfs;
    long kk, kk1, ii;
    uint32_t zsizeout;



    IDin = image_ID(inname);
    sizexwfs = data.image[IDin].md[0].size[0];
    sizeywfs = data.image[IDin].md[0].size[1];
    sizewfs = sizexwfs*sizeywfs;
    NBframes = data.image[IDin].md[0].size[2];

    IDindex = image_ID(indexname);



    IDhad = image_ID(Hmatname);
    if((data.image[IDhad].md[0].size[0]!=NBframes)||(data.image[IDhad].md[0].size[1]!=NBframes))
    {
        printf("ERROR: size of Hadamard matrix [%ld x %ld] does not match available number of frames [%ld]\n", (long) data.image[IDhad].md[0].size[0], (long) data.image[IDhad].md[0].size[1], NBframes);
        exit(0);
    }

    zsizeout = data.image[IDindex].md[0].size[0]*data.image[IDindex].md[0].size[1];
    IDout = create_3Dimage_ID(outname, sizexwfs, sizeywfs, zsizeout);

    long kk0;
# ifdef _OPENMP
    #pragma omp parallel for private(kk0,kk1,ii)
# endif
    for(kk=0; kk<zsizeout; kk++) // output frame
    {
        kk0 = (long) (data.image[IDindex].array.F[kk]+0.1);
        if(kk0 > -1)
        {   printf("\r  frame %5ld / %5ld     ", kk0, NBframes);
            fflush(stdout);
            for(kk1=0; kk1<NBframes; kk1++)
            {
                for(ii=0; ii<sizewfs; ii++)
                    data.image[IDout].array.F[kk*sizewfs+ii] += data.image[IDin].array.F[kk1*sizewfs+ii]*data.image[IDhad].array.F[kk0*NBframes+kk1];
            }
        }
    }

    for(kk=0; kk<zsizeout; kk++)
    {
        for(ii=0; ii<sizewfs; ii++)
            data.image[IDout].array.F[kk*sizewfs+ii] /= NBframes;

    }

    printf("\n\n");


    return(IDout);
}




// make low order DM modes
long AOloopControl_computeCalib_mkloDMmodes(const char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yc, double r0, double r1, int MaskMode)
{
    long IDmask;
    long ID, ID0, IDtm, IDem, IDtmpg, IDslaved;
    long ii, jj;
    double x, y, r, PA, xc1, yc1, totm, offset, rms, sigma;

    long NBZ, m;
    long zindex[10];
    double zcpa[10];  /// CPA for each Zernike (somewhat arbitrary... used to sort modes in CPA)
    long IDfreq, IDmfcpa;
    long k;

    double coeff;
    long kelim = 20;
    long conviter;




    zindex[0] = 1; // tip
    zcpa[0] = 0.0;

    zindex[1] = 2; // tilt
    zcpa[1] = 0.0;

    zindex[2] = 4; // focus
    zcpa[2] = 0.25;

    zindex[3] = 3; // astig
    zcpa[3] = 0.4;

    zindex[4] = 5; // astig
    zcpa[4] = 0.4;

    zindex[5] = 7; // coma
    zcpa[5] = 0.6;

    zindex[6] = 8; // coma
    zcpa[6] = 0.6;

    zindex[7] = 6; // trefoil
    zcpa[7] = 1.0;

    zindex[8] = 9; // trefoil
    zcpa[8] = 1.0;

    zindex[9] = 12;
    zcpa[9] = 1.5;


    printf("msizexy = %ld %ld\n", msizex, msizey);
    list_image_ID();
    IDmask = image_ID("dmmask");
    if(IDmask==-1)
    {
        double val0, val1;
        double a0=0.88;
        double b0=40.0;
        double a1=1.2;
        double b1=12.0;

        IDmask = create_2Dimage_ID("dmmask", msizex, msizey);
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                x = 1.0*ii-xc;
                y = 1.0*jj-yc;
                r = sqrt(x*x+y*y)/r1;
                val1 = 1.0-exp(-pow(a1*r,b1));
                r = sqrt(x*x+y*y)/r0;
                val0 = exp(-pow(a0*r,b0));
                data.image[IDmask].array.F[jj*msizex+ii] = val0*val1;
            }
        save_fits("dmmask", "!dmmask.fits");
        xc1 = xc;
        yc1 = yc;
    }
    else /// extract xc and yc from mask
    {
        xc1 = 0.0;
        yc1 = 0.0;
        totm = 0.0;
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                xc1 += 1.0*ii*data.image[IDmask].array.F[jj*msizex+ii];
                yc1 += 1.0*jj*data.image[IDmask].array.F[jj*msizex+ii];
                totm += data.image[IDmask].array.F[jj*msizex+ii];
            }
        // printf("xc1 yc1    %f  %f     %f\n", xc1, yc1, totm);
        xc1 /= totm;
        yc1 /= totm;
    }




    totm = arith_image_total("dmmask");
    if((msizex != data.image[IDmask].md[0].size[0])||(msizey != data.image[IDmask].md[0].size[1]))
    {
        printf("ERROR: file dmmask size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmask].md[0].size[0], (long) data.image[IDmask].md[0].size[1], (long) msizex, (long) msizey);
        exit(0);
    }


    NBZ = 0;

    for(m=0; m<10; m++)
    {
        if(zcpa[m]<CPAmax)
            NBZ++;
    }



    linopt_imtools_makeCPAmodes("CPAmodes", msizex, CPAmax, deltaCPA, 0.5*msizex, 1.2, 0);
    ID0 = image_ID("CPAmodes");
    IDfreq = image_ID("cpamodesfreq");



    printf("  %ld %ld %ld\n", msizex, msizey, (long) data.image[ID0].md[0].size[2]-1 );
    ID = create_3Dimage_ID(ID_name, msizex, msizey, data.image[ID0].md[0].size[2]-1+NBZ);





    IDmfcpa = create_2Dimage_ID("modesfreqcpa", data.image[ID0].md[0].size[2]-1+NBZ, 1);

    /*** Create TTF first */
    zernike_init();
    printf("r1 = %f    %f %f\n", r1, xc1, yc1);
    for(k=0; k<NBZ; k++)
    {
        data.image[IDmfcpa].array.F[k] = zcpa[k];
        for(ii=0; ii<msizex; ii++)
            for(jj=0; jj<msizey; jj++)
            {
                x = 1.0*ii-xc1;
                y = 1.0*jj-yc1;
                r = sqrt(x*x+y*y)/r1;
                PA = atan2(y,x);
                data.image[ID].array.F[k*msizex*msizey+jj*msizex+ii] = Zernike_value(zindex[k], r, PA);
            }
    }




    for(k=0; k<data.image[ID0].md[0].size[2]-1; k++)
    {
        data.image[IDmfcpa].array.F[k+NBZ] = data.image[IDfreq].array.F[k+1];
        for(ii=0; ii<msizex*msizey; ii++)
            data.image[ID].array.F[(k+NBZ)*msizex*msizey+ii] = data.image[ID0].array.F[(k+1)*msizex*msizey+ii];
    }


    for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
    {
        /// Remove excluded modes
        long IDeModes = image_ID("emodes");
        if(IDeModes!=-1)
        {
            IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

            for(ii=0; ii<msizex*msizey; ii++)
                data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
            linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmask", 1.0e-2, "lcoeff", 0);
            linopt_imtools_image_construct("emodes", "lcoeff", "em00");
            delete_image_ID("lcoeff");
            IDem = image_ID("em00");

            coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));
            if(k>2.0*kelim)
                coeff = 1.0;
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

            delete_image_ID("em00");
            delete_image_ID("tmpmode");
        }


        double totvm = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            //	  data.image[ID].array.F[k*msize*msize+ii] = data.image[ID0].array.F[(k+1)*msize*msize+ii];
            totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
        }
        offset = totvm/totm;

        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset;
            data.image[ID].array.F[k*msizex*msizey+ii] *= data.image[IDmask].array.F[ii];
        }

        offset = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
            offset += data.image[ID].array.F[k*msizex*msizey+ii];

        rms = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset/msizex/msizey;
            rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii];
        }
        rms = sqrt(rms/totm);
        printf("Mode %ld   RMS = %lf  (%f)\n", k, rms, totm);
        for(ii=0; ii<msizex*msizey; ii++)
            data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
    }


    for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
    {
        rms = 0.0;
        for(ii=0; ii<msizex*msizey; ii++)
        {
            data.image[ID].array.F[k*msizex*msizey+ii] -= offset/msizex/msizey;
            rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii];
        }
        rms = sqrt(rms/totm);
        printf("Mode %ld   RMS = %lf\n", k, rms);
    }



    if(MaskMode==1)
    {
        long kernsize = 5;
        long NBciter = 200;
        long citer;

        if(2*kernsize>msizex)
            kernsize = msizex/2;
        for(citer=0; citer<NBciter; citer++)
        {
            long IDg;

            printf("Convolution [%3ld/%3ld]\n", citer, NBciter);
            gauss_filter(ID_name, "modeg", 4.0*pow(1.0*(NBciter-citer)/NBciter,0.5), kernsize);
            IDg = image_ID("modeg");
            for(k=0; k<data.image[ID].md[0].size[2]; k++)
            {
                for(ii=0; ii<msizex*msizey; ii++)
                    if(data.image[IDmask].array.F[ii]<0.98)
                        data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDg].array.F[k*msizex*msizey+ii];
            }
            delete_image_ID("modeg");
        }
    }




    /// SLAVED ACTUATORS
    IDslaved = image_ID("dmslaved");
    ID = image_ID(ID_name);
    if((IDslaved != -1)&&(IDmask!=-1))
    {
        long IDtmp = create_2Dimage_ID("_tmpinterpol", msizex, msizey);
        long IDtmp1 = create_2Dimage_ID("_tmpcoeff1", msizex, msizey);
        long IDtmp2 = create_2Dimage_ID("_tmpcoeff2", msizex, msizey);
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            // write input DM mode
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];
                data.image[IDtmp1].array.F[ii] = data.image[IDmask].array.F[ii] * (1.0-data.image[IDslaved].array.F[ii]);
                data.image[IDtmp2].array.F[ii] = data.image[IDtmp1].array.F[ii];
            }

            long pixcnt = 1;
            float vxp, vxm, vyp, vym, cxp, cxm, cyp, cym;
            float ctot;

            while(pixcnt>0)
            {
                pixcnt = 0;
                for(ii=1; ii<msizex-1; ii++)
                    for(jj=1; jj<msizey-1; jj++)
                    {
                        if((data.image[IDtmp1].array.F[jj*msizex+ii]<0.5) && (data.image[IDslaved].array.F[jj*msizex+ii]>0.5))
                        {
                            pixcnt ++;
                            vxp = data.image[IDtmp].array.F[jj*msizex+(ii+1)];
                            cxp = data.image[IDtmp1].array.F[jj*msizex+(ii+1)];

                            vxm = data.image[IDtmp].array.F[jj*msizex+(ii-1)];
                            cxm = data.image[IDtmp1].array.F[jj*msizex+(ii-1)];

                            vyp = data.image[IDtmp].array.F[(jj+1)*msizex+ii];
                            cyp = data.image[IDtmp1].array.F[(jj+1)*msizex+ii];

                            vym = data.image[IDtmp].array.F[(jj-1)*msizex+ii];
                            cym = data.image[IDtmp1].array.F[(jj-1)*msizex+ii];

                            ctot = (cxp+cxm+cyp+cym);

                            if(ctot>0.5)
                            {
                                data.image[IDtmp].array.F[jj*msizex+ii] = (vxp*cxp+vxm*cxm+vyp*cyp+vym*cym)/ctot;
                                data.image[IDtmp2].array.F[jj*msizex+ii] = 1.0;
                            }
                        }
                    }
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[IDtmp1].array.F[ii] = data.image[IDtmp2].array.F[ii];
            }
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[ii];






            /*

                    IDtmp = create_2Dimage_ID("_tmpinterpol", msizex, msizey);
                    for(m=0; m<data.image[ID].md[0].size[2]; m++)
                    {
                        for(ii=0; ii<msizex*msizey; ii++)
                            data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];

                        for(conviter=0; conviter<NBconviter; conviter++)
                        {
                            sigma = 0.5*NBconviter/(1.0+conviter);
                            gauss_filter("_tmpinterpol", "_tmpinterpolg", 1.0, 2);
                            IDtmpg = image_ID("_tmpinterpolg");
                            for(ii=0; ii<msizex*msizey; ii++)
                            {
                                if((data.image[IDmask].array.F[ii]>0.5)&&(data.image[IDslaved].array.F[ii]<0.5))
                                    data.image[IDtmp].array.F[ii] = data.image[ID].array.F[m*msizex*msizey+ii];
                                else
                                    data.image[IDtmp].array.F[ii] = data.image[IDtmpg].array.F[ii];
                            }
                            delete_image_ID("_tmpinterpolg");
                        }
                        for(ii=0; ii<msizex*msizey; ii++)
                            if(data.image[IDmask].array.F[ii]>0.5)
                                data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[ii];
                    */
        }
        delete_image_ID("_tmpinterpol");
        delete_image_ID("_tmpcoeff1");
        delete_image_ID("_tmpcoeff2");
    }



    return(ID);
}





// dmmask_perclow = 0.2
// dmmask_coefflow = 1.0
// dmmask_perchigh = 0.7
// dmmask_coeffhigh = 0.3

int_fast8_t AOloopControl_computeCalib_mkCalib_map_mask(long loop, const char *zrespm_name, const char *WFSmap_name, const char *DMmap_name, float dmmask_perclow, float dmmask_coefflow, float dmmask_perchigh, float dmmask_coeffhigh, float wfsmask_perclow, float wfsmask_coefflow, float wfsmask_perchigh, float wfsmask_coeffhigh)
{
    long IDWFSmap, IDDMmap;
    long IDWFSmask, IDDMmask;
    long IDzrm;
    long ii;
    float lim, rms;
    double tmpv;
    long sizexWFS, sizeyWFS, sizeWFS;
    long sizexDM, sizeyDM;
    long IDdm;
    char name[200];
    long NBpoke, poke;
    long IDDMmap1;
    float lim0;
    long IDtmp;


    IDzrm = image_ID(zrespm_name);
    sizexWFS = data.image[IDzrm].md[0].size[0];
    sizeyWFS = data.image[IDzrm].md[0].size[1];
    NBpoke = data.image[IDzrm].md[0].size[2];

    if(sprintf(name, "aol%ld_dmC", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDdm = read_sharedmem_image(name);
    sizexDM = data.image[IDdm].md[0].size[0];
    sizeyDM = data.image[IDdm].md[0].size[1];

    sizeWFS = sizexWFS*sizeyWFS;

    IDWFSmap = create_2Dimage_ID(WFSmap_name, sizexWFS, sizeyWFS);
    IDDMmap = create_2Dimage_ID(DMmap_name, sizexDM, sizeyDM);
    IDWFSmask = create_2Dimage_ID("wfsmask", sizexWFS, sizeyWFS);
    IDDMmask = create_2Dimage_ID("dmmask", sizexDM, sizeyDM);




    printf("Preparing DM map ... ");
    fflush(stdout);
    for(poke=0; poke<NBpoke; poke++)
    {
        rms = 0.0;
        for(ii=0; ii<sizeWFS; ii++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDDMmap].array.F[poke] = rms;
    }
    printf("done\n");
    fflush(stdout);



    printf("Preparing WFS map ... ");
    fflush(stdout);
    for(ii=0; ii<sizeWFS; ii++)
    {
        rms = 0.0;
        for(poke=0; poke<NBpoke; poke++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDWFSmap].array.F[ii] = rms;
    }
    printf("done\n");
    fflush(stdout);




    printf("Preparing DM mask ... ");
    fflush(stdout);

    // pre-filtering
    // gauss_filter(DMmap_name, "dmmapg", 5.0, 8);
    // IDDMmap1 = image_ID("dmmapg");

    // (map/map1)*pow(map,0.25)

    // DMmask: select pixels
    lim0 = dmmask_coefflow*img_percentile(DMmap_name, dmmask_perclow);
    IDtmp = create_2Dimage_ID("_tmpdmmap", sizexDM, sizeyDM);
    for(ii=0; ii<sizexDM*sizeyDM; ii++)
        data.image[IDtmp].array.F[ii] = data.image[IDDMmap].array.F[ii] - lim0;
    lim = dmmask_coeffhigh*img_percentile("_tmpdmmap", dmmask_perchigh);

    for(poke=0; poke<NBpoke; poke++)
    {
        if(data.image[IDtmp].array.F[poke]<lim)
            data.image[IDDMmask].array.F[poke] = 0.0;
        else
            data.image[IDDMmask].array.F[poke] = 1.0;
    }
    delete_image_ID("_tmpdmmap");
    printf("done\n");
    fflush(stdout);



    // WFSmask : select pixels
    printf("Preparing WFS mask ... ");
    fflush(stdout);

    lim0 = wfsmask_coefflow*img_percentile(WFSmap_name, wfsmask_perclow);
    IDtmp = create_2Dimage_ID("_tmpwfsmap", sizexWFS, sizeyWFS);
    for(ii=0; ii<sizexWFS*sizeyWFS; ii++)
        data.image[IDtmp].array.F[ii] = data.image[IDWFSmap].array.F[ii] - lim0;
    lim = wfsmask_coeffhigh*img_percentile("_tmpwfsmap", wfsmask_perchigh);

    for(ii=0; ii<sizeWFS; ii++)
    {
        if(data.image[IDWFSmap].array.F[ii]<lim)
            data.image[IDWFSmask].array.F[ii] = 0.0;
        else
            data.image[IDWFSmask].array.F[ii] = 1.0;
    }
    delete_image_ID("_tmpwfsmap");
    printf("done\n");
    fflush(stdout);

    return(0);
}





//
// if images "Hmat" AND "pixindexim" are provided, decode the image
// TEST: if "RMpokeC" exists, decode it as well
//
int_fast8_t AOloopControl_computeCalib_Process_zrespM(long loop, const char *IDzrespm0_name, const char *IDwfsref_name, const char *IDzrespm_name, const char *WFSmap_name, const char *DMmap_name)
{
    long NBpoke;
    long IDzrm;

    long sizexWFS, sizeyWFS, sizexDM, sizeyDM;
    long sizeWFS, sizeDM;
    long poke, ii;
    char name[200];

    double rms, tmpv;
    long IDDMmap, IDWFSmap, IDdm;



    // DECODE MAPS (IF REQUIRED)
    IDzrm = image_ID(IDzrespm0_name);
    if((image_ID("RMmat")!=-1) && (image_ID("pixindexim")!=-1))  // start decoding
    {
        save_fits(IDzrespm0_name, "!zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM(IDzrespm0_name, "RMmat", "pixindexim", IDzrespm_name);
        IDzrm = image_ID(IDzrespm_name);

        if(image_ID("RMpokeC")!=-1)
        {
            AOloopControl_computeCalib_Hadamard_decodeRM("RMpokeC", "RMmat", "pixindexim", "RMpokeC1");
            //save_fits("RMpokeC1", "!tmp/test_RMpokeC1.fits");
        }
    }
    else // NO DECODING
    {
        copy_image_ID(IDzrespm0_name, IDzrespm_name, 0);
    }




    // create sensitivity maps

    sizexWFS = data.image[IDzrm].md[0].size[0];
    sizeyWFS = data.image[IDzrm].md[0].size[1];
    NBpoke = data.image[IDzrm].md[0].size[2];

    if(sprintf(name, "aol%ld_dmC", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    IDdm = read_sharedmem_image(name);
    sizexDM = data.image[IDdm].md[0].size[0];
    sizeyDM = data.image[IDdm].md[0].size[1];

    sizeWFS = sizexWFS*sizeyWFS;

    IDWFSmap = create_2Dimage_ID(WFSmap_name, sizexWFS, sizeyWFS);
    IDDMmap = create_2Dimage_ID(DMmap_name, sizexDM, sizeyDM);


    printf("Preparing DM map ... ");
    fflush(stdout);
    for(poke=0; poke<NBpoke; poke++)
    {
        rms = 0.0;
        for(ii=0; ii<sizeWFS; ii++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDDMmap].array.F[poke] = rms;
    }
    printf("done\n");
    fflush(stdout);



    printf("Preparing WFS map ... ");
    fflush(stdout);
    for(ii=0; ii<sizeWFS; ii++)
    {
        rms = 0.0;
        for(poke=0; poke<NBpoke; poke++)
        {
            tmpv = data.image[IDzrm].array.F[poke*sizeWFS+ii];
            rms += tmpv*tmpv;
        }
        data.image[IDWFSmap].array.F[ii] = rms;
    }
    printf("done\n");
    fflush(stdout);



    /*
    IDWFSmask = image_ID("wfsmask");


    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
    	tot += data.image[IDWFSref].array.F[ii]*data.image[IDWFSmask].array.F[ii];

    totm = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
    	totm += data.image[IDWFSmask].array.F[ii];

    for(ii=0; ii<sizeWFS; ii++)
    	data.image[IDWFSref].array.F[ii] /= tot;

    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke=0;poke<NBpoke;poke++)
    {
    	tot = 0.0;
    	for(ii=0; ii<sizeWFS; ii++)
    		tot += data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];

    	for(ii=0; ii<sizeWFS; ii++)
    		data.image[IDzrm].array.F[poke*sizeWFS+ii] -= tot*data.image[IDWFSmask].array.F[ii]/totm;

    	tot1 = 0.0;
    	for(ii=0; ii<sizeWFS; ii++)
    		tot1 += data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];
    	fprintf(fp, "%6ld %06ld %20f %20f\n", poke, NBpoke, tot, tot1);
    }
    fclose(fp);

    */

}





// CANDIDATE FOR RETIREMENT
//
// median-averages multiple response matrices to create a better one
//
// if images "Hmat" AND "pixindexim" are provided, decode the image
// TEST: if "RMpokeC" exists, decode it as well
//
int_fast8_t AOloopControl_computeCalib_ProcessZrespM_medianfilt(long loop, const char *zrespm_name, const char *WFSref0_name, const char *WFSmap_name, const char *DMmap_name, double rmampl, int normalize)
{
    long NBmat; // number of matrices to average
    FILE *fp;
    int r;
    char name[200];
    char fname[200];
    char zrname[200];
    long kmat;
    long sizexWFS, sizeyWFS, sizeWFS;
    long *IDzresp_array;
    long ii;
    double fluxpos, fluxneg;
    float *pixvalarray;
    long k, kmin, kmax, kband;
    long IDzrm;
    float ave;
    long *IDWFSrefc_array;
    long IDWFSref;
    long IDWFSmap, IDDMmap;
    long IDWFSmask, IDDMmask;
    float lim, rms;
    double tmpv;
    long NBmatlim = 3;
    long ID1;
    long NBpoke, poke;
    double tot, totm;


    if(sprintf(fname, "./zresptmp/%s_nbiter.txt", zrespm_name) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    if((fp = fopen(fname, "r"))==NULL)
    {
        printf("ERROR: cannot open file \"%s\"\n", fname);
        exit(0);
    }
    else
    {
        if(fscanf(fp, "%50ld", &NBmat) != 1)
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");

        fclose(fp);
    }


    if(NBmat<NBmatlim)
    {
        printf("ERROR: insufficient number of input matrixes:\n");
        printf(" NBmat = %ld, should be at least %ld\n", (long) NBmat, (long) NBmatlim);
        exit(0);
    }
    else
        printf("Processing %ld matrices\n", NBmat);

    if(sprintf(name, "aol%ld_dmC", loop) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");


    IDzresp_array = (long*) malloc(sizeof(long)*NBmat);
    IDWFSrefc_array = (long*) malloc(sizeof(long)*NBmat);

    // STEP 1: build individually cleaned RM
    for(kmat=0; kmat<NBmat; kmat++)
    {
        if(sprintf(fname, "./zresptmp/%s_pos_%03ld.fits", zrespm_name, kmat) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDzrespfp = load_fits(fname, "zrespfp", 2);

        if(sprintf(fname, "./zresptmp/%s_neg_%03ld.fits", zrespm_name, kmat) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDzrespfm = load_fits(fname, "zrespfm", 2);

        sizexWFS = data.image[IDzrespfp].md[0].size[0];
        sizeyWFS = data.image[IDzrespfp].md[0].size[1];
        NBpoke = data.image[IDzrespfp].md[0].size[2];
        sizeWFS = sizexWFS*sizeyWFS;

        if(sprintf(name, "wfsrefc%03ld", kmat) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        IDWFSrefc_array[kmat] = create_3Dimage_ID(name, sizexWFS, sizeyWFS, NBpoke);

        if(sprintf(zrname, "zrespm%03ld", kmat) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        IDzresp_array[kmat] = create_3Dimage_ID(zrname, sizexWFS, sizeyWFS, NBpoke);



# ifdef _OPENMP
        #pragma omp parallel for private(fluxpos,fluxneg,ii)
# endif
        for(poke=0; poke<NBpoke; poke++)
        {
            fluxpos = 0.0;
            fluxneg = 0.0;
            for(ii=0; ii<sizeWFS; ii++)
            {
                if(isnan(data.image[IDzrespfp].array.F[poke*sizeWFS+ii])!=0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n", IDzrespfp, poke*sizeWFS+ii);
                    data.image[IDzrespfp].array.F[poke*sizeWFS+ii] = 0.0;
                }
                fluxpos += data.image[IDzrespfp].array.F[poke*sizeWFS+ii];
            }

            for(ii=0; ii<sizeWFS; ii++)
            {
                if(isnan(data.image[IDzrespfm].array.F[poke*sizeWFS+ii])!=0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n", IDzrespfm, poke*sizeWFS+ii);
                    data.image[IDzrespfm].array.F[poke*sizeWFS+ii] = 0.0;
                }
                fluxneg += data.image[IDzrespfm].array.F[poke*sizeWFS+ii];
            }

            for(ii=0; ii<sizeWFS; ii++)
            {
                if(normalize==1)
                {
                    data.image[IDzrespfp].array.F[poke*sizeWFS+ii] /= fluxpos;
                    data.image[IDzrespfm].array.F[poke*sizeWFS+ii] /= fluxneg;
                }
                data.image[IDzresp_array[kmat]].array.F[poke*sizeWFS+ii] = 0.5*(data.image[IDzrespfp].array.F[poke*sizeWFS+ii]-data.image[IDzrespfm].array.F[poke*sizeWFS+ii]);
                data.image[IDWFSrefc_array[kmat]].array.F[poke*sizeWFS+ii] = 0.5*(data.image[IDzrespfp].array.F[poke*sizeWFS+ii]+data.image[IDzrespfm].array.F[poke*sizeWFS+ii]);

                if(isnan(data.image[IDzresp_array[kmat]].array.F[poke*sizeWFS+ii])!=0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n", IDzresp_array[kmat], poke*sizeWFS+ii);
                    data.image[IDzresp_array[kmat]].array.F[poke*sizeWFS+ii] = 0.0;
                }
                if(isnan(data.image[IDWFSrefc_array[kmat]].array.F[poke*sizeWFS+ii])!=0)
                {
                    printf("%ld element %ld is NAN -> replacing by 0\n", IDWFSrefc_array[kmat], poke*sizeWFS+ii);
                    data.image[IDWFSrefc_array[kmat]].array.F[poke*sizeWFS+ii] = 0.0;
                }
            }
        }

        delete_image_ID("zrespfp");
        delete_image_ID("zrespfm");
    }

    // STEP 2: average / median each pixel
    IDzrm = create_3Dimage_ID(zrespm_name, sizexWFS, sizeyWFS, NBpoke);
    IDWFSref = create_2Dimage_ID(WFSref0_name, sizexWFS, sizeyWFS);




    kband = (long) (0.2*NBmat);

    kmin = kband;
    kmax = NBmat-kband;

# ifdef _OPENMP
    #pragma omp parallel for private(ii,kmat,ave,k,pixvalarray)
# endif
    for(poke=0; poke<NBpoke; poke++)
    {
        printf("\r act %ld / %ld        ", poke, NBpoke);
        fflush(stdout);

        if((pixvalarray = (float*) malloc(sizeof(float)*NBmat))==NULL)
        {
            printf("ERROR: cannot allocate pixvalarray, size = %ld\n", (long) NBmat);
            exit(0);
        }

        for(ii=0; ii<sizeWFS; ii++)
        {
            for(kmat=0; kmat<NBmat; kmat++)
                pixvalarray[kmat] = data.image[IDzresp_array[kmat]].array.F[poke*sizeWFS+ii] ;
            quick_sort_float(pixvalarray, kmat);
            ave = 0.0;
            for(k=kmin; k<kmax; k++)
                ave += pixvalarray[k];
            ave /= (kmax-kmin);
            data.image[IDzrm].array.F[poke*sizeWFS+ii] = ave/rmampl;
        }
        free(pixvalarray);
    }

    printf("\n");



    kband = (long) (0.2*NBmat*NBpoke);
    kmin = kband;
    kmax = NBmat*NBpoke-kband;


# ifdef _OPENMP
    #pragma omp parallel for private(poke,kmat,pixvalarray,ave,k)
# endif
    for(ii=0; ii<sizeWFS; ii++)
    {
        printf("\r wfs pix %ld / %ld        ", ii, sizeWFS);
        fflush(stdout);
        if((pixvalarray = (float*) malloc(sizeof(float)*NBmat*NBpoke))==NULL)
        {
            printf("ERROR: cannot allocate pixvalarray, size = %ld x %ld\n", (long) NBmat, (long) NBpoke);
            exit(0);
        }

        for(poke=0; poke<NBpoke; poke++)
            for(kmat=0; kmat<NBmat; kmat++)
                pixvalarray[kmat*NBpoke+poke] = data.image[IDWFSrefc_array[kmat]].array.F[poke*sizeWFS+ii] ;


        quick_sort_float(pixvalarray, NBpoke*NBmat);

        ave = 0.0;
        for(k=kmin; k<kmax; k++)
            ave += pixvalarray[k];
        ave /= (kmax-kmin);
        data.image[IDWFSref].array.F[ii] = ave;

        //printf("free pixvalarray : %ld x %ld\n", NBmat, NBpoke);
        //fflush(stdout);
        free(pixvalarray);
        //printf("done\n");
        //fflush(stdout);
    }
    free(IDzresp_array);
    free(IDWFSrefc_array);





    // DECODE MAPS (IF REQUIRED)

    if((image_ID("Hmat")!=-1) && (image_ID("pixindexim")!=-1))
    {
        chname_image_ID(zrespm_name, "tmprm");
        save_fits("tmprm", "!zrespm_Hadamard.fits");

        AOloopControl_computeCalib_Hadamard_decodeRM("tmprm", "Hmat", "pixindexim", zrespm_name);
        delete_image_ID("tmprm");

        IDzrm = image_ID(zrespm_name);

        if(image_ID("RMpokeC") != -1)
        {
            AOloopControl_computeCalib_Hadamard_decodeRM("RMpokeC", "Hmat", "pixindexim", "RMpokeC1");
            save_fits("RMpokeC1", "!test_RMpokeC1.fits");
        }
    }

    NBpoke = data.image[IDzrm].md[0].size[2];


    AOloopControl_computeCalib_mkCalib_map_mask(loop, zrespm_name, WFSmap_name, DMmap_name, 0.2, 1.0, 0.7, 0.3, 0.05, 1.0, 0.65, 0.3);

    //	list_image_ID();
    //printf("========== STEP 000 ============\n");
    //	fflush(stdout);


    IDWFSmask = image_ID("wfsmask");
    //	printf("ID   %ld %ld\n", IDWFSmask, IDWFSref);

    // normalize wfsref with wfsmask
    tot = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
        tot += data.image[IDWFSref].array.F[ii]*data.image[IDWFSmask].array.F[ii];

    totm = 0.0;
    for(ii=0; ii<sizeWFS; ii++)
        totm += data.image[IDWFSmask].array.F[ii];

    for(ii=0; ii<sizeWFS; ii++)
        data.image[IDWFSref].array.F[ii] /= tot;



    // make zrespm flux-neutral over wfsmask
    fp = fopen("zrespmat_flux.log", "w");
    for(poke=0; poke<NBpoke; poke++)
    {
        tot = 0.0;
        for(ii=0; ii<sizeWFS; ii++)
            tot += data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];

        for(ii=0; ii<sizeWFS; ii++)
            data.image[IDzrm].array.F[poke*sizeWFS+ii] -= tot*data.image[IDWFSmask].array.F[ii]/totm;

        double tot1 = 0.0;
        for(ii=0; ii<sizeWFS; ii++)
            tot1 += data.image[IDzrm].array.F[poke*sizeWFS+ii]*data.image[IDWFSmask].array.F[ii];
        fprintf(fp, "%6ld %06ld %20f %20f\n", poke, NBpoke, tot, tot1);
    }
    fclose(fp);


    return(0);
}





// make control matrix
//
long AOloopControl_computeCalib_mkCM(const char *respm_name, const char *cm_name, float SVDlim)
{


    // COMPUTE OVERALL CONTROL MATRIX

    printf("COMPUTE OVERALL CONTROL MATRIX\n");
#ifdef HAVE_MAGMA
    CUDACOMP_magma_compute_SVDpseudoInverse(respm_name, cm_name, SVDlim, 100000, "VTmat", 0);
#else
    linopt_compute_SVDpseudoInverse(respm_name, cm_name, SVDlim, 10000, "VTmat");
#endif

    //save_fits("VTmat", "!./mkmodestmp/VTmat.fits");
    delete_image_ID("VTmat");


    return(image_ID(cm_name));
}




//
// make slave actuators from maskRM
//
long AOloopControl_computeCalib_mkSlavedAct(const char *IDmaskRM_name, float pixrad, const char *IDout_name)
{
    long IDout;
    long IDmaskRM;
    long ii, jj;
    long ii1, jj1;
    long xsize, ysize;
    long pixradl;
    long ii1min, ii1max, jj1min, jj1max;
    float dx, dy, r;


    IDmaskRM = image_ID(IDmaskRM_name);
    xsize = data.image[IDmaskRM].md[0].size[0];
    ysize = data.image[IDmaskRM].md[0].size[1];

    pixradl = (long) pixrad + 1;

    IDout = create_2Dimage_ID(IDout_name, xsize, ysize);
    for(ii=0; ii<xsize*ysize; ii++)
        data.image[IDout].array.F[ii] = xsize+ysize;

    for(ii=0; ii<xsize; ii++)
        for(jj=0; jj<ysize; jj++)
        {
            if(data.image[IDmaskRM].array.F[jj*xsize+ii] < 0.5)
            {
                ii1min = ii-pixradl;
                if(ii1min<0)
                    ii1min=0;
                ii1max = ii+pixradl;
                if(ii1max>(xsize-1))
                    ii1max = xsize-1;

                jj1min = jj-pixradl;
                if(jj1min<0)
                    jj1min=0;
                jj1max = jj+pixradl;
                if(jj1max>(ysize-1))
                    jj1max = ysize-1;

                for(ii1=ii1min; ii1<ii1max+1; ii1++)
                    for(jj1=jj1min; jj1<jj1max+1; jj1++)
                        if(data.image[IDmaskRM].array.F[jj1*xsize+ii1]>0.5)
                        {
                            dx = 1.0*(ii-ii1);
                            dy = 1.0*(jj-jj1);
                            r = sqrt(dx*dx+dy*dy);
                            if(r<pixrad)
                                if(r < data.image[IDout].array.F[jj*xsize+ii])
                                    data.image[IDout].array.F[jj*xsize+ii] = r;
                        }
            }
        }

    for(ii=0; ii<xsize; ii++)
        for(jj=0; jj<ysize; jj++)
            if(data.image[IDout].array.F[jj*xsize+ii] > (xsize+ysize)/2 )
                data.image[IDout].array.F[jj*xsize+ii] = 0.0;


    return(IDout);
}



static long AOloopControl_computeCalib_DMedgeDetect(const char *IDmaskRM_name, const char *IDout_name)
{
    long IDout;
    long IDmaskRM;
    long ii, jj;
    float val1;
    long xsize, ysize;


    IDmaskRM = image_ID(IDmaskRM_name);
    xsize = data.image[IDmaskRM].md[0].size[0];
    ysize = data.image[IDmaskRM].md[0].size[1];

    IDout = create_2Dimage_ID(IDout_name, xsize, ysize);

    for(ii=1; ii<xsize-1; ii++)
        for(jj=1; jj<ysize-1; jj++)
        {
            val1 = 0.0;
            if(data.image[IDmaskRM].array.F[jj*xsize+ii] > 0.5)
            {
                if(data.image[IDmaskRM].array.F[jj*xsize+ii+1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[jj*xsize+ii-1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj+1)*xsize+ii]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-1)*xsize+ii]<0.5)
                    val1 += 1.0;

                if(data.image[IDmaskRM].array.F[(jj+1)*xsize+ii+1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj+1)*xsize+ii-1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-1)*xsize+ii+1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-1)*xsize+ii-1]<0.5)
                    val1 += 1.0;

                if(data.image[IDmaskRM].array.F[jj*xsize+ii+2]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[jj*xsize+ii-2]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj+2)*xsize+ii]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-2)*xsize+ii]<0.5)
                    val1 += 1.0;



                if(data.image[IDmaskRM].array.F[(jj+1)*xsize+ii+2]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj+1)*xsize+ii-2]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-1)*xsize+ii+2]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-1)*xsize+ii-2]<0.5)
                    val1 += 1.0;

                if(data.image[IDmaskRM].array.F[(jj+2)*xsize+ii-1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-2)*xsize+ii-1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj+2)*xsize+ii+1]<0.5)
                    val1 += 1.0;
                if(data.image[IDmaskRM].array.F[(jj-2)*xsize+ii+1]<0.5)
                    val1 += 1.0;

            }
            if(val1>4.9)
                val1 = 1.0;
            else
                val1 = 0.0;
            data.image[IDout].array.F[jj*xsize+ii] = val1;
        }

    return(IDout);
}



static long AOloopControl_computeCalib_DMextrapolateModes(const char *IDin_name, const char *IDmask_name, const char *IDcpa_name, const char *IDout_name)
{
    long IDin, IDmask, IDcpa, IDout;
    long xsize, ysize, zsize, xysize;
    long IDpixdist;
    long ii, jj, ii1, jj1, dii, djj, dii2, djj2;
    float r, dist;
    float coeff;
    long index;
    long kk;


    IDin = image_ID(IDin_name);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
    if(data.image[IDin].md[0].naxis == 3)
    {
        zsize = data.image[IDin].md[0].size[2];
        IDout = create_3Dimage_ID(IDout_name, xsize, ysize, zsize);
    }
    else
    {
        zsize = 1;
        IDout = create_2Dimage_ID(IDout_name, xsize, ysize);
    }
    xysize = xsize*ysize;

    IDmask = image_ID(IDmask_name);
    IDcpa = image_ID(IDcpa_name);


    // measure pixel distance to mask
    IDpixdist = create_2Dimage_ID("pixmaskdist", xsize, ysize);
    for(ii=0; ii<xsize; ii++)
        for(jj=0; jj<ysize; jj++)
        {
            dist = 1.0*xsize+1.0*ysize;
            for(ii1=0; ii1<xsize; ii1++)
                for(jj1=0; jj1<ysize; jj1++)
                {
                    if(data.image[IDmask].array.F[jj1*xsize+ii1] > 0.5)
                    {
                        dii = ii1-ii;
                        djj = jj1-jj;
                        dii2 = dii*dii;
                        djj2 = djj*djj;
                        r = sqrt(dii2+djj2);
                        if(r<dist)
                            dist = r;
                    }
                }
            data.image[IDpixdist].array.F[jj*xsize+ii] = dist;
        }
    //save_fits("pixmaskdist", "!_tmp_pixmaskdist.fits");
    //save_fits(IDcpa_name, "!_tmp_cpa.fits");
    for(kk=0; kk<zsize; kk++)
    {
        for(ii=0; ii<xsize; ii++)
            for(jj=0; jj<ysize; jj++)
            {
                index = jj*xsize+ii;
                coeff =  data.image[IDpixdist].array.F[index] / ((1.0*xsize/(data.image[IDcpa].array.F[kk]+0.1))*0.8);

                coeff = (exp(-coeff*coeff)-exp(-1.0))  / (1.0 - exp(-1.0));
                if(coeff<0.0)
                    coeff = 0.0;
                data.image[IDout].array.F[kk*xysize+index] = coeff*data.image[IDin].array.F[kk*xysize+index]*coeff;
            }
    }
    delete_image_ID("pixmaskdist");


    return(IDout);
}



long AOloopControl_computeCalib_DMslaveExt(const char *IDin_name, const char *IDmask_name, const char *IDsl_name, const char *IDout_name, float r0)
{
    long IDin, IDmask, IDsl, IDout;
    long ii, jj, kk, ii1, jj1;
    long xsize, ysize, zsize, xysize;
    long index;
    float rfactor = 2.0;
    float val1, val1cnt;
    float pixrad;
    long pixradl;
    long ii1min, ii1max, jj1min, jj1max;
    float dx, dy, r, r1;
    float coeff;
    float valr;


    IDin = image_ID(IDin_name);
    xsize = data.image[IDin].md[0].size[0];
    ysize = data.image[IDin].md[0].size[1];
    if(data.image[IDin].md[0].naxis == 3)
    {
        zsize = data.image[IDin].md[0].size[2];
        IDout = create_3Dimage_ID(IDout_name, xsize, ysize, zsize);
    }
    else
    {
        zsize = 1;
        IDout = create_2Dimage_ID(IDout_name, xsize, ysize);
    }
    xysize = xsize*ysize;

    IDmask = image_ID(IDmask_name);
    IDsl = image_ID(IDsl_name);




    for(ii=0; ii<xsize; ii++)
        for(jj=0; jj<ysize; jj++)
        {
            index = jj*xsize+ii;
            if(data.image[IDmask].array.F[index]>0.5)
            {
                for(kk=0; kk<zsize; kk++)
                    data.image[IDout].array.F[kk*xysize+index] = data.image[IDin].array.F[kk*xysize+index];
            }
            else if (data.image[IDsl].array.F[index]>0.5)
            {
                for(kk=0; kk<zsize; kk++)
                {
                    val1 = 0.0;
                    val1cnt = 0.0;
                    pixrad = (rfactor*data.image[IDsl].array.F[index]+1.0);
                    pixradl = (long) pixrad + 1;

                    ii1min = ii-pixradl;
                    if(ii1min<0)
                        ii1min=0;
                    ii1max = ii+pixradl;
                    if(ii1max>(xsize-1))
                        ii1max = xsize-1;

                    jj1min = jj-pixradl;
                    if(jj1min<0)
                        jj1min=0;
                    jj1max = jj+pixradl;
                    if(jj1max>(ysize-1))
                        jj1max = ysize-1;


                    valr = 0.0;
                    for(ii1=ii1min; ii1<ii1max+1; ii1++)
                        for(jj1=jj1min; jj1<jj1max+1; jj1++)
                        {
                            dx = 1.0*(ii-ii1);
                            dy = 1.0*(jj-jj1);
                            r = sqrt(dx*dx+dy*dy);
                            if((r<pixrad)&&(data.image[IDmask].array.F[jj1*xsize+ii1]>0.5))
                            {
                                r1 = r/pixrad;
                                coeff = exp(-10.0*r1*r1);
                                valr += r*coeff;
                                val1 += data.image[IDin].array.F[kk*xysize+jj1*xsize+ii1]*coeff;
                                val1cnt += coeff;
                            }
                        }
                    valr /= val1cnt;
                    if(val1cnt>0.0001)
                        data.image[IDout].array.F[kk*xysize+index] = (val1/val1cnt)*exp(-(valr/r0)*(valr/r0));
                }
            }
            else
                for(kk=0; kk<zsize; kk++)
                    data.image[IDout].array.F[kk*xysize+index] = 0.0;
        }


    return(IDout);
}




/*** \brief creates AO control modes
 *
 *
 * creates image "modesfreqcpa" which contains CPA value for each mode
 *
 *
 * if Mmask exists, measure xc, yc from it, otherwise use values given to function
 *
 * INPUT (optional): "dmmaskRM" DM actuators directly controlled
 * INPUT (optional): "dmslaved" force these actuators to be slaved to neighbors
 * OUTPUT :          "dmmask" is the union of "dmmaskRM" and "dmslaved"
 *
 * MaskMode = 0  : tapered masking
 * MaskMode = 1  : STRICT masking
 *
 * if BlockNB < 0 : do all blocks
 * if BlockNB >= 0 : only update single block (untested)
 *
 * SVDlim = 0.05 works well
 *
 * OPTIONAL : if file zrespM exists, WFS modes will be computed
 *
 */

long AOloopControl_computeCalib_mkModes(const char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yc, double r0, double r1, int MaskMode, int BlockNB, float SVDlim)
{
    FILE *fp;
    long ID = -1;
    long ii, jj;

    long IDmaskRM; // DM mask

    double totm;

    double x, y, r, xc1, yc1;
    double rms;

    long IDz;

    long zindex[10];
    double zcpa[10];  /// CPA for each Zernike (somewhat arbitrary... used to sort modes in CPA)

    long mblock, m;
    long NBmblock;

    long MBLOCK_NBmode[MAX_MBLOCK]; // number of blocks
    long MBLOCK_ID[MAX_MBLOCK];
//    long MBLOCK_IDwfs[MAX_MBLOCK];
    float MBLOCK_CPA[MAX_MBLOCK];


    char *ptr0;
    char *ptr1;

    char imname[200];
    char imname1[200];


    char fname[200];
    char fname1[200];


    float value, value0, value1, value1cnt, valuen;
    long msizexy;
    long m0, mblock0;

    long iter;

	long IDmask;

    long IDzrespM;
    long wfsxsize, wfsysize, wfssize;
    long wfselem, act;

    long IDout, ID1, ID2, ID2b;
    long zsize1, zsize2;
    long z1, z2;
    long xysize1, xysize2;



    float SVDlim00;// DM filtering step 0
    float SVDlim01; // DM filtering step 1


    float rmslim1 = 0.1;
    long IDm;


    int *mok;
    long NBmm = 2000; // max number of modes per block
    long cnt;


    int reuse;
    long IDSVDmodein, IDSVDmode1, IDSVDcoeff, IDSVDmask;
    long m1;
    long IDnewmodeC;

    long IDtmp, IDtmpg;
    long conviter;
    float sigma;

    int MODAL; // 1 if "pixels" of DM are already modes

    long IDRMMmodes = -1;
    long IDRMMresp  = -1;
    long ID_imfit = -1;
    long IDRMM_coeff = -1;
    long IDcoeffmat = -1;

    long linfitsize;
    int linfitreuse;
    double res, res1, v0;


    double resn, vn;
    double LOcoeff;
    FILE *fpLOcoeff;
    long IDwfstmp;

    long pixcnt;
    float vxp, vxm, vyp, vym, cxp, cxm, cyp, cym, ctot;
    long IDtmp1, IDtmp2;


    int COMPUTE_DM_MODES = 1; // compute DM modes (initial step) fmode2b_xxx and fmodes2ball


    long ii1, jj1;
    float dx, dy, dist, dist0, val1cnt;
    long IDprox, IDprox1;
    float gain;

    FILE *fpcoeff;
    char fnameSVDcoeff[400];


    // extra block
    long extrablockIndex;



    // SET LIMITS
    SVDlim00 = SVDlim; // DM filtering step 0
    SVDlim01 = SVDlim; // DM filtering step 1




    MODAL = 0;
    if(msizey==1)
        MODAL = 1;


    zindex[0] = 1; // tip
    zcpa[0] = 0.0;

    zindex[1] = 2; // tilt
    zcpa[1] = 0.0;

    zindex[2] = 4; // focus
    zcpa[2] = 0.25;

    zindex[3] = 3; // astig
    zcpa[3] = 0.4;

    zindex[4] = 5; // astig
    zcpa[4] = 0.4;

    zindex[5] = 7; // coma
    zcpa[5] = 0.6;

    zindex[6] = 8; // coma
    zcpa[6] = 0.6;

    zindex[7] = 6; // trefoil
    zcpa[7] = 1.0;

    zindex[8] = 9; // trefoil
    zcpa[8] = 1.0;

    zindex[9] = 12;
    zcpa[9] = 1.5;



    if(system("mkdir -p mkmodestmp") < 1)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    msizexy = msizex*msizey;

    /// STEP 1: CREATE STARTING POINT : ZERNIKES + FOURIER MODES

    /// if Mmask exists, use it, otherwise create it
    if(MODAL == 0)
    {
        IDmaskRM = image_ID("dmmaskRM");
        if(IDmaskRM==-1)
        {
            double val0, val1;
            double a0=0.88;
            double b0=40.0;
            double a1=1.2;
            double b1=12.0;

            IDmaskRM = create_2Dimage_ID("dmmaskRM", msizex, msizey);
            for(ii=0; ii<msizex; ii++)
                for(jj=0; jj<msizey; jj++)
                {
                    x = 1.0*ii-xc;
                    y = 1.0*jj-yc;
                    r = sqrt(x*x+y*y)/r1;
                    val1 = 1.0-exp(-pow(a1*r,b1));
                    r = sqrt(x*x+y*y)/r0;
                    val0 = exp(-pow(a0*r,b0));
                    data.image[IDmaskRM].array.F[jj*msizex+ii] = val0*val1;
                }
            save_fits("dmmaskRM", "!dmmaskRM.fits");
            xc1 = xc;
            yc1 = yc;
        }
        else /// extract xc and yc from mask
        {
            xc1 = 0.0;
            yc1 = 0.0;
            totm = 0.0;
            for(ii=0; ii<msizex; ii++)
                for(jj=0; jj<msizey; jj++)
                {
                    xc1 += 1.0*ii*data.image[IDmaskRM].array.F[jj*msizex+ii];
                    yc1 += 1.0*jj*data.image[IDmaskRM].array.F[jj*msizex+ii];
                    totm += data.image[IDmaskRM].array.F[jj*msizex+ii];
                }
            xc1 /= totm;
            yc1 /= totm;
        }

        totm = arith_image_total("dmmaskRM");
        if((msizex != data.image[IDmaskRM].md[0].size[0])||(msizey != data.image[IDmaskRM].md[0].size[1]))
        {
            printf("ERROR: file dmmaskRM size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmaskRM].md[0].size[0], (long) data.image[IDmaskRM].md[0].size[1], (long) msizex, (long) msizey);
            exit(0);
        }
    }
    else
        totm = 1.0;




    COMPUTE_DM_MODES = 0;
    ID2b = image_ID("fmodes2ball");

    if(ID2b == -1)
        COMPUTE_DM_MODES = 1;



    if(COMPUTE_DM_MODES==1) // DM modes fmodes2b
    {
        long ID0 = -1;
        long NBZ = 0;
        long IDmfcpa;
        float CPAblocklim[MAX_MBLOCK]; // defines CPA limits for blocks
        long IDslaved;



        if(MODAL==0)
        {
            long IDmaskRMin;
            long IDmaskRMedge;
            

            // AOloopControl_mkloDMmodes(ID_name, msizex, msizey, CPAmax, deltaCPA, xc, yc, r0, r1, MaskMode);
            //NBZ = 5; /// 3: tip, tilt, focus
            NBZ = 0;
            for(m=0; m<10; m++)
            {
                if(zcpa[m]<CPAmax)
                    NBZ++;
            }


            // here we create simple Fourier modes
            linopt_imtools_makeCPAmodes("CPAmodes", msizex, CPAmax, deltaCPA, 0.5*msizex, 1.2, 0);
            ID0 = image_ID("CPAmodes");

            long IDfreq = image_ID("cpamodesfreq");



            printf("  %ld %ld %ld\n", msizex, msizey, (long) (data.image[ID0].md[0].size[2]-1) );
            ID = create_3Dimage_ID(ID_name, msizex, msizey, data.image[ID0].md[0].size[2]-1+NBZ);

            IDmfcpa = create_2Dimage_ID("modesfreqcpa", data.image[ID0].md[0].size[2]-1+NBZ, 1);


            zernike_init();

            double PA;
            uint_fast32_t k;
            for(k=0; k<NBZ; k++)
            {
                data.image[IDmfcpa].array.F[k] = zcpa[k];
                for(ii=0; ii<msizex; ii++)
                    for(jj=0; jj<msizey; jj++)
                    {
                        x = 1.0*ii-xc1;
                        y = 1.0*jj-yc1;
                        r = sqrt(x*x+y*y)/r1;
                        PA = atan2(y,x);
                        data.image[ID].array.F[k*msizex*msizey+jj*msizex+ii] = Zernike_value(zindex[k], r, PA);
                    }
            }

            for(k=0; k<data.image[ID0].md[0].size[2]-1; k++)
            {
                data.image[IDmfcpa].array.F[k+NBZ] = data.image[IDfreq].array.F[k+1];
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[(k+NBZ)*msizex*msizey+ii] = data.image[ID0].array.F[(k+1)*msizex*msizey+ii];
            }


            fp = fopen("rmscomp.dat", "w");



            for(k=0; k<data.image[ID0].md[0].size[2]-1+NBZ; k++)
            {
				
                // set RMS = 1 over mask
                rms = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    //data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                    rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];
                }
                rms = sqrt(rms/totm);
                printf("Mode %ld   RMS = %lf\n", k, rms);

                fprintf(fp, "%5ld  %g ", k, rms);

                /// Remove excluded modes if they exist
                /*          IDeModes = image_ID("emodes");
                          if(IDeModes!=-1)
                          {
                              IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

                              for(ii=0; ii<msizex*msizey; ii++)
                                  data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
                              linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmaskRM", 1.0e-3, "lcoeff", 0);
                              linopt_imtools_image_construct("emodes", "lcoeff", "em00");
                              delete_image_ID("lcoeff");
                              IDem = image_ID("em00");

                //					coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));

                			if(k>kelim)
                				coeff = 1.0;
                			else
                				coeff = 0.0;


                              for(ii=0; ii<msizex*msizey; ii++)
                                  data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

                              delete_image_ID("em00");
                              delete_image_ID("tmpmode");
                          }*/


                // Compute total of image over mask -> totvm
                double totvm = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                    totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

                // compute DC offset in mode
                double offset = totvm/totm;

                // remove DM offset
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] -= offset;

                offset = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                    offset += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

                // set RMS = 1 over mask
                rms = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                    rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];
                }
                rms = sqrt(rms/totm);
                printf("\r Mode %ld   RMS = %lf   ", k, rms);
                fprintf(fp, " %g\n", rms);

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
            }
            fclose(fp);
			printf("\n");


            if(MaskMode==1)
            {
                long kernsize = 5;
                if(2*kernsize>msizex)
                    kernsize = msizex/2;
                long citer;
                long NBciter = 200;
                for(citer=0; citer<NBciter; citer++)
                {
                    printf("Convolution [%3ld/%3ld]\n", citer, NBciter);
                    gauss_filter(ID_name, "modeg", 4.0*pow(1.0*(NBciter-citer)/NBciter,0.5), kernsize);
                    long IDg = image_ID("modeg");
                    uint_fast32_t  k;
                    for(k=0; k<data.image[ID].md[0].size[2]; k++)
                    {
                        for(ii=0; ii<msizex*msizey; ii++)
                            if(data.image[IDmaskRM].array.F[ii]<0.98)
                                data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDg].array.F[k*msizex*msizey+ii];
                    }
                    delete_image_ID("modeg");
                }
            }







            // MAKE MASKS FOR EDGE EXTRAPOLATION


            IDslaved = image_ID("dmslaved");
            // load or create DM mask : union of dmslaved and dmmaskRM
            //IDmask = load_fits("dmmask.fits", "dmmask", 1);
            printf("Create DM mask\n");
            fflush(stdout);


            //IDmask = -1;
            //if(IDmask == -1)
            //{
            IDmask = create_2Dimage_ID("dmmask", msizex, msizey);
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[IDmask].array.F[ii] = 1.0 - (1.0-data.image[IDmaskRM].array.F[ii])*(1.0-data.image[IDslaved].array.F[ii]);
            //    data.image[IDmask].array.F[ii] = 1.0 - (1.0-data.image[IDslaved].array.F[ii]);
                if(data.image[IDmask].array.F[ii]>1.0)
                    data.image[IDmask].array.F[ii] = 1.0;
            }
            save_fits("dmmask", "!dmmask.fits");
            //}

            // EDGE PIXELS IN IDmaskRM
            IDmaskRMedge = AOloopControl_computeCalib_DMedgeDetect(data.image[IDmaskRM].md[0].name, "dmmaskRMedge");
            save_fits("dmmaskRMedge", "!dmmaskRMedge.fits");

            // IDmaskRM pixels excluding edge
            IDmaskRMin = create_2Dimage_ID("dmmaskRMin", msizex, msizey);
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[IDmaskRMin].array.F[ii] = data.image[IDmaskRM].array.F[ii] * (1.0 - data.image[IDmaskRMedge].array.F[ii]);
            save_fits("dmmaskRMin", "!dmmaskRMin.fits");


            save_fits(ID_name, "!./mkmodestmp/_test_fmodes0all00.fits");


            IDtmp = AOloopControl_computeCalib_DMextrapolateModes(ID_name, "dmmaskRMin", "modesfreqcpa", "fmodes0test");
            save_fits("fmodes0test", "!fmodes0test.fits");


            for(m=0; m<data.image[ID].md[0].size[2]; m++)
            {
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[m*msizex*msizey+ii] = data.image[IDtmp].array.F[m*msizex*msizey+ii] * data.image[IDmask].array.F[ii];
            }
        }
        else
        {
            ID = create_3Dimage_ID(ID_name, msizex, msizey, msizex);
            IDmfcpa = create_2Dimage_ID("modesfreqcpa", msizex, 1);

            for(m=0; m<data.image[ID].md[0].size[2]; m++)
            {
                if(m<10)
                    data.image[IDmfcpa].array.F[m] = zcpa[m];
                else
                    data.image[IDmfcpa].array.F[m] = zcpa[9];

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[m*msizex*msizey+ii] = 0.0;
                data.image[ID].array.F[m*msizex*msizey+m] = 1.0;
            }
        }



        printf("SAVING MODES : %s...\n", ID_name);
        save_fits(ID_name, "!./mkmodestmp/fmodes0all_00.fits");






        // remove modes
        uint_fast32_t k;
        for(k=0; k<data.image[ID0].md[0].size[2]-1 + NBZ; k++)
        {
            /// Remove excluded modes if they exist
            long IDeModes = image_ID("emodes");
            if(IDeModes!=-1)
            {
                long kelim = 5;
                long IDtm = create_2Dimage_ID("tmpmode", msizex, msizey);

                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[IDtm].array.F[ii] = data.image[ID].array.F[k*msizex*msizey+ii];
                linopt_imtools_image_fitModes("tmpmode", "emodes", "dmmask", 1.0e-3, "lcoeff", 0);
                linopt_imtools_image_construct("emodes", "lcoeff", "em00");
                delete_image_ID("lcoeff");
                long IDem = image_ID("em00");

                double coeff = 1.0-exp(-pow(1.0*k/kelim,6.0));

                if(k>kelim)
                    coeff = 1.0;
                else
                    coeff = 0.0;


                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[ID].array.F[k*msizex*msizey+ii] = data.image[IDtm].array.F[ii] - coeff*data.image[IDem].array.F[ii];

                delete_image_ID("em00");
                delete_image_ID("tmpmode");
            }

            // Compute total of image over mask -> totvm
            double totvm = 0.0;
            totm = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
            {
                totvm += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
                totm += data.image[IDmask].array.F[ii];
            }

            // compute DC offset in mode
            double offset = totvm/totm;

            // remove DM offset
            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] -= offset;

            offset = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
                offset += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];

            // set RMS = 1 over mask
            rms = 0.0;
            for(ii=0; ii<msizex*msizey; ii++)
            {
                data.image[ID].array.F[k*msizex*msizey+ii] -= offset/totm;
                rms += data.image[ID].array.F[k*msizex*msizey+ii]*data.image[ID].array.F[k*msizex*msizey+ii]*data.image[IDmask].array.F[ii];
            }
            rms = sqrt(rms/totm);
            printf("Mode %ld   RMS = %lf\n", k, rms);

            for(ii=0; ii<msizex*msizey; ii++)
                data.image[ID].array.F[k*msizex*msizey+ii] /= rms;
        }

        save_fits(ID_name, "!./mkmodestmp/fmodes0all.fits");









        long IDmodes0all = image_ID(ID_name);
        printf("DONE SAVING\n");

        // time : 0:04



        /// COMPUTE WFS RESPONSE TO MODES -> fmodesWFS00all.fits
        msizexy = msizex*msizey;
        ID = image_ID(ID_name);
        IDzrespM = image_ID("zrespM");
        save_fits("zrespM", "!_test_zrespM.fits");
        save_fits(ID_name, "!_test_name.fits");
        if(data.image[IDzrespM].md[0].size[2]!=msizexy)
        {
            printf("ERROR: zrespM has wrong z size : %ld, should be %ld\n", (long) data.image[IDzrespM].md[0].size[2], (long) msizexy);
            exit(0);
        }

        wfsxsize = data.image[IDzrespM].md[0].size[0];
        wfsysize = data.image[IDzrespM].md[0].size[1];
        wfssize = wfsxsize*wfsysize;
        IDm = create_3Dimage_ID("fmodesWFS00all", wfsxsize, wfsysize, data.image[ID].md[0].size[2]);

        printf("size: %ld %ld %ld\n", (long) data.image[ID].md[0].size[2], msizexy, wfssize);
        printf("\n");

        long act1, act2;
# ifdef _OPENMP
        #pragma omp parallel for private(m,m1,act,act1,act2,wfselem)
# endif

        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            m1 = m*wfssize;

            printf("\r %5ld / %5ld   ", m, (long) data.image[ID].md[0].size[2]);
            fflush(stdout);
            for(act=0; act<msizexy; act++)
            {
                act1 = m*msizexy+act;
                act2 = act*wfssize;
                for(wfselem=0; wfselem<wfssize; wfselem++)
                {
                    data.image[IDm].array.F[m1+wfselem] += data.image[ID].array.F[act1] * data.image[IDzrespM].array.F[act2+wfselem];
                }
            }
        }



        // if modal response matrix exists, use it
        IDRMMmodes = image_ID("RMMmodes"); // modal resp matrix modes
        IDRMMresp = image_ID("RMMresp"); // modal resp matrix

        fpLOcoeff = fopen("./mkmodestmp/LOcoeff.txt", "w");
        if(fpLOcoeff == NULL)
        {
            printf("ERROR: cannot create file \"LOcoeff.txt\"\n");
            exit(0);
        }
        save_fits("fmodesWFS00all", "!./mkmodestmp/fmodesWFS00all.HO.fits");

        if((IDRMMmodes!=-1)&&(IDRMMresp!=-1))
        {
            linfitsize = data.image[IDRMMmodes].md[0].size[2];
            IDRMM_coeff = create_2Dimage_ID("linfitcoeff", linfitsize, 1);

            ID_imfit = create_2Dimage_ID("imfitim", msizex, msizey);

            IDcoeffmat = create_2Dimage_ID("imfitmat", linfitsize, data.image[ID].md[0].size[2]);

            linfitreuse = 0;

            IDwfstmp = create_2Dimage_ID("wfsimtmp", wfsxsize, wfsysize);

            for(m=0; m<data.image[IDmodes0all].md[0].size[2]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[ID_imfit].array.F[ii] = data.image[IDmodes0all].array.F[m*msizexy+ii];

                linopt_imtools_image_fitModes("imfitim", "RMMmodes", "dmmaskRM", 1.0e-2, "linfitcoeff", linfitreuse);
                linfitreuse = 1;

                for(jj=0; jj<linfitsize; jj++)
                    data.image[IDcoeffmat].array.F[m*linfitsize+jj] = data.image[IDRMM_coeff].array.F[jj];

                // construct linear fit result (DM)
                IDtmp = create_2Dimage_ID("testrc", msizex, msizey);
                for(jj=0; jj<linfitsize; jj++)
                    for(ii=0; ii<msizex*msizey; ii++)
                        data.image[IDtmp].array.F[ii] += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMMmodes].array.F[jj*msizex*msizey+ii];

                res = 0.0;
                resn = 0.0;
                for(ii=0; ii<msizex*msizey; ii++)
                {
                    v0 = data.image[IDtmp].array.F[ii]-data.image[ID_imfit].array.F[ii];
                    vn = data.image[ID_imfit].array.F[ii];
                    res += v0*v0;
                    resn += vn*vn;
                }
                res /= resn;

                res1 = 0.0;
                for(jj=0; jj<linfitsize; jj++)
                    res1 += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMM_coeff].array.F[jj];

                delete_image_ID("testrc");


                LOcoeff = 1.0/(1.0+pow(10.0*res, 4.0));

                if(res1>1.0)
                    LOcoeff *= 1.0/(1.0+pow((res1-1.0)*0.1, 2.0));


                fprintf(fpLOcoeff, "%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
                // printf("%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);

                if(LOcoeff>0.01)
                {
                    // construct linear fit (WFS space)
                    for(wfselem=0; wfselem<wfssize; wfselem++)
                        data.image[IDwfstmp].array.F[wfselem] = 0.0;
                    for(jj=0; jj<linfitsize; jj++)
                        for(wfselem=0; wfselem<wfssize; wfselem++)
                            data.image[IDwfstmp].array.F[wfselem] += data.image[IDRMM_coeff].array.F[jj] * data.image[IDRMMresp].array.F[jj*wfssize+wfselem];

                    for(wfselem=0; wfselem<wfssize; wfselem++)
                        data.image[IDm].array.F[m*wfssize+wfselem] = LOcoeff*data.image[IDwfstmp].array.F[wfselem] + (1.0-LOcoeff)*data.image[IDm].array.F[m*wfssize+wfselem];
                }
            }

            delete_image_ID("linfitcoeff");
            delete_image_ID("imfitim");
            delete_image_ID("wfsimtmp");
            save_fits("imfitmat", "!imfitmat.fits");
            delete_image_ID("imfitmat");
        }
        fclose(fpLOcoeff);

        printf("\n");
        save_fits("fmodesWFS00all", "!./mkmodestmp/fmodesWFS00all.fits");


        //    exit(0);




        // time : 0:42





        /// STEP 2: SEPARATE DM MODES INTO BLOCKS AND MASK
        msizexy = msizex*msizey;

        CPAblocklim[0] = 0.1; // tip and tilt
        CPAblocklim[1] = 0.3; // focus
        CPAblocklim[2] = 1.6; // other Zernikes
        CPAblocklim[3] = 3.0;
        CPAblocklim[4] = 5.0;
        CPAblocklim[5] = 7.0;
        CPAblocklim[6] = 9.0;
        CPAblocklim[7] = 11.0;
        CPAblocklim[8] = 13.0;
        CPAblocklim[9] = 15.0;
        CPAblocklim[10] = 17.0;
        CPAblocklim[11] = 19.0;
        CPAblocklim[12] = 21.0;
        CPAblocklim[13] = 100.0;

        for(mblock=0; mblock<MAX_MBLOCK; mblock++)
            MBLOCK_NBmode[mblock] = 0;



        NBmblock = 0;
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            float cpa = data.image[IDmfcpa].array.F[m];
            mblock = 0;
            while (cpa > CPAblocklim[mblock])
            {
                //    printf("[%ld  %f %f -> +]\n", mblock, cpa, CPAblocklim[mblock]);
                mblock++;
            }

            MBLOCK_NBmode[mblock]++;

            if(mblock>NBmblock)
                NBmblock = mblock;

            //    printf("%ld %f  -> %ld\n", m, cpa, mblock);
        }

        NBmblock++;


        long IDextrablock = image_ID("extrablockM");
        if(IDextrablock != -1)
        {
            extrablockIndex = 4;

            fp = fopen("./conf/param_extrablockIndex.txt", "r");
            if(fp != NULL)
            {
                if(fscanf(fp, "%50ld", &extrablockIndex) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "cannot read parameter from file");
                fclose(fp);
            }
        }



        for(mblock=0; mblock<NBmblock; mblock++)
        {
            long mblock1;

            if(IDextrablock != -1)
            {
                mblock1 = mblock;
                if(mblock>extrablockIndex-1)
                    mblock1 = mblock+1;
            }
            else
                mblock1 = mblock;

            if(sprintf(imname, "fmodes0_%02ld", mblock1) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            MBLOCK_ID[mblock1] = create_3Dimage_ID(imname, msizex, msizey, MBLOCK_NBmode[mblock]);
            MBLOCK_ID[mblock1] = image_ID(imname);
        }



        for(mblock=0; mblock<MAX_MBLOCK; mblock++)
            MBLOCK_NBmode[mblock] = 0;


		ID = image_ID("fmodes");
        for(m=0; m<data.image[ID].md[0].size[2]; m++)
        {
            long mblock1;

            float cpa = data.image[IDmfcpa].array.F[m];

            mblock = 0;
            while (cpa > CPAblocklim[mblock])
                mblock++;

            if(IDextrablock!= -1)
            {
                mblock1 = mblock;
                if(mblock>extrablockIndex-1)
                    mblock1 = mblock+1;
            }
            else
                mblock1 = mblock;


            for(ii=0; ii<msizex*msizey; ii++)
                data.image[MBLOCK_ID[mblock1]].array.F[MBLOCK_NBmode[mblock1]*msizex*msizey+ii] = data.image[ID].array.F[m*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

            MBLOCK_NBmode[mblock1]++;
        }


        if(IDextrablock != -1)
        {
            mblock = extrablockIndex;

            if(sprintf(imname, "fmodes0_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            MBLOCK_NBmode[mblock] = data.image[IDextrablock].md[0].size[2];
            MBLOCK_ID[mblock] = create_3Dimage_ID(imname, msizex, msizey, MBLOCK_NBmode[mblock]);

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                for(ii=0; ii<msizex*msizey; ii++)
                    data.image[MBLOCK_ID[mblock]].array.F[m*msizex*msizey+ii] = data.image[IDextrablock].array.F[m*msizex*msizey+ii]*data.image[IDmaskRM].array.F[ii];

            NBmblock++;
        }





        // time : 00:42

        /// STEP 3: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim00 FOR CUTOFF -> fmodes1all.fits  (DM space)
        printf("STEP 3: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim00 FOR CUTOFF -> fmodes1all.fits  (DM space)\n");
        fflush(stdout);

        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf("\nMODE BLOCK %ld\n", mblock);
            fflush(stdout);

            if(sprintf(imname, "fmodes0_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

			//TEST
			//sprintf(fname, "!./mkmodestmp/fmodes0_%02ld.fits", mblock);
			//save_fits(imname, fname);

            printf("SVD decomp ... (%ld) .... ", (long) data.image[image_ID(imname)].md[0].size[2]);
            fflush(stdout);
            linopt_compute_SVDdecomp(imname, "svdmodes", "svdcoeff");
            printf("DONE\n");
            fflush(stdout);
            cnt = 0;
            IDSVDcoeff = image_ID("svdcoeff");
            float svdcoeff0 = data.image[IDSVDcoeff].array.F[0];
            for(m=0; m<data.image[IDSVDcoeff].md[0].size[0]; m++)
            {
				//printf("( %ld -> %g )\n", m, data.image[IDSVDcoeff].array.F[m]);
                if(data.image[IDSVDcoeff].array.F[m] > SVDlim00*svdcoeff0)
                    cnt++;
            }
            printf("BLOCK %ld/%ld: keeping %ld / %ld modes  ( %f %f ) [%ld  %ld %ld]\n", mblock, NBmblock, cnt, m, SVDlim00, svdcoeff0, (long) data.image[IDSVDcoeff].md[0].size[0], msizex, msizey);
            fflush(stdout);

            if(sprintf(imname1, "fmodes1_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDm = create_3Dimage_ID(imname1, msizex, msizey, cnt);
            long IDSVDmodes = image_ID("svdmodes");
            for(ii=0; ii<cnt*msizex*msizey; ii++)
                data.image[IDm].array.F[ii] = data.image[IDSVDmodes].array.F[ii];

            MBLOCK_NBmode[mblock] = cnt;
            MBLOCK_ID[mblock] = IDm;

            if(sprintf(fname1, "!./mkmodestmp/fmodes1_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname1);

            delete_image_ID("svdmodes");
            delete_image_ID("svdcoeff");
        }


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes1all", msizex, msizey, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                // printf("Writing cnt %ld    %ld of %ld  [%ld -> %ld]\n", cnt, m, mblock, MBLOCK_ID[mblock], IDm);
                cnt++;
            }
        }
        save_fits("fmodes1all", "!./mkmodestmp/fmodes1all.fits");


	


        /// STEP 4: REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes2all.fits  (DM space)
        /// fmodes1all -> fmodes2all
        printf("STEP 4: REMOVE MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE DM-SPACE ORTHOGONALITY BETWEEN BLOCKS -> fmodes2all.fits  (DM space)\n");
        fflush(stdout);

        IDSVDmask = create_2Dimage_ID("SVDmask", msizex, msizey);
        for(ii=0; ii<msizexy; ii++)
            data.image[IDSVDmask].array.F[ii] = data.image[IDmaskRM].array.F[ii];
        IDSVDmodein = create_2Dimage_ID("SVDmodein", msizex, msizey);

        mok = (int*) malloc(sizeof(int)*NBmm);
        for(m=0; m<NBmm; m++)
            mok[m] = 1;

        for(mblock=0; mblock<NBmblock; mblock++)
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                mok[m] = 1;
            for(mblock0=0; mblock0<mblock; mblock0++)
            {
                reuse = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    //  printf("STEP 4: REMOVING BLOCK %ld from   block %ld mode %ld/%ld      ", mblock0, mblock, m, MBLOCK_NBmode[mblock]);
                    //  fflush(stdout);

                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDSVDmodein].array.F[ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];

                    if(sprintf(imname, "fmodes1_%02ld", mblock0) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    linopt_imtools_image_fitModes("SVDmodein", imname, "SVDmask", 1.0e-2, "modecoeff", reuse);


                    reuse = 1;
                    linopt_imtools_image_construct(imname, "modecoeff", "SVDmode1");
                    IDSVDmode1 = image_ID("SVDmode1");
                    delete_image_ID("modecoeff");
                    value1 = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                    {
                        data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii] -= data.image[IDSVDmode1].array.F[ii];;
                        value1 += data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii]*data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                    }
                    delete_image_ID("SVDmode1");

                    rms = sqrt(value1/totm);
                    float rmslim0 = 0.01;
                    if(rms>rmslim0)
                    {
                        //       for(ii=0; ii<msizexy; ii++)
                        //         data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii] /= rms;
                    }
                    else
                        mok[m] = 0;

                    //                    printf("->  %12g (%g %g)\n", rms, value1, totm);
                    //					fflush(stdout);
                }
            }
            cnt = 0;
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                cnt += mok[m];
            printf("====== BLOCK %ld : keeping %ld / %ld modes\n", mblock, cnt, MBLOCK_NBmode[mblock]);
            fflush(stdout);
            if(cnt>0)
            {
                if(sprintf(imname, "fmodes2_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                printf("saving result %s \n", imname);
                fflush(stdout);
                IDm = create_3Dimage_ID(imname, msizex, msizey, cnt);
                m1 = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    if(mok[m]==1)
                    {
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDm].array.F[m1*msizex*msizey+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                        printf("BLOCK %ld   [%ld]  m1 = %ld / %ld\n", mblock, IDm, m1, cnt);
                        fflush(stdout);
                        m1++;
                    }
                }
                MBLOCK_ID[mblock] = IDm;

                char fname2[200];
                if(sprintf(fname2, "!./mkmodestmp/fmodes2_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imname, fname2);
            }
            MBLOCK_NBmode[mblock] = cnt;
        }

        delete_image_ID("SVDmask");
        delete_image_ID("SVDmodein");

        free(mok);


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes2all", msizex, msizey, cnt);


        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                cnt++;
            }
        }
        save_fits("fmodes2all", "!./mkmodestmp/fmodes2all.fits");







        /// STEP 5: REMOVE NULL SPACE WITHIN EACH BLOCK - USE SVDlim01 FOR CUTOFF -> fmodes2ball.fits  (DM space)
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf("MODE BLOCK %ld\n", mblock);
            fflush(stdout);

            if(sprintf(imname, "fmodes2_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            printf("SVD decomp ...");
            fflush(stdout);
            linopt_compute_SVDdecomp(imname, "svdmodes", "svdcoeff");
            printf("DONE\n");
            fflush(stdout);
            cnt = 0;
            IDSVDcoeff = image_ID("svdcoeff");
            float svdcoeff0 = data.image[IDSVDcoeff].array.F[0];

            if(sprintf(fnameSVDcoeff, "./mkmodestmp/SVDcoeff01_%02ld.txt", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            fpcoeff = fopen(fnameSVDcoeff, "w");
            for(m=0; m<data.image[IDSVDcoeff].md[0].size[0]; m++)
            {
                fprintf(fpcoeff, "%5ld   %12g   %12g  %5ld     %10.8f  %10.8f\n", m, data.image[IDSVDcoeff].array.F[m], data.image[IDSVDcoeff].array.F[0], cnt, data.image[IDSVDcoeff].array.F[m]/data.image[IDSVDcoeff].array.F[0], SVDlim01);

                if(data.image[IDSVDcoeff].array.F[m]>SVDlim01*svdcoeff0)
                    cnt++;
            }
            fclose(fpcoeff);

            printf("BLOCK %ld/%ld: keeping %ld / %ld modes\n", mblock, NBmblock, cnt, m);
            fflush(stdout);

            if(sprintf(imname1, "fmodes2b_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDm = create_3Dimage_ID(imname1, msizex, msizey, cnt);
            long IDSVDmodes = image_ID("svdmodes");
            for(ii=0; ii<cnt*msizex*msizey; ii++)
                data.image[IDm].array.F[ii] = data.image[IDSVDmodes].array.F[ii];

            for(m=0; m<cnt; m++)
            {
                value1 = 0.0;
                value1cnt = 0.0;
                for(ii=0; ii<msizexy; ii++)
                {
                    value1 += data.image[IDm].array.F[m*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                    value1cnt += data.image[IDmaskRM].array.F[ii];
                }
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] -= value1/value1cnt;

                value1 = 0.0;
                for(ii=0; ii<msizexy; ii++)
                    value1 += data.image[IDm].array.F[m*msizexy+ii]*data.image[IDm].array.F[m*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                rms = sqrt(value1/value1cnt);
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] /= rms;
            }

            // Extrapolate outside maskRM
            IDtmp = AOloopControl_computeCalib_DMslaveExt(data.image[IDm].md[0].name, data.image[IDmaskRM].md[0].name, "dmslaved", "fmodesext", 100.0);
            for(m=0; m<cnt; m++)
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[m*msizexy+ii] = data.image[IDtmp].array.F[m*msizexy+ii];
            delete_image_ID("fmodesext");


            MBLOCK_NBmode[mblock] = cnt;
            MBLOCK_ID[mblock] = IDm;

            if(sprintf(fname1, "!./mkmodestmp/fmodes2b_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname1);

            delete_image_ID("svdmodes");
            delete_image_ID("svdcoeff");
        }


        fp = fopen("./mkmodestmp/NBblocks.txt", "w");
        fprintf(fp, "%ld\n", NBmblock);
        fclose(fp);

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodes2ball", msizex, msizey, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                // printf("Writing cnt %ld    %ld of %ld  [%ld -> %ld]\n", cnt, m, mblock, MBLOCK_ID[mblock], IDm);
                cnt++;
            }
        }
        save_fits("fmodes2ball", "!./mkmodestmp/fmodes2ball.fits");
    }
    else
    {
        fp = fopen("./mkmodestmp/NBblocks.txt", "r");
        if(fscanf(fp, "%50ld", &NBmblock) != 1)
            printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

        fclose(fp);
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(fname, "./mkmodestmp/fmodes2b_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imname, "fmodes2b_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            ID = load_fits(fname, imname, 1);
            MBLOCK_NBmode[mblock] = data.image[ID].md[0].size[2];
            MBLOCK_ID[mblock] = ID;
        }
    }







    // 1:25


    // ==================================================



    // WFS modes
    IDzrespM = image_ID("zrespM");
    if(IDzrespM!=-1) // compute WFS response to DM modes
    {
        /// STEP 6: COMPUTE WFS RESPONSE TO MODES
        /// fmodes2ball -> fmodesWFS0all.fits

        char imnameDM[200];
        char imnameDM1[200];
        long MBLOCK_IDwfs[MAX_MBLOCK];



        if(BlockNB<0)
        {   // check size
            if(data.image[IDzrespM].md[0].size[2]!=msizexy)
            {
                printf("ERROR: zrespM has wrong z size : %ld, should be %ld\n", (long) data.image[IDzrespM].md[0].size[2], (long) msizexy);
                exit(0);
            }

            wfsxsize = data.image[IDzrespM].md[0].size[0];
            wfsysize = data.image[IDzrespM].md[0].size[1];
            wfssize = wfsxsize*wfsysize;


            /// Load ... or create WFS mask
            long IDwfsmask = image_ID("wfsmask");
            if((wfsxsize!=data.image[IDwfsmask].md[0].size[0])||(wfsysize!=data.image[IDwfsmask].md[0].size[1]))
            {
                printf("ERROR: File wfsmask has wrong size\n");
                exit(0);
            }
            if(IDwfsmask==-1)
            {
                IDwfsmask = create_2Dimage_ID("wfsmask", wfsxsize, wfsysize);
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDwfsmask].array.F[ii] = 1.0;
            }



            for(mblock=0; mblock<NBmblock; mblock++)
            {
                printf("BLOCK %ld has %ld modes\n", mblock, MBLOCK_NBmode[mblock]);
                fflush(stdout);


                if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(MBLOCK_NBmode[mblock]>0)
                {
                    long IDwfsMresp = create_3Dimage_ID(imname, wfsxsize, wfsysize, MBLOCK_NBmode[mblock]);

# ifdef _OPENMP
                    #pragma omp parallel for private(m,act,wfselem)
# endif
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        for(act=0; act<msizexy; act++)
                        {
                            for(wfselem=0; wfselem<wfssize; wfselem++)
                            {
                                data.image[IDwfsMresp].array.F[m*wfssize+wfselem] += data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+act] * data.image[IDzrespM].array.F[act*wfssize+wfselem];
                            }
                        }
                    }

                    if((IDRMMmodes!=-1)&&(IDRMMresp!=-1))
                    {
                        char fnameLOcoeff[200];
                        if(sprintf(fnameLOcoeff, "./mkmodestmp/LOcoeff_%02ld.txt", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        fpLOcoeff = fopen(fnameLOcoeff, "w");
                        if(fpLOcoeff == NULL)
                        {
                            printf("ERROR: cannot create file \"LOcoeff1.txt\"\n");
                            exit(0);
                        }



                        linfitsize = data.image[IDRMMmodes].md[0].size[2];
                        IDRMM_coeff = create_2Dimage_ID("linfitcoeff", linfitsize, 1);

                        ID_imfit = create_2Dimage_ID("imfitim", msizex, msizey);

                        IDcoeffmat = create_2Dimage_ID("imfitmat", linfitsize, data.image[ID].md[0].size[2]);

                        linfitreuse = 0;

                        IDwfstmp = create_2Dimage_ID("wfsimtmp", wfsxsize, wfsysize);

                        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                        {
                            for(ii=0; ii<msizexy; ii++)
                                data.image[ID_imfit].array.F[ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];

                            linopt_imtools_image_fitModes("imfitim", "RMMmodes", "dmmaskRM", 1.0e-2, "linfitcoeff", linfitreuse);
                            linfitreuse = 1;

                            for(jj=0; jj<linfitsize; jj++)
                                data.image[IDcoeffmat].array.F[m*linfitsize+jj] = data.image[IDRMM_coeff].array.F[jj];

                            // prevent large coefficients (noise propagation)


                            // construct linear fit result (DM)
                            IDtmp = create_2Dimage_ID("testrc", msizex, msizey);
                            for(jj=0; jj<linfitsize; jj++)
                                for(ii=0; ii<msizex*msizey; ii++)
                                    data.image[IDtmp].array.F[ii] += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMMmodes].array.F[jj*msizex*msizey+ii];

                            res = 0.0;
                            resn = 0.0;
                            for(ii=0; ii<msizex*msizey; ii++)
                            {
                                v0 = data.image[IDtmp].array.F[ii]-data.image[ID_imfit].array.F[ii];
                                vn = data.image[ID_imfit].array.F[ii];
                                res += v0*v0;
                                resn += vn*vn;
                            }
                            res /= resn;

                            res1 = 0.0;  // norm squared of linear vector
                            for(jj=0; jj<linfitsize; jj++)
                                res1 += data.image[IDRMM_coeff].array.F[jj]*data.image[IDRMM_coeff].array.F[jj];

                            delete_image_ID("testrc");


                            LOcoeff = 1.0/(1.0+pow(10.0*res, 4.0));

                            if(res1>1.0)
                                LOcoeff *= 1.0/(1.0+pow((res1-1.0)*0.1, 2.0));


                            fprintf(fpLOcoeff, "%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);
                            // printf("%5ld   %20g  %20g   ->  %f\n", m, res, res1, LOcoeff);

                            if(LOcoeff>0.01)
                            {
                                // construct linear fit (WFS space)
                                for(wfselem=0; wfselem<wfssize; wfselem++)
                                    data.image[IDwfstmp].array.F[wfselem] = 0.0;
                                for(jj=0; jj<linfitsize; jj++)
                                    for(wfselem=0; wfselem<wfssize; wfselem++)
                                        data.image[IDwfstmp].array.F[wfselem] += data.image[IDRMM_coeff].array.F[jj] * data.image[IDRMMresp].array.F[jj*wfssize+wfselem];

                                for(wfselem=0; wfselem<wfssize; wfselem++)
                                    data.image[IDwfsMresp].array.F[m*wfssize+wfselem] = LOcoeff*data.image[IDwfstmp].array.F[wfselem] + (1.0-LOcoeff)*data.image[IDwfsMresp].array.F[m*wfssize+wfselem];
                            }
                        }

                        delete_image_ID("linfitcoeff");
                        delete_image_ID("imfitim");

                        save_fits("imfitmat", "!imfitmat.fits");
                        delete_image_ID("imfitmat");

                        fclose(fpLOcoeff);
                    }


                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                        for(wfselem=0; wfselem<wfssize; wfselem++)
                            data.image[IDwfsMresp].array.F[m*wfssize+wfselem] *= data.image[IDwfsmask].array.F[wfselem];


                    if(sprintf(fname, "!./mkmodestmp/fmodesWFS0_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname, fname);
                }
            }

            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
                cnt += MBLOCK_NBmode[mblock];
            IDm = create_3Dimage_ID("fmodesWFS0all", wfsxsize, wfsysize, cnt);
            cnt = 0;


            for(mblock=0; mblock<NBmblock; mblock++)
            {
                if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs = image_ID(imname);
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    for(ii=0; ii<wfssize; ii++)
                        data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                    cnt++;
                }
            }
            save_fits("fmodesWFS0all", "!./mkmodestmp/fmodesWFS0all.fits");





            // time : 02:00


            /// STEP 7: REMOVE WFS MODES THAT ARE CONTAINED IN PREVIOUS BLOCKS, AND ENFORCE WFS-SPACE ORTHOGONALITY BETWEEN BLOCKS
            /// Input: fmodesWFS0all (corresponding to fmodes2ball)
            /// Output -> fmodesWFS1all / fmodes3all

            IDSVDmask = create_2Dimage_ID("SVDmask", wfsxsize, wfsysize);
            for(ii=0; ii<wfssize; ii++)
                data.image[IDSVDmask].array.F[ii] = 1.0;
            IDSVDmodein = create_2Dimage_ID("SVDmodein", wfsxsize, wfsysize);

            mok = (int*) malloc(sizeof(int)*NBmm);
            for(m=0; m<NBmm; m++)
                mok[m] = 1;



            for(mblock=0; mblock<NBmblock; mblock++)
            {
                float *rmsarray;
                rmsarray = (float*) malloc(sizeof(float)*MBLOCK_NBmode[mblock]);
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                {
                    if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);
                    value1 = 0.0;
                    for(ii=0; ii<wfssize; ii++)
                        value1 += data.image[IDmwfs].array.F[m*wfssize+ii]*data.image[IDmwfs].array.F[m*wfssize+ii];
                    rmsarray[m] = sqrt(value1/wfssize);
                }

                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    mok[m] = 1;



                // REMOVE WFS MODES FROM PREVIOUS BLOCKS

                for(mblock0=0; mblock0<mblock; mblock0++)
                {

                    reuse = 0;
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        long IDmwfs = image_ID(imname);

                        if(sprintf(imnameDM, "fmodes2b_%02ld", mblock) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        IDm = image_ID(imnameDM);


                        for(ii=0; ii<wfsxsize*wfsysize; ii++)
                            data.image[IDSVDmodein].array.F[ii] = data.image[IDmwfs].array.F[m*wfssize+ii];

                        if(sprintf(imname, "fmodesWFS0_%02ld", mblock0) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        if(sprintf(imnameDM, "fmodes2b_%02ld", mblock0) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        linopt_imtools_image_fitModes("SVDmodein", imname, "SVDmask", 1.0e-2, "modecoeff", reuse);
                        IDSVDcoeff = image_ID("modecoeff");
                        reuse = 1;
                        linopt_imtools_image_construct(imname, "modecoeff", "SVDmode1");
                        linopt_imtools_image_construct(imnameDM, "modecoeff", "SVDmode1DM");
                        IDSVDmode1 = image_ID("SVDmode1");

                        long IDSVDmode1DM = image_ID("SVDmode1DM");

                        delete_image_ID("modecoeff");

                        value1 = 0.0;
                        for(ii=0; ii<wfssize; ii++)
                        {
                            data.image[IDmwfs].array.F[m*wfssize+ii] -= data.image[IDSVDmode1].array.F[ii];
                            value1 += data.image[IDmwfs].array.F[m*wfssize+ii]*data.image[IDmwfs].array.F[m*wfssize+ii];
                        }
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDm].array.F[m*msizexy+ii] -= data.image[IDSVDmode1DM].array.F[ii];


                        delete_image_ID("SVDmode1");
                        delete_image_ID("SVDmode1DM");

                        rms = sqrt(value1/wfssize);

                        if(rms<rmsarray[m]*rmslim1)
                        {
                            mok[m] = 0;
                        }
                        printf("RMS RATIO  %3ld :   %12g\n", m, rms/rmsarray[m]);
                    }
                }

                cnt = 0;
                for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    cnt += mok[m];
                printf("====== WFS BLOCK %ld : keeping %ld / %ld modes\n", mblock, cnt, MBLOCK_NBmode[mblock]);

                if(cnt>0)
                {
                    if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");


                    long IDmwfs1 = create_3Dimage_ID(imname, wfsxsize, wfsysize, cnt);
                    long IDmdm1 = create_3Dimage_ID(imnameDM, msizex, msizey, cnt);
                    m1 = 0;

                    if(sprintf(imname, "fmodesWFS0_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);

                    if(sprintf(imnameDM, "fmodes2b_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmdm = image_ID(imnameDM);
                    if(IDmdm==-1)
                    {
                        printf("ERROR: image %s does not exist\n", imnameDM);
                        exit(0);
                    }
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        if(mok[m]==1)
                        {
                            printf("writing %ld / %ld  ->  %ld / %ld        \n", m, (long) data.image[IDmwfs].md[0].size[2], m1, (long) data.image[IDm].md[0].size[2]);

                            printf("max index IDmwfs1 %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m1, (long) (m1*wfssize+wfssize-1), (long) (data.image[IDmwfs1].md[0].size[0]*data.image[IDmwfs1].md[0].size[1]*data.image[IDmwfs1].md[0].size[2]), (long) data.image[IDmwfs1].md[0].size[0], (long) data.image[IDmwfs1].md[0].size[1], (long) data.image[IDmwfs1].md[0].size[2]);
                            printf("max index IDmwfs  %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m, (long) (m*wfssize+wfssize-1), (long) (data.image[IDmwfs].md[0].size[0]*data.image[IDmwfs].md[0].size[1]*data.image[IDmwfs].md[0].size[2]), (long) data.image[IDmwfs].md[0].size[0], (long) data.image[IDmwfs].md[0].size[1], (long) data.image[IDmwfs].md[0].size[2]);

                            printf("max index IDmdm1  %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m1, (long) (m1*msizexy+msizexy-1), (long) (data.image[IDmdm1].md[0].size[0]*data.image[IDmdm1].md[0].size[1]*data.image[IDmdm1].md[0].size[2]), (long) data.image[IDmdm1].md[0].size[0], (long) data.image[IDmdm1].md[0].size[1], (long) data.image[IDmdm1].md[0].size[2]);
                            printf("max index IDmdm   %ld  = %ld / %ld    [ %ld %ld %ld ]\n", (long) m, (long) (m*msizexy+msizexy-1), (long) (data.image[IDmdm].md[0].size[0]*data.image[IDmdm].md[0].size[1]*data.image[IDmdm].md[0].size[2]), (long) data.image[IDmdm].md[0].size[0], (long) data.image[IDmdm].md[0].size[1], (long) data.image[IDmdm].md[0].size[2]);


                            fflush(stdout);//TEST
                            for(ii=0; ii<wfssize; ii++)
                                data.image[IDmwfs1].array.F[m1*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                            for(ii=0; ii<msizexy; ii++)
                                data.image[IDmdm1].array.F[m1*msizexy+ii] = data.image[IDmdm].array.F[m*msizexy+ii];
                            value1 = 0.0;
                            m1++;
                        }
                        else
                        {
                            printf("Skipping %ld / %ld\n", m, (long) data.image[IDmwfs].md[0].size[2]);
                            fflush(stdout);
                        }
                    }
                    printf("STEP 0000\n");
                    fflush(stdout);//TEST

                    if(sprintf(imname1, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(fname1, "!./mkmodestmp/fmodesWFS1_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    printf("   saving   %s -> %s\n", imname1, fname1);
                    fflush(stdout);//TEST

                    save_fits(imname1, fname1);

                    printf("STEP 0001\n");
                    fflush(stdout);//TEST

                    if(sprintf(imname1, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(fname1, "!./mkmodestmp/fmodes3_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname1, fname1);
                    MBLOCK_ID[mblock] = IDmdm1;
                    printf("STEP 0002\n");
                    fflush(stdout);//TEST
                }
                else
                {
                    printf("ERROR: keeping no mode in block !!!\n");
                    exit(0);
                }
                printf("STEP 0010\n");
                fflush(stdout);//TEST

                MBLOCK_NBmode[mblock] = cnt;
                free(rmsarray);
            }
            delete_image_ID("SVDmask");
            delete_image_ID("SVDmodein");

            printf("STEP 0020\n");
            fflush(stdout);//TEST

            free(mok);


            // time : 04:34


            list_image_ID();
            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
                cnt += MBLOCK_NBmode[mblock];
            IDm = create_3Dimage_ID("fmodesWFS1all", wfsxsize, wfsysize, cnt);
            long IDmdm1 = create_3Dimage_ID("fmodes3all", msizex, msizey, cnt);

            cnt = 0;
            for(mblock=0; mblock<NBmblock; mblock++)
            {
                if(MBLOCK_NBmode[mblock]>0)
                {
                    if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmwfs = image_ID(imname);

                    if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    long IDmdm = image_ID(imnameDM);

                    if(IDmwfs==-1)
                    {
                        printf("ERROR: image %s does not exit\n", imname);
                        exit(0);
                    }
                    for(m=0; m<MBLOCK_NBmode[mblock]; m++)
                    {
                        // printf("writing %ld / %ld  ->  %ld / %ld\n", m, data.image[IDmwfs].md[0].size[2], cnt, data.image[IDm].md[0].size[2]);
                        // fflush(stdout);
                        for(ii=0; ii<wfssize; ii++)
                            data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDmdm1].array.F[cnt*msizexy+ii] = data.image[IDmdm].array.F[m*msizexy+ii];
                        cnt++;
                    }
                }
            }
            save_fits("fmodesWFS1all", "!./mkmodestmp/fmodesWFS1all.fits");
            save_fits("fmodes3all", "!./mkmodestmp/fmodes3all.fits");


        }


        // time : 04:36

        if(BlockNB<0)
        {
            char command[1000];
            if(sprintf(command, "echo \"%ld\" > ./conf_staged/param_NBmodeblocks.txt", NBmblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");
        }
        else
        {
            if((fp = fopen("./conf/param_NBmodeblocks.txt", "r"))==NULL)
            {
                printf("ERROR: cannot read file ./conf_staged/param_NBmodeblocks.txt\n");
                exit(0);
            }
            if(fscanf(fp, "%50ld", &NBmblock) != 1)
                printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");
            fclose(fp);
        }

        printf("%ld blocks\n", NBmblock);



        /// STEP 8: SVD WFS SPACE IN EACH BLOCK
        /// fmodesWFS1all, fmodes3 -> fmodesall

        // fmodesWFS1_##, fmodes3_## -> fmodes_##

        for(mblock=0; mblock<NBmblock; mblock++)
        {
			long IDmask;
			
            if(BlockNB>-1) // LOAD & VERIFY SIZE
            {
                if(sprintf(imname1, "fmodesWFS1_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(fname1, "./mkmodestmp/fmodesWFS1_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                ID = load_fits(fname1, imname1, 1);
                wfsxsize = data.image[ID].md[0].size[0];
                wfsysize = data.image[ID].md[0].size[1];
                wfssize = wfsxsize*wfsysize;

                if(sprintf(imname1, "fmodes3_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(fname1, "./mkmodestmp/fmodes3_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                ID = load_fits(fname1, imname1, 1);
                if((data.image[ID].md[0].size[0] != msizex) && (msizey != data.image[ID].md[0].size[0]))
                {
                    printf("ERROR: file dmmaskRM size (%ld %ld) does not match expected size (%ld %ld)\n", (long) data.image[IDmask].md[0].size[0], (long) data.image[IDmask].md[0].size[1], (long) msizex, (long) msizey);
                    exit(0);
                }
                msizexy = data.image[ID].md[0].size[0]*data.image[ID].md[0].size[1];
            }


            if((BlockNB<0)||(BlockNB==mblock))
            {
				char command[1000];
                
                
                if(sprintf(command, "echo \"%f\" > ./conf_staged/block%02ld_SVDlim.txt", SVDlim, mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command) != 0)
                    printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");


                //if(MBLOCK_NBmode[mblock]>-1)
                //{

                if(sprintf(imname, "fmodesWFS1_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs = image_ID(imname);
                if(IDmwfs==-1)
                {
                    printf("ERROR: image %s does not exit\n", imname);
                    exit(0);
                }

                if(sprintf(imnameDM, "fmodes3_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmdm = image_ID(imnameDM);
                if(IDmdm==-1)
                {
                    printf("ERROR: image %s does not exit\n", imnameDM);
                    exit(0);
                }

                if(sprintf(imnameDM1, "fmodes_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");


                linopt_compute_SVDdecomp(imname, "SVDout", "modecoeff"); // SVD
                IDSVDcoeff = image_ID("modecoeff");

                cnt = 0;

                if(sprintf(fnameSVDcoeff, "./mkmodestmp/SVDcoeff_%02ld.txt", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                fpcoeff = fopen(fnameSVDcoeff, "w");
                uint_fast16_t kk;
                for(kk=0; kk<data.image[IDSVDcoeff].md[0].size[0]; kk++)
                {
                    fprintf(fpcoeff, "%5ld   %12g   %12g  %5ld     %10.8f  %10.8f\n", kk, data.image[IDSVDcoeff].array.F[kk], data.image[IDSVDcoeff].array.F[0], cnt, data.image[IDSVDcoeff].array.F[kk]/data.image[IDSVDcoeff].array.F[0], SVDlim);
                    printf("==== %ld %12g %12g  %3ld\n", kk, data.image[IDSVDcoeff].array.F[kk], data.image[IDSVDcoeff].array.F[0], cnt);
                    if(data.image[IDSVDcoeff].array.F[kk]>SVDlim*data.image[IDSVDcoeff].array.F[0])
                        cnt++;
                }
                fclose(fpcoeff);


                long IDmdm1 = create_3Dimage_ID(imnameDM1, msizex, msizey, cnt);

                char imnameWFS1[200];
                if(sprintf(imnameWFS1, "fmodesWFS_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmwfs1 = create_3Dimage_ID(imnameWFS1, wfsxsize, wfsysize, cnt);
                long ID_VTmatrix = image_ID("SVD_VTm");


                for(kk=0; kk<cnt; kk++) /// eigen mode index
                {
                    long kk1;
                    for(kk1=0; kk1<data.image[IDSVDcoeff].md[0].size[0]; kk1++)
                    {
                        for(ii=0; ii<msizexy; ii++)
                            data.image[IDmdm1].array.F[kk*msizexy + ii] += data.image[ID_VTmatrix].array.F[kk1*data.image[IDSVDcoeff].md[0].size[0]+kk]*data.image[IDmdm].array.F[kk1*msizexy + ii];

                        for(ii=0; ii<wfssize; ii++)
                            data.image[IDmwfs1].array.F[kk*wfssize + ii] += data.image[ID_VTmatrix].array.F[kk1*data.image[IDSVDcoeff].md[0].size[0]+kk]*data.image[IDmwfs].array.F[kk1*wfssize + ii];
                    }

                    value1 = 0.0;
                    value1cnt = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                    {
                        value1 += data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                        value1cnt += data.image[IDmaskRM].array.F[ii];
                    }
                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDmdm1].array.F[kk*msizexy+ii] -= value1/value1cnt;

                    value1 = 0.0;
                    for(ii=0; ii<msizexy; ii++)
                        value1 += data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmdm1].array.F[kk*msizexy+ii]*data.image[IDmaskRM].array.F[ii];
                    rms = sqrt(value1/value1cnt);

                    for(ii=0; ii<msizexy; ii++)
                        data.image[IDmdm1].array.F[kk*msizexy+ii] /= rms;

                    for(ii=0; ii<wfssize; ii++)
                        data.image[IDmwfs1].array.F[kk*wfssize+ii] /= rms;


                    /*     value1 = 0.0;
                         for(ii=0; ii<msizexy; ii++)
                             value1 += data.image[IDmdm1].array.F[kk*msizexy + ii]*data.image[IDmdm1].array.F[kk*msizexy + ii];
                         rms = sqrt(value1/totm);
                         */


                    // for(ii=0; ii<msizexy; ii++)
                    //     data.image[IDmdm1].array.F[kk*msizexy + ii] /= rms;
                }
                delete_image_ID("SVDout");
                delete_image_ID("modecoeff");

                if(sprintf(fname, "!./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imnameDM1, fname);

                if(sprintf(fname, "!./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                save_fits(imnameWFS1, fname);
                MBLOCK_ID[mblock] = IDmdm1;
                MBLOCK_IDwfs[mblock] = IDmwfs1;
                MBLOCK_NBmode[mblock] = cnt;
                //}
            }
            else
            {
                if(sprintf(fname, "./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(sprintf(imnameDM1, "fmodes_%02ld", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                long IDmdm1 = load_fits(fname, imnameDM1, 1);
                MBLOCK_ID[mblock] = IDmdm1;
                //MBLOCK_IDwfs[mblock] = IDmwfs1;
                MBLOCK_NBmode[mblock] = data.image[IDmdm1].md[0].size[2];
            }
        }

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodesall", msizex, msizey, cnt);
        long IDwfs = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        long cnt1 = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(MBLOCK_NBmode[mblock]>0)
                cnt1++;

            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<msizexy; ii++)
                    data.image[IDm].array.F[cnt*msizexy+ii] = data.image[MBLOCK_ID[mblock]].array.F[m*msizexy+ii];
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDwfs].array.F[cnt*wfssize+ii] = data.image[MBLOCK_IDwfs[mblock]].array.F[m*wfssize+ii];


                cnt++;
            }
        }

        save_fits("fmodesall", "!./mkmodestmp/fmodesall.fits");
        save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");

        NBmblock = cnt1;











        /// WFS MODES, MODAL CONTROL MATRICES
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            printf(".... BLOCK %ld has %ld modes\n", mblock, MBLOCK_NBmode[mblock]);
            fflush(stdout);

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCM[200]; // modal control matrix
            if(sprintf(imnameCM, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCMc[200]; // zonal ("combined") control matrix
            if(sprintf(imnameCMc, "cmatc_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            char imnameCMcact[200]; // zonal control matrix masked
            if(sprintf(imnameCMcact, "cmatcact_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if((BlockNB<0)||(BlockNB==mblock))
            {
                if(MBLOCK_NBmode[mblock]>0)
                {



                    printf("-- COMPUTE MODAL CONTROL MATRICES\n");
                    fflush(stdout);

                    // COMPUTE MODAL CONTROL MATRICES
                    printf("COMPUTE CONTROL MATRIX\n");
                    float SVDlim1 = 0.01; // WFS filtering (ONLY USED FOR FULL SINGLE STEP INVERSION)
#ifdef HAVE_MAGMA
                    CUDACOMP_magma_compute_SVDpseudoInverse(imname, imnameCM, SVDlim1, 10000, "VTmat", 0);
#else
                    linopt_compute_SVDpseudoInverse(imname, imnameCM, SVDlim1, 10000, "VTmat");
#endif

                    delete_image_ID("VTmat");

                    if(sprintf(fname, "!./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imnameCM, fname);

                    printf("-- COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX\n");
                    fflush(stdout);

                    // COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX
                    sprintf(imname, "fmodes_%02ld", mblock);
                    AOloopControl_computeCalib_compute_CombinedControlMatrix(imnameCM, imname, "wfsmask", "dmmask", imnameCMc, imnameCMcact);


                    if(sprintf(fname, "!./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imnameCMc, fname);

                    if(sprintf(fname, "!./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(imname1, "%s_00", imnameCMcact) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    save_fits(imname1, fname);

                    list_image_ID();
                }

            }
            else
            {
                printf("LOADING WFS MODES, MODAL CONTROL MATRICES: block %ld\n", mblock);
                fflush(stdout);

                //	list_image_ID();

                if(sprintf(fname, "./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imname, 1);

                if(sprintf(fname, "./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCM, 1);

                if(sprintf(fname, "./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCMc, 1);

                if(sprintf(fname, "./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                load_fits(fname, imnameCMcact, 1);
            }
        }

        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        IDm = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            char command[1000];
            if(sprintf(command, "echo \"%ld\" > ./conf_staged/block%02ld_NBmodes.txt", MBLOCK_NBmode[mblock], mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            long IDmwfs = image_ID(imname);
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
                cnt++;
            }
        }
        save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");


		fp = fopen("./mkmodestmp/NBmodes.txt", "w");
        fprintf(fp, "%ld\n", cnt);
        fclose(fp);		
		
		
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
            cnt += MBLOCK_NBmode[mblock];
        long IDcmatall = create_3Dimage_ID("cmatall", wfsxsize, wfsysize, cnt);
        cnt = 0;
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(imname, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            long IDcmat = image_ID(imname);
            for(m=0; m<MBLOCK_NBmode[mblock]; m++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[IDcmatall].array.F[cnt*wfssize+ii] = data.image[IDcmat].array.F[m*wfssize+ii];
                cnt++;
            }
        }
        save_fits("cmatall", "!./mkmodestmp/cmatall.fits");




        // COMPUTE OVERALL CONTROL MATRIX
        /*    int COMPUTE_FULL_CMAT = 0;
            if(COMPUTE_FULL_CMAT == 1)
            {
                printf("COMPUTE OVERALL CONTROL MATRIX\n");
                float SVDlim1 = 0.01; // WFS filtering (ONLY USED FOR FULL SINGLE STEP INVERSION)
                #ifdef HAVE_MAGMA
                    CUDACOMP_magma_compute_SVDpseudoInverse("fmodesWFSall", "cmat", SVDlim1, 100000, "VTmat", 0);
                #else
                    linopt_compute_SVDpseudoInverse("fmodesWFSall", "cmat", SVDlim1, 10000, "VTmat");
        		#endif

                delete_image_ID("VTmat");
                save_fits("cmat", "!./mkmodestmp/cmat.fits");

        	}

        		char command[1000];
                if(sprintf(command, "echo \"%ld\" > ./conf_staged/param_NBmodes.txt", cnt) < 1)
        			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                if(system(command) != 0)
        			printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

            */
    }
    // time : 07:43


    return(ID);
}




/*** \brief Creates control matrices per block, using native modes
 */
long AOloopControl_computeCalib_mkModes_Simple(const char *IDin_name, long NBmblock, long Cmblock, float SVDlim)
{
    long IDin; // input WFS responses
    FILE *fp;
    long mblock;
    long *MBLOCK_NBmode;
    long *MBLOCK_blockstart;
    long *MBLOCK_blockend;
    char fname[500];

    char imname[500];
    char imname1[500];
    long ID;
    long wfsxsize, wfsysize;
    long wfssize;
    long ii, kk;
    char imnameCM[500];
    char imnameCMc[500];
    char imnameCMcact[500];
    long IDwfsmask;
    long IDdmmask;
    long IDmodes;
    long NBmodes;
    long cnt;
    long IDm;
    long m;
    long IDcmatall;
    char command[500];


    printf("Function AOloopControl_mkModes_Simple - Cmblock = %ld / %ld\n", Cmblock, NBmblock);
    fflush(stdout);

    if(system("mkdir -p mkmodestmp") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");



    MBLOCK_NBmode = (long*) malloc(sizeof(long)*NBmblock);
    MBLOCK_blockstart = (long*) malloc(sizeof(long)*NBmblock);
    MBLOCK_blockend = (long*) malloc(sizeof(long)*NBmblock);


    IDin = image_ID(IDin_name);
    wfsxsize = data.image[IDin].md[0].size[0];
    wfsysize = data.image[IDin].md[0].size[1];
    wfssize = wfsxsize*wfsysize;
    NBmodes = data.image[IDin].md[0].size[2];

    // read block ends
    if(NBmblock==1)
    {
        MBLOCK_blockend[0] = NBmodes;

        if(sprintf(fname, "./conf_staged/param_block00end.txt") < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        fp = fopen(fname, "w");
        fprintf(fp, "%03ld\n", NBmodes);
        fclose(fp);
    }
    else
    {
        for(mblock=0; mblock<NBmblock; mblock++)
        {
            if(sprintf(fname, "./conf_staged/param_block%02ldend.txt", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            fp = fopen(fname, "r");
            if(fp==NULL)
            {
                printf("ERROR: File \"%s\" not found\n", fname);
                exit(0);
            }
            if(fscanf(fp, "%50ld", &MBLOCK_blockend[mblock]) != 1)
                printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");
            fclose(fp);

            printf("Block end %ld = %ld\n", mblock, MBLOCK_blockend[mblock]);
            fflush(stdout);
        }
    }

    MBLOCK_NBmode[0] = MBLOCK_blockend[0];
    MBLOCK_blockstart[0] = 0;
    for(mblock=1; mblock<NBmblock; mblock++)
    {
        MBLOCK_NBmode[mblock] = MBLOCK_blockend[mblock] - MBLOCK_blockend[mblock-1];
        MBLOCK_blockstart[mblock] =  MBLOCK_blockstart[mblock-1] + MBLOCK_NBmode[mblock-1];
    }




    IDmodes = create_3Dimage_ID("fmodesall", NBmodes, 1, NBmodes);
    for(kk=0; kk<NBmodes*NBmodes; kk++)
        data.image[IDmodes].array.F[kk] = 0.0;
    for(kk=0; kk<NBmodes; kk++)
        data.image[IDmodes].array.F[kk*NBmodes+kk] = 1.0;
    save_fits("fmodesall", "!./mkmodestmp/fmodesall.fits");

    for(mblock=0; mblock<NBmblock; mblock++)
    {
        printf("mblock %02ld  : %ld modes\n", mblock, MBLOCK_NBmode[mblock]);


        if( (Cmblock == mblock) || (Cmblock == -1) )
        {
            printf("Reconstructing block %ld\n", mblock);

            if(sprintf(command, "echo \"%f\" > ./conf_staged/block%02ld_SVDlim.txt", SVDlim, mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");


            IDdmmask = create_2Dimage_ID("dmmask", NBmodes, 1);
            for(kk=0; kk<NBmodes; kk++)
                data.image[IDdmmask].array.F[kk] = 1.0;

            if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            ID = create_3Dimage_ID(imname, wfsxsize, wfsysize, MBLOCK_NBmode[mblock]);
            for(kk=0; kk<MBLOCK_NBmode[mblock]; kk++)
            {
                for(ii=0; ii<wfssize; ii++)
                    data.image[ID].array.F[kk*wfssize+ii] = data.image[IDin].array.F[(kk+MBLOCK_blockstart[mblock])*wfssize+ii];
            }

            if(sprintf(fname, "!./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname, fname);


            if(sprintf(imnameCM, "cmat_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imnameCMc, "cmatc_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imnameCMcact, "cmatcact_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            // COMPUTE MODAL CONTROL MATRICES
            printf("COMPUTE CONTROL MATRIX\n");
#ifdef HAVE_MAGMA
            CUDACOMP_magma_compute_SVDpseudoInverse(imname, imnameCM, SVDlim, 10000, "VTmat", 0);
#else
            linopt_compute_SVDpseudoInverse(imname, imnameCM, SVDlim, 10000, "VTmat");
#endif

            delete_image_ID("VTmat");

            if(sprintf(fname, "!./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imnameCM, fname);

            printf("-- COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX\n");
            fflush(stdout);

            // COMPUTE ZONAL CONTROL MATRIX FROM MODAL CONTROL MATRIX
            if(sprintf(imname, "fmodes_%02ld", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            IDmodes = create_3Dimage_ID(imname, NBmodes, 1, MBLOCK_NBmode[mblock]);
            list_image_ID();
            for(kk=0; kk<MBLOCK_NBmode[mblock]; kk++)
            {
                for(ii=0; ii<NBmodes; ii++)
                    data.image[IDmodes].array.F[kk*NBmodes+ii] = 0.0;
                data.image[IDmodes].array.F[kk*NBmodes+(kk+MBLOCK_blockstart[mblock])] = 1.0;
            }

            if(sprintf(fname, "!./mkmodestmp/fmodes_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname, fname);


            AOloopControl_computeCalib_compute_CombinedControlMatrix(imnameCM, imname, "wfsmask", "dmmask", imnameCMc, imnameCMcact);
            delete_image_ID("dmmask");

            if(sprintf(fname, "!./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imnameCMc, fname);

            if(sprintf(fname, "!./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(sprintf(imname1, "%s_00", imnameCMcact) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            save_fits(imname1, fname);
        }

        else
        {
            printf("LOADING WFS MODES, MODAL CONTROL MATRICES: block %ld\n", mblock);
            fflush(stdout);

            if(sprintf(fname, "./mkmodestmp/fmodesWFS_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imname, 1);

            if(sprintf(fname, "./mkmodestmp/cmat_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCM, 1);

            if(sprintf(fname, "./mkmodestmp/cmatc_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCMc, 1);

            if(sprintf(fname, "./mkmodestmp/cmatcact_%02ld.fits", mblock) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            load_fits(fname, imnameCMcact, 1);
        }
    }


    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
        cnt += MBLOCK_NBmode[mblock];
    IDm = create_3Dimage_ID("fmodesWFSall", wfsxsize, wfsysize, cnt);
    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
    {
        if(sprintf(command, "echo \"%ld\" > ./conf_staged/block%02ld_NBmodes.txt", MBLOCK_NBmode[mblock], mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if(system(command) != 0)
            printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

        if(sprintf(imname, "fmodesWFS_%02ld", mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDmwfs = image_ID(imname);
        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
        {
            for(ii=0; ii<wfssize; ii++)
                data.image[IDm].array.F[cnt*wfssize+ii] = data.image[IDmwfs].array.F[m*wfssize+ii];
            cnt++;
        }
    }
    save_fits("fmodesWFSall", "!./mkmodestmp/fmodesWFSall.fits");


    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
        cnt += MBLOCK_NBmode[mblock];
    IDcmatall = create_3Dimage_ID("cmatall", wfsxsize, wfsysize, cnt);
    cnt = 0;
    for(mblock=0; mblock<NBmblock; mblock++)
    {
        if(sprintf(imname, "cmat_%02ld", mblock) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        long IDcmat = image_ID(imname);
        for(m=0; m<MBLOCK_NBmode[mblock]; m++)
        {
            for(ii=0; ii<wfssize; ii++)
                data.image[IDcmatall].array.F[cnt*wfssize+ii] = data.image[IDcmat].array.F[m*wfssize+ii];
            cnt++;
        }
    }
    save_fits("cmatall", "!./mkmodestmp/cmatall.fits");





    free(MBLOCK_NBmode);
    free(MBLOCK_blockstart);
    free(MBLOCK_blockend);


    return(IDin);
}



/** \brief Computes control matrix using SVD
 *
 *        Conventions:
 * 				m: number of actuators (= NB_MODES);
 * 				n: number of sensors  (= # of pixels)
 *	works even for m != n
 *
 *
 *
 */

int_fast8_t AOloopControl_computeCalib_compute_ControlMatrix(long loop, long NB_MODE_REMOVED, const char *ID_Rmatrix_name, const char *ID_Cmatrix_name, const char *ID_VTmatrix_name, double Beta, long NB_MODE_REMOVED_STEP, float eigenvlim)
{
    FILE *fp;
    long ii1, jj1, k, ii;
    gsl_matrix *matrix_D; /* this is the response matrix */
    gsl_matrix *matrix_Ds; /* this is the pseudo inverse of D */
    gsl_matrix *matrix_Dtra;
    gsl_matrix *matrix_DtraD;
    gsl_matrix *matrix_DtraDinv;
    gsl_matrix *matrix_DtraD_evec;
    gsl_matrix *matrix1;
    gsl_matrix *matrix2;
    gsl_vector *matrix_DtraD_eval;
    gsl_eigen_symmv_workspace *w;

    gsl_matrix *matrix_save;

    long m;
    long n;
    long ID_Rmatrix, ID_Cmatrix, ID_VTmatrix;
    uint32_t *arraysizetmp;

    long IDmodes;

    long IDeigenmodesResp;
    long kk, kk1;
    long ID_RMmask;

    double *CPAcoeff; /// gain applied to modes to enhance low orders in SVD

    char fname[200];
    long NB_MR;  /// number of modes removed

    long NB_MODE_REMOVED1;
    float eigenvmin=0.0;
    long NBMODES_REMOVED_EIGENVLIM = 0;


    long MB_MR_start;
    long MB_MR_end;
    long MB_MR_step;

    int ret;
    char command[200];


    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    arraysizetmp = (uint32_t*) malloc(sizeof(uint32_t)*3);


    ID_Rmatrix = image_ID(ID_Rmatrix_name);


    n = data.image[ID_Rmatrix].md[0].size[0]*data.image[ID_Rmatrix].md[0].size[1]; //AOconf[loop].NBDMmodes;
    m = data.image[ID_Rmatrix].md[0].size[2]; //AOconf[loop].sizeWFS;


    ID_RMmask = image_ID("RMmask");
    if(ID_RMmask!=-1) // apply mask to response matrix
    {
        for(kk=0; kk<m; kk++)
        {
            for(ii=0; ii<n; ii++)
                data.image[ID_Rmatrix].array.F[kk*n+ii] *= data.image[ID_RMmask].array.F[ii];
        }
    }



    /** in this procedure, m=number of actuators/modes, n=number of WFS elements */
    //  long m = smao[0].NBmode;
    // long n = smao[0].NBwfselem;

    printf("m = %ld actuators (modes), n = %ld sensors\n", m, n);
    fflush(stdout);

    NB_MODE_REMOVED1 = m-1;

    matrix_DtraD_eval = gsl_vector_alloc (m);
    matrix_D = gsl_matrix_alloc (n,m);
    matrix_Ds = gsl_matrix_alloc (m,n);
    matrix_Dtra = gsl_matrix_alloc (m,n);
    matrix_DtraD = gsl_matrix_alloc (m,m);
    matrix_DtraDinv = gsl_matrix_alloc (m,m);
    matrix_DtraD_evec = gsl_matrix_alloc (m,m);


    CPAcoeff = (double*) malloc(sizeof(double)*m);

    if(Beta>0.000001)
    {
        long ID = load_fits("modesfreqcpa.fits", "modesfreqcpa", 1);
        if(ID==-1)
        {
            for(k=0; k<m; k++)
                CPAcoeff[k] = 1.0;
        }
        else
        {
            for(k=0; k<m; k++)
            {
                CPAcoeff[k] =  exp(-data.image[ID].array.F[k]*Beta);
                printf("%5ld %5.3f %g\n", k, data.image[ID].array.F[k], CPAcoeff[k]);
            }
        }
    }
    else
    {
        for(k=0; k<m; k++)
            CPAcoeff[k] = 1.0;
    }


    /* write matrix_D */
    for(k=0; k<m; k++)
    {
        for(ii=0; ii<n; ii++)
            gsl_matrix_set (matrix_D, ii, k, data.image[ID_Rmatrix].array.F[k*n+ii]*CPAcoeff[k]);
    }
    /* compute DtraD */
    gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1.0, matrix_D, matrix_D, 0.0, matrix_DtraD);


    /* compute the inverse of DtraD */

    /* first, compute the eigenvalues and eigenvectors */
    w =   gsl_eigen_symmv_alloc (m);
    matrix_save = gsl_matrix_alloc (m,m);
    gsl_matrix_memcpy(matrix_save, matrix_DtraD);
    gsl_eigen_symmv (matrix_save, matrix_DtraD_eval, matrix_DtraD_evec, w);
    gsl_matrix_free(matrix_save);
    gsl_eigen_symmv_free(w);
    gsl_eigen_symmv_sort (matrix_DtraD_eval, matrix_DtraD_evec, GSL_EIGEN_SORT_ABS_DESC);

    printf("Eigenvalues\n");
    fflush(stdout);

    // Write eigenvalues
    if((fp=fopen("eigenv.dat","w"))==NULL)
    {
        printf("ERROR: cannot create file \"eigenv.dat\"\n");
        exit(0);
    }
    for(k=0; k<m; k++)
        fprintf(fp,"%ld %g\n", k, gsl_vector_get(matrix_DtraD_eval,k));
    fclose(fp);

    eigenvmin = eigenvlim*gsl_vector_get(matrix_DtraD_eval,0);

    NBMODES_REMOVED_EIGENVLIM = 0;
    for(k=0; k<m; k++)
    {
        printf("Mode %ld eigenvalue = %g\n", k, gsl_vector_get(matrix_DtraD_eval,k));
        if(gsl_vector_get(matrix_DtraD_eval,k) < eigenvmin)
            NBMODES_REMOVED_EIGENVLIM++;
    }




    /** Write rotation matrix to go from DM modes to eigenmodes */
    arraysizetmp[0] = m;
    arraysizetmp[1] = m;
    ID_VTmatrix = create_image_ID(ID_VTmatrix_name, 2, arraysizetmp, _DATATYPE_FLOAT, 0, 0);
    for(ii=0; ii<m; ii++) // modes
        for(k=0; k<m; k++) // modes
            data.image[ID_VTmatrix].array.F[k*m+ii] = (float) gsl_matrix_get( matrix_DtraD_evec, k, ii);


    /// Compute eigenmodes responses
    IDeigenmodesResp = create_3Dimage_ID("eigenmodesrespM", data.image[ID_Rmatrix].md[0].size[0], data.image[ID_Rmatrix].md[0].size[1], data.image[ID_Rmatrix].md[0].size[2]);
    printf("Computing eigenmode responses .... \n");
    for(kk=0; kk<m; kk++) /// eigen mode index
    {
        printf("\r eigenmode %4ld / %4ld   ", kk, m);
        fflush(stdout);
        for(kk1=0; kk1<m; kk1++)
        {
            for(ii=0; ii<n; ii++)
                data.image[IDeigenmodesResp].array.F[kk*n + ii] += data.image[ID_VTmatrix].array.F[kk1*m+kk]*data.image[ID_Rmatrix].array.F[kk1*n + ii];
        }
    }
    if(sprintf(fname, "!eigenmodesrespM_%4.2f.fits", Beta) < 1)
        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

    save_fits("eigenmodesrespM", fname);
    printf("\n");


    /// if modesM exists, compute eigenmodes using rotation matrix
    IDmodes = image_ID("modesM");
    if(IDmodes!=-1)
    {
        uint32_t xsize_modes, ysize_modes, zsize_modes;
        xsize_modes = data.image[IDmodes].md[0].size[0];
        ysize_modes = data.image[IDmodes].md[0].size[1];
        zsize_modes = data.image[IDmodes].md[0].size[2];
        if(zsize_modes != m)
            printf("ERROR: zsize (%ld) of modesM does not match expected size (%ld)\n", (long) zsize_modes, (long) m);
        else
        {
            long IDeigenmodes = create_3Dimage_ID("eigenmodesM", xsize_modes, ysize_modes, m);
            printf("Computing eigenmodes .... \n");
            for(kk=0; kk<m; kk++) /// eigen mode index
            {
                printf("\r eigenmode %4ld / %4ld   ", kk, m);
                fflush(stdout);
                for(kk1=0; kk1<m; kk1++)
                {
                    for(ii=0; ii<xsize_modes*ysize_modes; ii++)
                        data.image[IDeigenmodes].array.F[kk*xsize_modes*ysize_modes + ii] += data.image[ID_VTmatrix].array.F[kk1*m+kk]*data.image[IDmodes].array.F[kk1*xsize_modes*ysize_modes + ii];
                }
            }
            printf("\n");
        }

        if(sprintf(fname, "!eigenmodesM_%4.2f.fits", Beta) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        save_fits("eigenmodesM", fname);
    }




    /// second, build the "inverse" of the diagonal matrix of eigenvalues (matrix1)
    matrix1 = gsl_matrix_alloc (m, m);
    matrix2 = gsl_matrix_alloc (m, m);
    arraysizetmp[0] = AOconf[loop].sizexWFS;
    arraysizetmp[1] = AOconf[loop].sizeyWFS;
    arraysizetmp[2] = m;
    ID_Cmatrix = create_image_ID(ID_Cmatrix_name, 3, arraysizetmp, _DATATYPE_FLOAT, 0, 0);

    printf("COMPUTING CMAT .... \n");


    if(NB_MODE_REMOVED_STEP==0)
    {
        MB_MR_start = NBMODES_REMOVED_EIGENVLIM;
        MB_MR_end = NBMODES_REMOVED_EIGENVLIM+1;
        MB_MR_step = 10;
    }
    else
    {
        MB_MR_start = 0;
        MB_MR_end = NB_MODE_REMOVED1;
        MB_MR_step = NB_MODE_REMOVED_STEP;
    }

    for(NB_MR=MB_MR_start; NB_MR<MB_MR_end; NB_MR+=MB_MR_step)
    {
        printf("\r Number of modes removed : %5ld / %5ld  (step %ld)  ", NB_MR, NB_MODE_REMOVED1, NB_MODE_REMOVED_STEP);
        fflush(stdout);
        for(ii1=0; ii1<m; ii1++)
            for(jj1=0; jj1<m; jj1++)
            {
                if(ii1==jj1)
                {
                    if((m-ii1-1)<NB_MR)
                        gsl_matrix_set(matrix1, ii1, jj1, 0.0);
                    else
                        gsl_matrix_set(matrix1, ii1, jj1, 1.0/gsl_vector_get(matrix_DtraD_eval,ii1));
                }
                else
                    gsl_matrix_set(matrix1, ii1, jj1, 0.0);
            }


        /* third, compute the "inverse" of DtraD */
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, matrix_DtraD_evec, matrix1, 0.0, matrix2);
        gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, matrix2, matrix_DtraD_evec, 0.0, matrix_DtraDinv);
        gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, matrix_DtraDinv, matrix_D, 0.0, matrix_Ds);

        /* write result */
        printf("write result to ID %ld   [%ld %ld]\n", ID_Cmatrix, n, m);
        fflush(stdout);

        for(ii=0; ii<n; ii++) // sensors
            for(k=0; k<m; k++) // actuator modes
                data.image[ID_Cmatrix].array.F[k*n+ii] = (float) gsl_matrix_get(matrix_Ds, k, ii)*CPAcoeff[k];


        if(NB_MODE_REMOVED_STEP==0)
        {
            save_fits(ID_Cmatrix_name, "!cmat.fits");

            if(sprintf(command, "echo \"%ld\" > ./cmat.NB_MODES_RM.txt", NBMODES_REMOVED_EIGENVLIM) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

            if(sprintf(command, "echo \"%ld\" > ./cmat.NB_MODES.txt",  m) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            if(system(command) != 0)
                printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");
        }
        else
        {
            if(sprintf(fname, "!cmat_%4.2f_%02ld.fits", Beta, NB_MR) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            printf("  SAVING -> %s\n", fname);
            fflush(stdout);
            save_fits(ID_Cmatrix_name, fname);
        }
    }

    printf("\n\n");

    gsl_matrix_free(matrix1);
    gsl_matrix_free(matrix2);

    gsl_vector_free(matrix_DtraD_eval);
    gsl_matrix_free(matrix_D);
    gsl_matrix_free(matrix_Ds);
    gsl_matrix_free(matrix_Dtra);
    gsl_matrix_free(matrix_DtraD);
    gsl_matrix_free(matrix_DtraDinv);
    gsl_matrix_free(matrix_DtraD_evec);

    free(arraysizetmp);

    free(CPAcoeff);



    return(ID_Cmatrix);
}





//
// computes combined control matrix
//
//

long AOloopControl_computeCalib_compute_CombinedControlMatrix(const char *IDcmat_name, const char *IDmodes_name, const char* IDwfsmask_name, const char *IDdmmask_name, const char *IDcmatc_name, const char *IDcmatc_active_name)
{
    // long ID;
    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    double tdiffv;

    float *matrix_cmp;
    long wfselem, act, mode;
    //    long n_sizeDM, n_NBDMmodes, n_sizeWFS;
    float *matrix_Mc, *matrix_DMmodes;
    long act_active, wfselem_active;

    long IDwfsmask, IDdmmask;
    long sizexWFS, sizeyWFS, sizeWFS, sizeWFS_active[100];
    long ii, ii1;
    long sizexDM, sizeyDM;
    long sizeDM_active;
    uint32_t *sizearray;
    long IDcmat;
    long IDcmatc;
    long IDmodes;
    long NBDMmodes;
    long sizeDM;
    long IDcmatc_active[100];
    char name[200];
    char imname[200];
    int slice;


    printf("COMPUTING COMBINED CONTROL MATRIX .... \n");
    fflush(stdout);

    clock_gettime(CLOCK_REALTIME, &t1);


    // initialize size of arrays
    IDwfsmask = image_ID(IDwfsmask_name);
    sizexWFS = data.image[IDwfsmask].md[0].size[0];
    sizeyWFS = data.image[IDwfsmask].md[0].size[1];
    sizeWFS = sizexWFS*sizeyWFS;

    printf("IDwfsmask = %ld\n", IDwfsmask);
    fflush(stdout);

    IDdmmask = image_ID(IDdmmask_name);
    sizexDM = data.image[IDdmmask].md[0].size[0];
    sizeyDM = data.image[IDdmmask].md[0].size[1];
    sizeDM = sizexDM*sizeyDM;

    printf("IDdmmask = %ld\n", IDdmmask);
    fflush(stdout);

    IDmodes = image_ID(IDmodes_name);

    printf("IDmodes = %ld\n", IDmodes);
    fflush(stdout);

    NBDMmodes = data.image[IDmodes].md[0].size[2];



    // allocate array for combined matrix
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);
    sizearray[0] = sizexWFS;
    sizearray[1] = sizeyWFS;
    sizearray[2] = sizeDM;

    printf("Creating 3D image : %ld %ld %ld\n", sizexWFS, sizeyWFS, sizeDM);
    fflush(stdout);
    IDcmatc = create_image_ID(IDcmatc_name, 3, sizearray, _DATATYPE_FLOAT, 0, 0);
    free(sizearray);


    printf("PREPARE MATRIX MULT\n");
    fflush(stdout);



    // init matrix_Mc
    matrix_Mc = (float*) malloc(sizeof(float)*sizeWFS*sizeDM);
    memcpy(matrix_Mc, data.image[IDcmatc].array.F, sizeof(float)*sizeWFS*sizeDM);

    // copy modal control matrix to matrix_cmp
    IDcmat = image_ID(IDcmat_name);
    matrix_cmp = (float*) malloc(sizeof(float)*sizeWFS*NBDMmodes);
    memcpy(matrix_cmp, data.image[IDcmat].array.F, sizeof(float)*sizeWFS*NBDMmodes);

    // copy modes matrix to matrix_DMmodes
    matrix_DMmodes = (float*) malloc(sizeof(float)*NBDMmodes*sizeDM);
    memcpy(matrix_DMmodes, data.image[IDmodes].array.F, sizeof(float)*NBDMmodes*sizeDM);

    printf("START MATRIX MULT\n");
    fflush(stdout);



    // computing combine matrix (full size)
    //# ifdef _OPENMP
    //   #pragma omp parallel shared(matrix_Mc, matrix_cmp, matrix_DMmodes ,chunk) private( mode, act, wfselem)
    //  {
    //        #pragma omp for schedule (static)
    //# endif
    for(mode=0; mode<NBDMmodes; mode++)
    {
        for(act=0; act<sizeDM; act++)
        {
            for(wfselem=0; wfselem<sizeWFS; wfselem++)
                matrix_Mc[act*sizeWFS+wfselem] += matrix_cmp[mode*sizeWFS+wfselem]*matrix_DMmodes[mode*sizeDM+act];
        }
    }
    //# ifdef _OPENMP
    //    }
    //# endif
    memcpy(data.image[IDcmatc].array.F, matrix_Mc, sizeof(float)*sizeWFS*sizeDM);
    free(matrix_cmp);

    printf("REDUCE MATRIX SIZE\n");
    fflush(stdout);



    WFS_active_map = (int*) malloc(sizeof(int)*sizeWFS*PIXSTREAM_NBSLICES);
    for(slice=0; slice<PIXSTREAM_NBSLICES; slice++)
    {
        ii1 = 0;
        for(ii=0; ii<sizeWFS; ii++)
            if(data.image[IDwfsmask].array.F[ii]>0.1)
            {
                if(slice==0)
                {
                    WFS_active_map[ii1] = ii;
                    ii1++;
                }
                else
                {
                    if(data.image[aoconfID_pixstream_wfspixindex].array.UI16[ii]==slice+1)
                    {
                        WFS_active_map[slice*sizeWFS+ii1] = ii;
                        ii1++;
                    }
                }
            }
        sizeWFS_active[slice] = ii1;

        if(sprintf(imname, "aol%ld_imWFS2active_%02d", LOOPNUMBER, slice) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        /* CAN CRASH
                sizearray = (long*) malloc(sizeof(long)*2);
                sizearray[0] =  sizeWFS_active[slice];
                sizearray[1] =  1;
                aoconfID_imWFS2_active[slice] = create_image_ID(imname, 2, sizearray, FLOAT, 1, 0);
        		free(sizearray);
        */
    }



    DM_active_map = (int*) malloc(sizeof(int)*sizeDM);
    ii1 = 0;
    for(ii=0; ii<sizeDM; ii++)
        if(data.image[IDdmmask].array.F[ii]>0.1)
        {
            DM_active_map[ii1] = ii;
            ii1++;
        }
    sizeDM_active = ii1;
    //   aoconfID_meas_act_active = create_2Dimage_ID("meas_act_active", sizeDM_active, 1);

    /* CAN CRASH
        sizearray = (long*) malloc(sizeof(long)*2);
        sizearray[0] = sizeDM_active;
        sizearray[1] = 1;
        sprintf(name, "aol%ld_meas_act_active", LOOPNUMBER);
        aoconfID_meas_act_active = create_image_ID(name, 2, sizearray, FLOAT, 1, 0);
       free(sizearray);
    */





    // reduce matrix size to active elements
    for(slice=0; slice<PIXSTREAM_NBSLICES; slice++)
    {
        if(sprintf(imname, "%s_%02d", IDcmatc_active_name, slice) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        IDcmatc_active[slice] = create_2Dimage_ID(imname, sizeWFS_active[slice], sizeDM_active);
        for(act_active=0; act_active<sizeDM_active; act_active++)
        {
            for(wfselem_active=0; wfselem_active<sizeWFS_active[slice]; wfselem_active++)
            {
                act = DM_active_map[act_active];
                wfselem = WFS_active_map[slice*sizeWFS+wfselem_active];
                data.image[IDcmatc_active[slice]].array.F[act_active*sizeWFS_active[slice]+wfselem_active] = matrix_Mc[act*sizeWFS+wfselem];
            }
        }
        printf("PIXEL SLICE %d     Keeping only active pixels / actuators : %ld x %ld   ->   %ld x %ld\n", slice, sizeWFS, sizeDM, sizeWFS_active[slice], sizeDM_active);


    }

    free(matrix_Mc);
    free(matrix_DMmodes);



    clock_gettime(CLOCK_REALTIME, &t2);
    tdiff = info_time_diff(t1, t2);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    printf("\n");
    printf("TIME TO COMPUTE MATRIX = %f sec\n", tdiffv);

    return(0);
}






long AOloopControl_computeCalib_loadCM(long loop, const char *CMfname)
{
    long ID = -1;



    if(AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(0);

    if( (ID = load_fits(CMfname, "tmpcontrM", 1)) != -1 )
    {

        // check size is OK
        int vOK = 1;


        if(data.image[ID].md[0].naxis!=3)
        {
            printf("Control matrix has wrong dimension\n");
            vOK = 0;
        }
        if(data.image[ID].md[0].atype!=_DATATYPE_FLOAT)
        {
            printf("Control matrix has wrong type\n");
            vOK = 0;
        }
        if(vOK==1)
        {
            if(data.image[ID].md[0].size[0]!=AOconf[loop].sizexWFS)
            {
                printf("Control matrix has wrong x size : is %ld, should be %ld\n", (long) data.image[ID].md[0].size[0], (long) AOconf[loop].sizexWFS);
                vOK = 0;
            }
            if(data.image[ID].md[0].size[1]!=AOconf[loop].sizeyWFS)
            {
                printf("Control matrix has wrong y size\n");
                vOK = 0;
            }
            if(data.image[ID].md[0].size[2]!=AOconf[loop].NBDMmodes)
            {
                printf("Control matrix has wrong z size\n");
                vOK = 0;
            }
        }


        if(vOK==1)
        {
            AOconf[loop].init_CM = 1;
            char name[200];
            if(sprintf(name, "ContrM_%ld", loop) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

            ID = image_ID(name);
            if(ID==-1)
                ID = read_sharedmem_image(name);
            long ID0 = image_ID("tmpcontrM");
            data.image[ID].md[0].write  = 1;
            long ii;
            for(ii=0; ii<AOconf[loop].sizexWFS*AOconf[loop].sizeyWFS*AOconf[loop].NBDMmodes; ii++)
                data.image[ID].array.F[ii] = data.image[ID0].array.F[ii];
            data.image[ID].md[0].write  = 0;
            data.image[ID].md[0].cnt0++;
        }
        delete_image_ID("tmpcontrM");
    }

    return(ID);
}














