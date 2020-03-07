/**
 * @file    AOloopControl_computeCalib_processRM.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
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



extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c

long aoconfID_imWFS2_active[100];







//
// if images "Hmat" AND "pixindexim" are provided, decode the image
// TEST: if "RMpokeC" exists, decode it as well
//
errno_t AOloopControl_computeCalib_Process_zrespM(
    long        loop,
    const char *IDzrespm0_name,
    __attribute__((unused)) const char *IDwfsref_name,
    const char *IDzrespm_name,
    const char *WFSmap_name,
    const char *DMmap_name
)
{
    imageID  IDzrm;
    uint64_t sizeWFS;
    char     name[200];

    double rms, tmpv;
    imageID IDDMmap, IDWFSmap, IDdm;



    // DECODE MAPS (IF REQUIRED)
    IDzrm = image_ID(IDzrespm0_name);
    if((image_ID("RMmat")!=-1) && (image_ID("pixindexim")!=-1))  // start decoding
    {
        save_fits(IDzrespm0_name, "!./tmpRMacqu/zrespm_Hadamard.fits");

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

    uint32_t sizexWFS = data.image[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = data.image[IDzrm].md[0].size[1];
    uint32_t NBpoke = data.image[IDzrm].md[0].size[2];

    if(sprintf(name, "aol%ld_dmC", loop) < 1)
        print_ERROR("sprintf wrote <1 char");

    IDdm = read_sharedmem_image(name);
    uint32_t sizexDM = data.image[IDdm].md[0].size[0];
    uint32_t sizeyDM = data.image[IDdm].md[0].size[1];

    sizeWFS = sizexWFS*sizeyWFS;

    IDWFSmap = create_2Dimage_ID(WFSmap_name, sizexWFS, sizeyWFS);
    IDDMmap = create_2Dimage_ID(DMmap_name, sizexDM, sizeyDM);


    printf("Preparing DM map ... ");
    fflush(stdout);
    for(uint32_t poke=0; poke<NBpoke; poke++)
    {
        rms = 0.0;
        for(uint32_t ii=0; ii<sizeWFS; ii++)
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
    for(uint32_t ii=0; ii<sizeWFS; ii++)
    {
        rms = 0.0;
        for(uint32_t poke=0; poke<NBpoke; poke++)
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

	return RETURN_SUCCESS;
}








// CANDIDATE FOR RETIREMENT
//
// median-averages multiple response matrices to create a better one
//
// if images "Hmat" AND "pixindexim" are provided, decode the image
// TEST: if "RMpokeC" exists, decode it as well
//
errno_t AOloopControl_computeCalib_ProcessZrespM_medianfilt(
    long        loop,
    const char *zrespm_name,
    const char *WFSref0_name,
    const char *WFSmap_name,
    const char *DMmap_name,
    double      rmampl,
    int         normalize
)
{
    long NBmat; // number of matrices to average
    FILE *fp;
    //int r;
    char name[200];
    char fname[200];
    char zrname[200];
    long kmat;
    long sizexWFS, sizeyWFS, sizeWFS;
    imageID *IDzresp_array;
    long ii;
    double fluxpos, fluxneg;
    float *pixvalarray;
    long k, kmin, kmax, kband;
    imageID IDzrm;
    float ave;
    imageID *IDWFSrefc_array;
    imageID IDWFSref;
    //imageID IDWFSmap;
    //imageID IDDMmap;
    imageID IDWFSmask;
    //imageID IDDMmask;
//    float lim, rms;
//    double tmpv;
    long NBmatlim = 3;
    long NBpoke, poke;
    double tot, totm;


    if(sprintf(fname, "./zresptmp/%s_nbiter.txt", zrespm_name) < 1)
        print_ERROR("sprintf wrote <1 char");

    if((fp = fopen(fname, "r"))==NULL)
    {
        printf("ERROR: cannot open file \"%s\"\n", fname);
        exit(0);
    }
    else
    {
        if(fscanf(fp, "%50ld", &NBmat) != 1)
            print_ERROR("Cannot read parameter for file");

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
        print_ERROR("sprintf wrote <1 char");


    IDzresp_array = (imageID*) malloc(sizeof(imageID)*NBmat);
    IDWFSrefc_array = (imageID*) malloc(sizeof(imageID)*NBmat);

    // STEP 1: build individually cleaned RM
    for(kmat=0; kmat<NBmat; kmat++)
    {
        if(sprintf(fname, "./zresptmp/%s_pos_%03ld.fits", zrespm_name, kmat) < 1)
            print_ERROR("sprintf wrote <1 char");

        long IDzrespfp = load_fits(fname, "zrespfp", 2);

        if(sprintf(fname, "./zresptmp/%s_neg_%03ld.fits", zrespm_name, kmat) < 1)
            print_ERROR("sprintf wrote <1 char");

        long IDzrespfm = load_fits(fname, "zrespfm", 2);

        sizexWFS = data.image[IDzrespfp].md[0].size[0];
        sizeyWFS = data.image[IDzrespfp].md[0].size[1];
        NBpoke = data.image[IDzrespfp].md[0].size[2];
        sizeWFS = sizexWFS*sizeyWFS;

        if(sprintf(name, "wfsrefc%03ld", kmat) < 1)
            print_ERROR("sprintf wrote <1 char");

        IDWFSrefc_array[kmat] = create_3Dimage_ID(name, sizexWFS, sizeyWFS, NBpoke);

        if(sprintf(zrname, "zrespm%03ld", kmat) < 1)
            print_ERROR("sprintf wrote <1 char");

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
                data.image[IDzresp_array[kmat]].array.F[poke*sizeWFS+ii] = 
                0.5*(data.image[IDzrespfp].array.F[poke*sizeWFS+ii]-data.image[IDzrespfm].array.F[poke*sizeWFS+ii]);
                data.image[IDWFSrefc_array[kmat]].array.F[poke*sizeWFS+ii] = 
                0.5*(data.image[IDzrespfp].array.F[poke*sizeWFS+ii]+data.image[IDzrespfm].array.F[poke*sizeWFS+ii]);

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


    AOloopControl_computeCalib_mkCalib_map_mask(
    loop, zrespm_name, WFSmap_name, DMmap_name, 
    0.2, 1.0, 0.7, 0.3, 0.05, 1.0, 0.65, 0.3);

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


    return RETURN_SUCCESS;
}





errno_t AOloopControl_computeCalib_mkCM_FPCONF(
    char *fpsname,
    uint32_t CMDmode
)
{
    //uint16_t loopstatus;

    // ===========================
    // SETUP FPS
    // ===========================
    /*int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps = function_parameter_FPCONFsetup(fpsname, CMDmode, &loopstatus, &SMfd);
    strncpy(fps.md->sourcefname, __FILE__, FPS_SRCDIR_STRLENMAX);
    fps.md->sourceline = __LINE__;
    */

    FPS_SETUP_INIT(fpsname, CMDmode);

    // ===========================
    // ALLOCATE FPS ENTRIES
    // ===========================

    void *pNull = NULL;
    uint64_t FPFLAG;

    long loop_default[4] = { 0, 0, 10, 0 };
    __attribute__((unused)) long fpi_loop =
        function_parameter_add_entry(&fps, ".loop",
                                     "loop index",
                                     FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, &loop_default);


    double SVDlimdefault[4] = { 0.001, 0.0, 1.0, 0.001 };
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_MINLIMIT | FPFLAG_MAXLIMIT;  // required to enforce the min and max limits
    __attribute__((unused)) long fpi_SVDlim =
        function_parameter_add_entry(&fps, ".SVDlim", "SVD limit value",
                                     FPTYPE_FLOAT64, FPFLAG, &SVDlimdefault);

    // Input file name
    FPFLAG = FPFLAG_DEFAULT_INPUT | FPFLAG_FILE_RUN_REQUIRED;
    __attribute__((unused)) long fpi_filename_respm        =
        function_parameter_add_entry(&fps, ".fname_respM", "response matrix",
                                     FPTYPE_FILENAME, FPFLAG, pNull);

    // Output file name
    FPFLAG = FPFLAG_DEFAULT_OUTPUT;
    __attribute__((unused)) long fpi_filename_out         =
        function_parameter_add_entry(&fps, ".outfname", "output name",
                                     FPTYPE_FILENAME, FPFLAG, pNull);



    // Actions
    long fpi_adoptCM = function_parameter_add_entry(&fps, ".adoptCM",
                       "adopt CM",
                       FPTYPE_ONOFF, FPFLAG_DEFAULT_INPUT, pNull);




    // =====================================
    // PARAMETER LOGIC AND UPDATE LOOP
    // =====================================

    while ( fps.loopstatus == 1 )
    {
        usleep(50);
        if( function_parameter_FPCONFloopstep(&fps) == 1) // Apply logic if update is needed
        {
            // here goes the logic
            //
            // Action: adopt CM
            //
            if(fps.parray[fpi_adoptCM].fpflag & FPFLAG_ONOFF) {
                FILE *fpconf;
                char conffname[200];
                long loop = functionparameter_GetParamValue_INT64(&fps, ".loop");
                sprintf(conffname, "conf/shmim.aol%ld_CMat.fname.txt", loop);
                fpconf = fopen(conffname, "w");
                fprintf(fpconf, "%s", functionparameter_GetParamPtr_STRING(&fps, ".outfname") );
                fclose(fpconf);

                // set back to OFF
                fps.parray[fpi_adoptCM].fpflag &= ~FPFLAG_ONOFF;

            }


            functionparameter_CheckParametersAll(&fps);  // check all parameter values
        }

    }



    function_parameter_FPCONFexit( &fps );

    return RETURN_SUCCESS;
}




errno_t AOloopControl_computeCalib_mkCM_RUN(
    char *fpsname
)
{


    FPS_CONNECT( fpsname, FPSCONNECT_RUN );



    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================

    __attribute__((unused)) long loop        =
        functionparameter_GetParamValue_INT64(&fps, ".loop");

    float SVDlim = functionparameter_GetParamValue_FLOAT64(&fps, ".SVDlim");

    char respMname[FUNCTION_PARAMETER_STRMAXLEN+1];
    strncpy(respMname, functionparameter_GetParamPtr_STRING(&fps, ".fname_respM"), FUNCTION_PARAMETER_STRMAXLEN);




    load_fits(respMname, "respM", 1);

    char cm_name[] = "sCMat";
#ifdef HAVE_MAGMA
    CUDACOMP_magma_compute_SVDpseudoInverse("respM", cm_name, SVDlim, 100000, "VTmat", 0, 0, 1.e-4, 1.e-7, 0);
#else
    linopt_compute_SVDpseudoInverse("respM", cm_name, SVDlim, 10000, "VTmat");
#endif

    //save_fits("VTmat", "!./mkmodestmp/VTmat.fits");
    delete_image_ID("VTmat");




    // write output
    //char stagedir[] = "conf_fCM_staged";

    // Get time
    time_t tnow;
    struct tm *tmnow;
    char datestring[200];

    time(&tnow);
    tmnow = gmtime(&tnow);

    printf("TIMESTRING:  %04d %02d %02d  %02d:%02d:%02d\n", 1900+tmnow->tm_year, tmnow->tm_mon, tmnow->tm_mday, tmnow->tm_hour, tmnow->tm_min, tmnow->tm_sec);
    sprintf(datestring, "%04d-%02d-%02d_%02d:%02d:%02d", 1900+tmnow->tm_year, tmnow->tm_mon, tmnow->tm_mday, tmnow->tm_hour, tmnow->tm_min, tmnow->tm_sec);

    char fnamedest[500];
    sprintf(fnamedest, "sCM/sCM_%s.fits", datestring);

    save_fits(cm_name, fnamedest);
    functionparameter_SetParamValue_STRING(&fps, ".outfname", fnamedest);


    delete_image_ID(cm_name);

    function_parameter_RUNexit( &fps );


    return RETURN_SUCCESS;
}






// make control matrix
//
errno_t AOloopControl_computeCalib_mkCM(
    __attribute__((unused)) const char *respm_name,
    float       SVDlim
)
{
    char fpsname[200];

    long pindex = (long) getpid();  // index used to differentiate multiple calls to function
    // if we don't have anything more informative, we use PID

	//int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps;

    // create FPS
    sprintf(fpsname, "compsCM-%06ld", pindex);
    AOloopControl_computeCalib_mkCM_FPCONF(fpsname, CMDCODE_FPSINIT);

    function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_SIMPLE);

    functionparameter_SetParamValue_FLOAT64(&fps, ".SVDlim", SVDlim);


    function_parameter_struct_disconnect(&fps);




    AOloopControl_computeCalib_mkCM_RUN(fpsname);

    return RETURN_SUCCESS;
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
