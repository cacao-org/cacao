/**
 * @file    AOloopControl_IOtools_load_image_sharedmem.c
 * @brief   Adaptive Optics Control loop engine I/O tools
 * 
 * AO engine uses stream data structure
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

#include <string.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_IOtools/AOloopControl_IOtools.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "00CORE/00CORE.h"

/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif


/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */


//extern long aoloopcontrol_var.AOcontrolNBtimers;           // declared in AOloopControl.c

//extern long aoloopcontrol_var.aoconfID_wfsim;              // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS0;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS0tot;          // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_imWFS1;             // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_wfsdark;            // declared in AOloopControl.c
//extern long aoloopcontrol_var.aoconfID_wfsmask;            // declared in AOloopControl.c

//extern uint8_t aoloopcontrol_var.WFSatype;                 // declared in AOloopControl.c

//extern long aoloopcontrol_var.aoconfID_looptiming;         // declared in AOloopControl.c



static sem_t AOLCOMPUTE_TOTAL_ASYNC_sem_name;

static long long imtotalcnt;
static int AOLCOMPUTE_DARK_SUBTRACT_THREADinit = 0;
static int COMPUTE_DARK_SUBTRACT_NBTHREADS = 1;
static sem_t AOLCOMPUTE_DARK_SUBTRACT_sem_name[32];
static sem_t AOLCOMPUTE_DARK_SUBTRACT_RESULT_sem_name[32];


static int avcamarraysInit = 0;
static unsigned short *arrayutmp;

static char Average_cam_frames_dname[200];
static long Average_cam_frames_IDdark = -1;
static long Average_cam_frames_nelem = 1;


static float *arrayftmp;


// TIMING
static struct timespec tnow;
static struct timespec tdiff;
static double tdiffv;

//extern int aoloopcontrol_var.PIXSTREAM_SLICE;

static long ti; // thread index

static int AOLCOMPUTE_TOTAL_ASYNC_THREADinit = 0;
static int AOLCOMPUTE_TOTAL_INIT = 0; // toggles to 1 AFTER total for first image is computed


//extern float aoloopcontrol_var.normfloorcoeff;


//extern float aoloopcontrol_var.GPU_alpha;
//extern float aoloopcontrol_var.GPU_beta;








/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools - 2. LOAD DATA STREAMS     
 *  Load 2D and 3D shared memory images */
/* =============================================================================================== */
/* =============================================================================================== */




long AOloopControl_IOtools_2Dloadcreate_shmim(
    const char *name,
    const char *fname,
    long xsize,
    long ysize,
    float DefaultValue
) {
    long ID;
    int CreateSMim = 0;
    int sizeOK;
    uint32_t *sizearray;
    long ii;

    int loadcreatestatus = -1;
    // value of loadcreatestatus :
    // 0 : existing stream has wrong size -> recreating stream
    // 1 : new stream created and content loaded
    // 2 : existing stream updated
    // 3 : FITS image <fname> has wrong size -> do nothing
    // 4 : FITS image <fname> does not exist, stream <name> exists -> do nothing
    // 5 : FITS image <fname> does not exist, stream <name> does not exist -> create empty stream


    printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
    fflush(stdout);

#ifdef AOLOOPCONTROL_LOGFUNC
    AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall(AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    ID = image_ID(name);
    sizearray = (uint32_t *) malloc(sizeof(uint32_t) * 2);

    if(ID == -1) { // if <name> is not loaded in memory
        printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
        fflush(stdout);

        CreateSMim = 0;
        ID = read_sharedmem_image(name);
        printf("------------- ID = %ld\n", (long) ID);

        if(ID != -1) { // ... and <name> does not exist as a memory stream
            list_image_ID();

            printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
            fflush(stdout);

            sizeOK = COREMOD_MEMORY_check_2Dsize(name, xsize, ysize);

            printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
            fflush(stdout);

            if(sizeOK == 0) { // if size is different, delete stream -> create new one
                char command[500];

                printf("\n========== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s ===========\n\n", name, name);
                delete_image_ID(name);

                if(sprintf(command, "rm %s/%s.im.shm", data.shmdir, name) < 1) {
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
                }

                if(system(command) != 0) {
                    printERROR(__FILE__, __func__, __LINE__, "system() error");
                }

                CreateSMim = 1;
                loadcreatestatus = 0;
            }
        } else { //  ... and <name> does not exist as a stream -> create new stream
            CreateSMim = 1;
            loadcreatestatus = 1;
        }

        printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
        fflush(stdout);

        if(CreateSMim == 1) {
            sizearray[0] =  xsize;
            sizearray[1] =  ysize;
            if(xsize * ysize > 0) {
                ID = create_image_ID(name, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            }
            for(ii = 0; ii < xsize * ysize; ii++) {
                data.image[ID].array.F[ii] = DefaultValue;
            }
        }
    }
    free(sizearray);

    printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
    fflush(stdout);


    if(ID == -1) {
        printf("ERROR: could not load/create %s\n", name);
        printf("Function %s\n", __func__);
        printf("INPUT : \n");
        printf("   name         = \"%s\"\n", name);
        printf("   fname        = \"%s\"\n", fname);
        printf("   xsize        = %ld\n", xsize);
        printf("   ysize        = %ld\n", ysize);
        printf("   DefaultValue = %f\n", DefaultValue);
        printf("\n");
        exit(0);
    } else {
        long ID1;

        ID1 = load_fits(fname, "tmp2Dim", 3);

        if(ID1 != -1) {
            sizeOK = COREMOD_MEMORY_check_2Dsize("tmp2Dim", xsize, ysize);
            if(sizeOK == 1) {
                memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize * ysize);
                printf("loaded file \"%s\" to shared memory \"%s\"\n", fname, name);
                loadcreatestatus = 2;
            } else {
                printf("File \"%s\" has wrong size (should be 2-D %ld x %ld,  is %d-D %ld x %ld): ignoring\n", fname, xsize, ysize, (int) data.image[ID1].md[0].naxis, (long) data.image[ID1].md[0].size[0], (long) data.image[ID1].md[0].size[1]);
                loadcreatestatus = 3;
            }
            delete_image_ID("tmp2Dim");
        } else {
            if(CreateSMim == 0) {
                loadcreatestatus = 4;
            } else {
                loadcreatestatus = 5;
            }
        }
    }


    printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
    fflush(stdout);

    // logging


    if(loadcreateshm_log == 1) { // results should be logged in ASCII file
        switch(loadcreatestatus) {
            case 0 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream has wrong size -> recreating stream\n", fname, name);
                break;
            case 1 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: new stream created and content loaded\n", fname, name);
                break;
            case 2 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream updated\n", fname, name);
                break;
            case 3 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image has wrong size -> do nothing\n", fname, name);
                break;
            case 4 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream exists -> do nothing\n", fname, name);
                break;
            case 5 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream does not exist -> create empty stream\n", fname, name);
                break;
            default:
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: UNKNOWN ERROR CODE\n", fname, name);
                break;
        }
    }


#ifdef AOLOOPCONTROL_LOGFUNC
    AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall(AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    printf("%5d   %s   %s <-> %s  exit status = %d\n", __LINE__, __FUNCTION__, name, fname, loadcreatestatus);
    fflush(stdout);


    return ID;
}








long AOloopControl_IOtools_3Dloadcreate_shmim(const char *name, const char *fname, long xsize, long ysize, long zsize, float DefaultValue) {
    long ID;
    int CreateSMim;
    int sizeOK;
    uint32_t *sizearray;
    long ID1;
    int creashmimfromFITS = 0;
    long ii;

    int loadcreatestatus = -1;
    // value of loadcreatestatus :
    // 0 : existing stream has wrong size -> recreating stream
    // 1 : new stream created and content loaded
    // 2 : existing stream updated
    // 3 : FITS image <fname> has wrong size -> do nothing
    // 4 : FITS image <fname> does not exist, stream <name> exists -> do nothing
    // 5 : FITS image <fname> does not exist, stream <name> does not exist -> create empty stream
    // 6 : stream exists, size is correct



#ifdef AOLOOPCONTROL_LOGFUNC
    AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall(AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif


    printf("-------- ENTERING AOloopControl_3Dloadcreate_shmim   name = %s ----------\n", name);
    fflush(stdout);

    list_image_ID();


    ID = image_ID(name);
    sizearray = (uint32_t *) malloc(sizeof(uint32_t) * 3);

    printf("        ENTERING AOloopControl_3Dloadcreate_shmim: ============== %ld  %ld  %ld ===== %ld ======\n", xsize, ysize, zsize, ID);
    fflush(stdout);

    if(ID == -1) {
        CreateSMim = 0;
        ID = read_sharedmem_image(name);

        printf("        AOloopControl_3Dloadcreate_shmim: ============== %ld  ======\n", ID);
        fflush(stdout);


        if(ID != -1) { // stream exists

            sizeOK = COREMOD_MEMORY_check_3Dsize(name, xsize, ysize, zsize);
            if(sizeOK == 0) {
                char command[500];

                //               printf("\n========== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s ===========\n\n", name, name);
                printf("        AOloopControl_3Dloadcreate_shmim: ===== EXISTING %s HAS WRONG SIZE -> CREATING BLANK %s\n", name, name);
                fflush(stdout);

                delete_image_ID(name);

                if(sprintf(command, "rm %s/%s.im.shm", data.shmdir, name) < 1) {
                    printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
                }

                if(system(command) != 0) {
                    printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");
                }

                CreateSMim = 1;
                loadcreatestatus = 0;
            } else { // SIZE OK
                printf("        AOloopControl_3Dloadcreate_shmim: ===== SIZE OK\n");
                fflush(stdout);
                CreateSMim = 0;
                loadcreatestatus = 2;
            }
        } else {
            CreateSMim = 1;
            loadcreatestatus = 1;
        }

        if(CreateSMim == 1) {
            sizearray[0] = xsize;
            sizearray[1] = ysize;
            sizearray[2] = zsize;
            if(xsize * ysize * zsize > 0) {
                printf("        AOloopControl_3Dloadcreate_shmim: ===== create_image_ID\n");
                fflush(stdout);
                ID = create_image_ID(name, 3, sizearray, _DATATYPE_FLOAT, 1, 0);
                for(ii = 0; ii < xsize * ysize * zsize; ii++) {
                    data.image[ID].array.F[ii] = DefaultValue;
                }
                creashmimfromFITS = 0;
            } else {
                creashmimfromFITS = 1;
            }
        }
    }
    free(sizearray);


    printf("        AOloopControl_3Dloadcreate_shmim: ===== TEST pt\n");
    fflush(stdout);

    // here, ID is either loaded, or it should be created from FITS image
    if((ID == -1) && (creashmimfromFITS == 0)) {
        printf("ERROR: could not load/create %s\n", name);
        exit(0);
    }

    ID1 = load_fits(fname, "tmp3Dim", 3);
    printf("        AOloopControl_3Dloadcreate_shmim: ===== ID1 = %ld\n", ID1);
    fflush(stdout);
    if(ID1 != -1) {
        if(creashmimfromFITS == 1) { // create shared mem from FITS
            sizeOK = COREMOD_MEMORY_check_3Dsize("tmp3Dim", xsize, ysize, zsize);
            printf("        AOloopControl_3Dloadcreate_shmim: ===== sizeOK = %d\n", (int) sizeOK);
            fflush(stdout);
            if(sizeOK == 1) {
                long xsize1, ysize1, zsize1;

                xsize1 = data.image[ID1].md[0].size[0];
                ysize1 = data.image[ID1].md[0].size[1];
                zsize1 = data.image[ID1].md[0].size[2];
                sizearray[0] = xsize1;
                sizearray[1] = ysize1;
                sizearray[2] = zsize1;
                ID = create_image_ID(name, 3, sizearray, _DATATYPE_FLOAT, 1, 0);

                printf("        AOloopControl_3Dloadcreate_shmim: ===== [1] memcpy  %ld %ld %ld\n", xsize1, ysize1, zsize1);
                fflush(stdout);

                memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize1 * ysize1 * zsize1);

                printf("        AOloopControl_3Dloadcreate_shmim: ===== [1] memcpy  DONE\n");
                fflush(stdout);

                loadcreatestatus = 1;
            } else {
                printf("File \"%s\" has wrong size (should be 3-D %ld x %ld, x %ld  is %d-D %ld x %ld x %ld): ignoring\n", fname, xsize, ysize, zsize, (int) data.image[ID1].md[0].naxis, (long) data.image[ID1].md[0].size[0], (long) data.image[ID1].md[0].size[1], (long) data.image[ID1].md[0].size[2]);
                loadcreatestatus = 3;
            }
        } else {
            printf("        AOloopControl_3Dloadcreate_shmim: ===== [2] memcpy %ld <- %ld     %ld %ld %ld\n", ID, ID1, xsize, ysize, zsize);
            fflush(stdout);
            list_image_ID();

            memcpy(data.image[ID].array.F, data.image[ID1].array.F, sizeof(float)*xsize * ysize * zsize);

            printf("        AOloopControl_3Dloadcreate_shmim: ===== [2] memcpy  DONE\n");
            fflush(stdout);

            loadcreatestatus = 2;
        }
        delete_image_ID("tmp3Dim");
    } else {
        if(CreateSMim == 0) {
            loadcreatestatus = 4;
        } else {
            loadcreatestatus = 5;
        }
    }


    if(loadcreateshm_log == 1) { // results should be logged in ASCII file
        switch(loadcreatestatus) {
            case 0 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream has wrong size -> recreating stream\n", fname, name);
                break;
            case 1 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: new stream created and content loaded\n", fname, name);
                break;
            case 2 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: existing stream updated\n", fname, name);
                break;
            case 3 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image has wrong size -> do nothing\n", fname, name);
                break;
            case 4 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream exists -> do nothing\n", fname, name);
                break;
            case 5 :
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: FITS image does not exist, stream does not exist -> create empty stream\n", fname, name);
                break;
            default:
                fprintf(loadcreateshm_fplog, "LOADING FITS FILE %s TO STREAM %s: UNKNOWN ERROR CODE\n", fname, name);
                break;
        }
    }


    printf("-------- EXITING AOloopControl_3Dloadcreate_shmim ----------\n");
    fflush(stdout);


#ifdef AOLOOPCONTROL_LOGFUNC
    AOLOOPCONTROL_logfunc_level = 2;
    CORE_logFunctionCall(AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    return ID;
}

