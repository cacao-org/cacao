/**
 * @file    AOloopControl_computeCalib_CM.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
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

#define MAX_MBLOCK 20





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

imageID AOloopControl_computeCalib_compute_ControlMatrix(
    long        loop,
    __attribute__((unused)) long        NB_MODE_REMOVED,
    const char *ID_Rmatrix_name,
    const char *ID_Cmatrix_name,
    const char *ID_VTmatrix_name,
    double      Beta,
    long        NB_MODE_REMOVED_STEP,
    float       eigenvlim
)
{
    FILE *fp;
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

    imageID ID_Rmatrix, ID_Cmatrix, ID_VTmatrix;
    uint32_t *arraysizetmp;

    imageID IDmodes;

    imageID IDeigenmodesResp;
    imageID ID_RMmask;

    double *CPAcoeff; /// gain applied to modes to enhance low orders in SVD

    char fname[200];
    long NB_MR;  /// number of modes removed

    long NB_MODE_REMOVED1;
    float eigenvmin=0.0;
    long NBMODES_REMOVED_EIGENVLIM = 0;


    long MB_MR_start;
    long MB_MR_end;
    long MB_MR_step;

    char command[200];


    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    arraysizetmp = (uint32_t*) malloc(sizeof(uint32_t)*3);


    ID_Rmatrix = image_ID(ID_Rmatrix_name);


    uint64_t n = data.image[ID_Rmatrix].md[0].size[0]*data.image[ID_Rmatrix].md[0].size[1]; //AOconf[loop].AOpmodecoeffs.NBDMmodes;
    uint32_t m = data.image[ID_Rmatrix].md[0].size[2]; //AOconf[loop].WFSim.sizeWFS;


    ID_RMmask = image_ID("RMmask");
    if(ID_RMmask!=-1) // apply mask to response matrix
    {
        for(uint32_t kk=0; kk<m; kk++)
        {
            for(uint64_t ii=0; ii<n; ii++)
                data.image[ID_Rmatrix].array.F[kk*n+ii] *= data.image[ID_RMmask].array.F[ii];
        }
    }



    /** in this procedure, m=number of actuators/modes, n=number of WFS elements */
    //  long m = smao[0].NBmode;
    // long n = smao[0].NBwfselem;

    printf("m = %u actuators (modes), n = %lu sensors\n", m, n);
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
            for(uint32_t k=0; k<m; k++)
                CPAcoeff[k] = 1.0;
        }
        else
        {
            for(uint32_t k=0; k<m; k++)
            {
                CPAcoeff[k] =  exp(-data.image[ID].array.F[k]*Beta);
                printf("%5u %5.3f %g\n", k, data.image[ID].array.F[k], CPAcoeff[k]);
            }
        }
    }
    else
    {
        for(uint32_t k=0; k<m; k++)
            CPAcoeff[k] = 1.0;
    }


    /* write matrix_D */
    for(uint32_t k=0; k<m; k++)
    {
        for(uint64_t ii=0; ii<n; ii++)
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
    for(uint32_t k=0; k<m; k++)
        fprintf(fp,"%u %g\n", k, gsl_vector_get(matrix_DtraD_eval,k));
    fclose(fp);

    eigenvmin = eigenvlim*gsl_vector_get(matrix_DtraD_eval,0);

    NBMODES_REMOVED_EIGENVLIM = 0;
    for(uint32_t k=0; k<m; k++)
    {
        printf("Mode %u eigenvalue = %g\n", k, gsl_vector_get(matrix_DtraD_eval,k));
        if(gsl_vector_get(matrix_DtraD_eval,k) < eigenvmin)
            NBMODES_REMOVED_EIGENVLIM++;
    }




    /** Write rotation matrix to go from DM modes to eigenmodes */
    arraysizetmp[0] = m;
    arraysizetmp[1] = m;
    ID_VTmatrix = create_image_ID(ID_VTmatrix_name, 2, arraysizetmp, _DATATYPE_FLOAT, 0, 0);
    for(uint64_t ii=0; ii<m; ii++) // modes
        for(uint32_t k=0; k<m; k++) // modes
            data.image[ID_VTmatrix].array.F[k*m+ii] = (float) gsl_matrix_get( matrix_DtraD_evec, k, ii);


    /// Compute eigenmodes responses
    IDeigenmodesResp = create_3Dimage_ID(
                           "eigenmodesrespM",
                           data.image[ID_Rmatrix].md[0].size[0],
                           data.image[ID_Rmatrix].md[0].size[1],
                           data.image[ID_Rmatrix].md[0].size[2]);
    printf("Computing eigenmode responses .... \n");
    for(uint32_t kk=0; kk<m; kk++) /// eigen mode index
    {
        printf("\r eigenmode %4u / %4u   ", kk, m);
        fflush(stdout);
        for(uint32_t kk1=0; kk1<m; kk1++)
        {
            for(uint64_t ii=0; ii<n; ii++)
                data.image[IDeigenmodesResp].array.F[kk*n + ii] +=
                    data.image[ID_VTmatrix].array.F[kk1*m+kk]*data.image[ID_Rmatrix].array.F[kk1*n + ii];
        }
    }
    if(sprintf(fname, "!eigenmodesrespM_%4.2f.fits", Beta) < 1)
        print_ERROR("sprintf wrote <1 char");

    save_fits("eigenmodesrespM", fname);
    printf("\n");


    /// if modesM exists, compute eigenmodes using rotation matrix
    IDmodes = image_ID("modesM");
    if(IDmodes!=-1)
    {
        uint32_t xsize_modes = data.image[IDmodes].md[0].size[0];
        uint32_t ysize_modes = data.image[IDmodes].md[0].size[1];
        uint32_t zsize_modes = data.image[IDmodes].md[0].size[2];
        if(zsize_modes != m)
            printf("ERROR: zsize (%ld) of modesM does not match expected size (%ld)\n", (long) zsize_modes, (long) m);
        else
        {
            long IDeigenmodes = create_3Dimage_ID("eigenmodesM", xsize_modes, ysize_modes, m);
            printf("Computing eigenmodes .... \n");
            for(uint32_t kk=0; kk<m; kk++) /// eigen mode index
            {
                printf("\r eigenmode %4u / %4u   ", kk, m);
                fflush(stdout);
                for(uint32_t kk1=0; kk1<m; kk1++)
                {
                    for(uint64_t ii=0; ii<xsize_modes*ysize_modes; ii++)
                        data.image[IDeigenmodes].array.F[kk*xsize_modes*ysize_modes + ii] +=
                            data.image[ID_VTmatrix].array.F[kk1*m+kk]*data.image[IDmodes].array.F[kk1*xsize_modes*ysize_modes + ii];
                }
            }
            printf("\n");
        }

        if(sprintf(fname, "!eigenmodesM_%4.2f.fits", Beta) < 1)
            print_ERROR("sprintf wrote <1 char");

        save_fits("eigenmodesM", fname);
    }




    /// second, build the "inverse" of the diagonal matrix of eigenvalues (matrix1)
    matrix1 = gsl_matrix_alloc (m, m);
    matrix2 = gsl_matrix_alloc (m, m);
    arraysizetmp[0] = AOconf[loop].WFSim.sizexWFS;
    arraysizetmp[1] = AOconf[loop].WFSim.sizeyWFS;
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
        for(uint32_t ii1=0; ii1<m; ii1++)
            for(uint32_t jj1=0; jj1<m; jj1++)
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
        printf("write result to ID %ld   [%lu %u]\n", ID_Cmatrix, n, m);
        fflush(stdout);

        for(uint64_t ii=0; ii<n; ii++) // sensors
            for(uint32_t k=0; k<m; k++) // actuator modes
                data.image[ID_Cmatrix].array.F[k*n+ii] = (float) gsl_matrix_get(matrix_Ds, k, ii)*CPAcoeff[k];


        if(NB_MODE_REMOVED_STEP==0)
        {
            save_fits(ID_Cmatrix_name, "!cmat.fits");

            if(sprintf(command, "echo \"%ld\" > ./cmat.NB_MODES_RM.txt", NBMODES_REMOVED_EIGENVLIM) < 1)
                print_ERROR("sprintf wrote <1 char");

            if(system(command) != 0)
                print_ERROR("system() returns non-zero value");

            if(sprintf(command, "echo \"%u\" > ./cmat.NB_MODES.txt",  m) < 1)
                print_ERROR("sprintf wrote <1 char");

            if(system(command) != 0)
                print_ERROR("system() returns non-zero value");
        }
        else
        {
            if(sprintf(fname, "!cmat_%4.2f_%02ld.fits", Beta, NB_MR) < 1)
                print_ERROR("sprintf wrote <1 char");

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


    return ID_Cmatrix;
}





//
// computes combined control matrix
//
//

errno_t AOloopControl_computeCalib_compute_CombinedControlMatrix(
    const char *IDcmat_name,
    const char *IDmodes_name,
    const char* IDwfsmask_name,
    const char *IDdmmask_name,
    const char *IDcmatc_name,
    const char *IDcmatc_active_name
)
{
    // long ID;
    struct timespec t1;
    struct timespec t2;
    struct timespec tdiff;
    double tdiffv;

    float *matrix_cmp;
    float *matrix_Mc, *matrix_DMmodes;

    imageID    IDwfsmask;
    imageID    IDdmmask;
    uint64_t   sizeWFS_active[100];
    uint64_t   sizeDM_active;
    uint32_t  *sizearray;
    imageID    IDcmat;
    imageID    IDcmatc;
    imageID    IDmodes;
    long       IDcmatc_active[100];
    //char       name[200];
    char       imname[200];


    printf("COMPUTING COMBINED CONTROL MATRIX .... \n");
    fflush(stdout);

    clock_gettime(CLOCK_REALTIME, &t1);


    // initialize size of arrays
    IDwfsmask = image_ID(IDwfsmask_name);
    uint32_t sizexWFS = data.image[IDwfsmask].md[0].size[0];
    uint32_t sizeyWFS = data.image[IDwfsmask].md[0].size[1];
    uint64_t sizeWFS  = sizexWFS*sizeyWFS;

    printf("IDwfsmask = %ld\n", IDwfsmask);
    fflush(stdout);

    IDdmmask = image_ID(IDdmmask_name);
    uint32_t sizexDM = data.image[IDdmmask].md[0].size[0];
    uint32_t sizeyDM = data.image[IDdmmask].md[0].size[1];
    uint64_t sizeDM  = sizexDM*sizeyDM;

    printf("IDdmmask = %ld\n", IDdmmask);
    fflush(stdout);

    IDmodes = image_ID(IDmodes_name);

    printf("IDmodes = %ld\n", IDmodes);
    fflush(stdout);

    uint32_t NBDMmodes = data.image[IDmodes].md[0].size[2];



    // allocate array for combined matrix
    sizearray = (uint32_t*) malloc(sizeof(uint32_t)*3);
    sizearray[0] = sizexWFS;
    sizearray[1] = sizeyWFS;
    sizearray[2] = sizeDM;

    printf("Creating 3D image : %u %u %lu\n", sizexWFS, sizeyWFS, sizeDM);
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
    for(uint32_t mode=0; mode<NBDMmodes; mode++)
    {
        for(uint64_t act=0; act<sizeDM; act++)
        {
            for(uint64_t wfselem=0; wfselem<sizeWFS; wfselem++)
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



    aoloopcontrol_var.WFS_active_map = (int*) malloc(sizeof(int)*sizeWFS*aoloopcontrol_var.PIXSTREAM_NBSLICES);
    for(uint32_t slice=0; slice < (uint32_t) aoloopcontrol_var.PIXSTREAM_NBSLICES; slice++)
    {
        uint64_t ii1 = 0;
        for(uint64_t ii=0; ii<sizeWFS; ii++)
            if(data.image[IDwfsmask].array.F[ii]>0.1)
            {
                if(slice==0)
                {
                    aoloopcontrol_var.WFS_active_map[ii1] = ii;
                    ii1++;
                }
                else
                {
                    if(data.image[aoloopcontrol_var.aoconfID_pixstream_wfspixindex].array.UI16[ii]==slice+1)
                    {
                        aoloopcontrol_var.WFS_active_map[slice*sizeWFS+ii1] = ii;
                        ii1++;
                    }
                }
            }
        sizeWFS_active[slice] = ii1;

        if(sprintf(imname, "aol%ld_imWFS2active_%02d", LOOPNUMBER, slice) < 1)
            print_ERROR("sprintf wrote <1 char");

        /* CAN CRASH
                sizearray = (long*) malloc(sizeof(long)*2);
                sizearray[0] =  sizeWFS_active[slice];
                sizearray[1] =  1;
                aoconfID_imWFS2_active[slice] = create_image_ID(imname, 2, sizearray, FLOAT, 1, 0);
        		free(sizearray);
        */
    }



    aoloopcontrol_var.DM_active_map = (int*) malloc(sizeof(int)*sizeDM);
    uint64_t ii1 = 0;
    for(uint64_t ii=0; ii<sizeDM; ii++)
        if(data.image[IDdmmask].array.F[ii]>0.1)
        {
            aoloopcontrol_var.DM_active_map[ii1] = ii;
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
    for(uint32_t slice=0; slice < (uint32_t) aoloopcontrol_var.PIXSTREAM_NBSLICES; slice++)
    {
        if(sprintf(imname, "%s_%02d", IDcmatc_active_name, slice) < 1)
            print_ERROR("sprintf wrote <1 char");

        IDcmatc_active[slice] = create_2Dimage_ID(imname, sizeWFS_active[slice], sizeDM_active);
        for(uint64_t act_active=0; act_active<sizeDM_active; act_active++)
        {
            for(uint64_t wfselem_active=0; wfselem_active<sizeWFS_active[slice]; wfselem_active++)
            {
                uint64_t act = aoloopcontrol_var.DM_active_map[act_active];
                uint64_t wfselem = aoloopcontrol_var.WFS_active_map[slice*sizeWFS+wfselem_active];
                data.image[IDcmatc_active[slice]].array.F[act_active*sizeWFS_active[slice]+wfselem_active] =
                    matrix_Mc[act*sizeWFS+wfselem];
            }
        }
        printf("PIXEL SLICE %d     Keeping only active pixels / actuators : %ld x %ld   ->   %ld x %ld\n",
               slice, sizeWFS, sizeDM, sizeWFS_active[slice], sizeDM_active);


    }

    free(matrix_Mc);
    free(matrix_DMmodes);



    clock_gettime(CLOCK_REALTIME, &t2);
    tdiff = info_time_diff(t1, t2);
    tdiffv = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
    printf("\n");
    printf("TIME TO COMPUTE MATRIX = %f sec\n", tdiffv);

    return RETURN_SUCCESS;
}





imageID AOloopControl_computeCalib_loadCM(
    long        loop,
    const char *CMfname
)
{
    imageID ID = -1;



    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
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
        if(data.image[ID].md[0].datatype!=_DATATYPE_FLOAT)
        {
            printf("Control matrix has wrong type\n");
            vOK = 0;
        }
        if(vOK==1)
        {
            if(data.image[ID].md[0].size[0]!=AOconf[loop].WFSim.sizexWFS)
            {
                printf("Control matrix has wrong x size : is %ld, should be %ld\n",
                       (long) data.image[ID].md[0].size[0], (long) AOconf[loop].WFSim.sizexWFS);
                vOK = 0;
            }
            if(data.image[ID].md[0].size[1]!=AOconf[loop].WFSim.sizeyWFS)
            {
                printf("Control matrix has wrong y size\n");
                vOK = 0;
            }
            if(data.image[ID].md[0].size[2]!=AOconf[loop].AOpmodecoeffs.NBDMmodes)
            {
                printf("Control matrix has wrong z size\n");
                vOK = 0;
            }
        }


        if(vOK==1)
        {
            AOconf[loop].aorun.init_CM = 1;
            char name[200];
            if(sprintf(name, "ContrM_%ld", loop) < 1)
                print_ERROR("sprintf wrote <1 char");

            ID = image_ID(name);
            if(ID==-1)
                ID = read_sharedmem_image(name);
            long ID0 = image_ID("tmpcontrM");
            data.image[ID].md[0].write  = 1;

            for(uint64_t ii=0; ii<AOconf[loop].WFSim.sizexWFS*AOconf[loop].WFSim.sizeyWFS*AOconf[loop].AOpmodecoeffs.NBDMmodes; ii++) {
                data.image[ID].array.F[ii] = data.image[ID0].array.F[ii];
            }
            data.image[ID].md[0].write  = 0;
            data.image[ID].md[0].cnt0++;
        }
        delete_image_ID("tmpcontrM");
    }

    return ID;
}
