/**
 * @file    AOloopControl_perfTest.c
 * @brief   Adaptive Optics Control loop engine testing
 *
 * AO engine uses stream data structure
 *
 *
 * ## Main files
 *
 * AOloopControl_perfTest_DM.c               Test DM speed and examine DM modes
 * AOloopControl_perfTest_status.c           Report loop metrics, loop
 * performance monitor
 *
 *
 *
 */


// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaopt"

// Module short description
#define MODULE_DESCRIPTION "AO loop control performance monitoring and testing"

// Application to which module belongs
#define MODULE_APPLICATION "cacao-cli"

#define _GNU_SOURCE


#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string.h> /* strrchr */
#include <sys/stat.h>
#include <unistd.h> /* chdir */

#include "CLIcore.h"

#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_perfTest/AOloopControl_perfTest.h"

#include "compRMsensitivity.h"
#include "mlat.h"
#include "mlat_decode.h"

#include "streamlogtimesample.h"
#include "wfsrefoptimselect.h"
#include "zoptsearch.h"


#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

#define MaxNBdatFiles 100000


typedef struct
{
    char   name[500];
    double tstart;
    double tend;
    long   cnt;
} StreamDataFile;


INIT_MODULE_LIB(AOloopControl_perfTest)


static errno_t init_module_CLI()
{


    CLIADDCMD_AOloopControl_perfTest__compRMsensitivity();
    CLIADDCMD_AOloopControl_perfTest__mlat();
    CLIADDCMD_AOloopControl_perfTest__mlat_decode();

    CLIADDCMD_AOloopControl_perfTest__streamlogtimesample();

    CLIADDCMD_AOloopControl_perfTest__WFSref_optimize_PSFselection();

    CLIADDCMD_AOloopControl_perfTest__zoptsearch();

    return RETURN_SUCCESS;
}


/**
 * # Purpose
 *
 * Perform statistical analysis of two streams from similarity matrices
 *
 * # Details
 *
 * Selects the NBselected most similar pairs in stream0 and stream1 separated by
 * at least dtmin frames
 *
 * Computes the differences between the corresponding pairs in the other stream
 *
 * # Output
 *
 * sim0pairs.txt  : best NBselected stream0 pairs\n
 * sim1pairs.txt  : best NBselected stream1 pairs\n
 * sim2Ddistrib   : 2D similarity distribution image\n
 *
 * sim0diff0      : best sim pairs 0, differences stream 0 images\n
 * sim0diff1      : best sim pairs 0, differences stream 1 images\n
 * sim1diff0      : best sim pairs 1, differences stream 0 images\n
 * sim1diff1      : best sim pairs 1, differences stream 1 images\n
 *
 */
/*
errno_t AOloopControl_perfTest_StatAnalysis_2streams(char *IDname_stream0,
        char *IDname_stream1,
        char *IDname_simM0,
        char *IDname_simM1,
        long  dtmin,
        unsigned long NBselected)
{
    imageID IDstream0;
    imageID IDstream1;
    imageID IDsimM0;
    imageID IDsimM1;

    unsigned long long NBpairMax;

    // similarity pairs extracted from stream0
    unsigned long *sim0pair_k1;
    unsigned long *sim0pair_k2;
    double        *sim0pair_val;

    // similarity pairs extracted from stream1
    unsigned long *sim1pair_k1;
    unsigned long *sim1pair_k2;
    double        *sim1pair_val;

    uint32_t           xsize0, ysize0;
    unsigned long      NBframe0, xysize0;
    uint32_t           xsize1, ysize1;
    unsigned long      NBframe1, xysize1;
    unsigned long      k1, k2;
    unsigned long long paircnt;

    // ouput
    unsigned long pair;
    FILE         *fpout0;
    FILE         *fpout1;

    double mediansim0, mediansim1;

    IDstream0 = image_ID(IDname_stream0,
        data.core.image,
        data.core.NB_MAX_IMAGE);
    xsize0    = data.core.image[IDstream0].md[0].size[0];
    ysize0    = data.core.image[IDstream0].md[0].size[1];
    xysize0   = xsize0 * ysize0;
    NBframe0  = data.core.image[IDstream0].md[0].size[2];

    NBpairMax =
        (unsigned long long) NBframe0; // data.core.image[IDsimM0].md[0].size[0];
    NBpairMax *= (unsigned long long)(NBframe0 - 1) / 2;
    printf("NBpairMax = %llu x %llu =  %llu\n",
           (unsigned long long) NBframe0,
           (unsigned long long)(NBframe0 - 1) / 2,
           NBpairMax);

    IDstream1 = image_ID(IDname_stream1,
        data.core.image,
        data.core.NB_MAX_IMAGE);
    xsize1    = data.core.image[IDstream1].md[0].size[0];
    ysize1    = data.core.image[IDstream1].md[0].size[1];
    xysize1   = xsize1 * ysize1;
    NBframe1  = data.core.image[IDstream1].md[0].size[2];

    IDsimM0 = image_ID(IDname_simM0, data.core.image, data.core.NB_MAX_IMAGE);
    IDsimM1 = image_ID(IDname_simM1, data.core.image, data.core.NB_MAX_IMAGE);

    // a few checks before proceeding
    if(NBframe0 != NBframe1)
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != NBframe1 (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               NBframe1);
        exit(0);
    }

    if(NBframe0 != data.core.image[IDsimM0].md[0].size[0])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != simM0 xsize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               (long) data.core.image[IDsimM0].md[0].size[0]);
        exit(0);
    }

    if(NBframe0 != data.core.image[IDsimM0].md[0].size[1])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe0 (%ld) != simM0 ysize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe0,
               (long) data.core.image[IDsimM0].md[0].size[1]);
        exit(0);
    }

    if(NBframe1 != data.core.image[IDsimM1].md[0].size[0])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe1 (%ld) != simM1 xsize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe1,
               (long) data.core.image[IDsimM1].md[0].size[0]);
        exit(0);
    }

    if(NBframe1 != data.core.image[IDsimM1].md[0].size[1])
    {
        printf("[%s] [%s] [%d]  ERROR: NBframe1 (%ld) != simM1 ysize (%ld)\n",
               __FILE__,
               __FUNCTION__,
               __LINE__,
               NBframe1,
               (long) data.core.image[IDsimM1].md[0].size[1]);
        exit(0);
    }

    sim0pair_k1 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim0pair_k1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim0pair_k2 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim0pair_k2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim0pair_val = (double *) malloc(sizeof(double) * NBpairMax);
    if(sim0pair_val == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_k1 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim1pair_k1 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_k2 = (unsigned long *) malloc(sizeof(unsigned long) * NBpairMax);
    if(sim1pair_k2 == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    sim1pair_val = (double *) malloc(sizeof(double) * NBpairMax);
    if(sim1pair_val == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    paircnt = 0;
    for(k1 = 0; k1 < NBframe0; k1++)
    {
        for(k2 = 0; k2 < k1; k2++)
        {
            if(((long) k1 - (long) k2) > dtmin)
            {
                if(paircnt > NBpairMax - 1)
                {
                    printf(
                        "[%s] [%s] [%d]  ERROR: paircnt (%llu) >= NBpairMax "
                        "(%llu)\n",
                        __FILE__,
                        __FUNCTION__,
                        __LINE__,
                        paircnt,
                        NBpairMax);
                    printf("NBframe0 = %ld\n", NBframe0);

                    exit(0);
                }

                sim0pair_k1[paircnt] = k1;
                sim0pair_k2[paircnt] = k2;
                sim0pair_val[paircnt] =
                    data.core.image[IDsimM0].array.F[k1 * NBframe0 + k2];

                sim1pair_k1[paircnt] = k1;
                sim1pair_k2[paircnt] = k2;
                sim1pair_val[paircnt] =
                    data.core.image[IDsimM1].array.F[k1 * NBframe1 + k2];

                paircnt++;
            }
        }
    }

    printf("Running quicksort sim0 (%llu elements) ... ", paircnt);
    fflush(stdout);
    quick_sort3ulul_double(sim0pair_val,
                           sim0pair_k1,
                           sim0pair_k2,
                           (long) paircnt);
    printf("Done\n");
    fflush(stdout);

    printf("Running quicksort sim1 (%llu elements) ... ", paircnt);
    fflush(stdout);
    quick_sort3ulul_double(sim1pair_val,
                           sim1pair_k1,
                           sim1pair_k2,
                           (long) paircnt);
    printf("Done\n");
    fflush(stdout);

    mediansim0 = sim0pair_val[paircnt / 2];
    mediansim1 = sim1pair_val[paircnt / 2];

    if((fpout0 = fopen("sim0pairs.txt", "w")) == NULL)
    {
        printf("[%s] [%s] [%d]  ERROR: cannot create file\n",
               __FILE__,
               __FUNCTION__,
               __LINE__);
        exit(0);
    }
    for(pair = 0; pair < NBselected; pair++)
    {
        k1 = sim0pair_k1[pair];
        k2 = sim0pair_k2[pair];
        fprintf(fpout0,
                "%5ld  %5ld  %5ld  %8.6f  %8.6f\n",
                pair,
                k1,
                k2,
                data.core.image[IDsimM0].array.F[k1 * NBframe0 + k2] / mediansim0,
                data.core.image[IDsimM1].array.F[k1 * NBframe0 + k2] / mediansim1);
    }
    fclose(fpout0);

    if((fpout1 = fopen("sim1pairs.txt", "w")) == NULL)
    {
        printf("[%s] [%s] [%d]  ERROR: cannot create file\n",
               __FILE__,
               __FUNCTION__,
               __LINE__);
        exit(0);
    }
    for(pair = 0; pair < NBselected; pair++)
    {
        k1 = sim1pair_k1[pair];
        k2 = sim1pair_k2[pair];
        fprintf(fpout1,
                "%5ld  %5ld  %5ld  %8.6f  %8.6f\n",
                pair,
                k1,
                k2,
                data.core.image[IDsimM0].array.F[k1 * NBframe0 + k2] / mediansim0,
                data.core.image[IDsimM1].array.F[k1 * NBframe1 + k2] / mediansim1);
    }
    fclose(fpout1);

    // Create 2D distribution of similarities

    uint32_t xsize2Ddistrib = 512;
    uint32_t ysize2Ddistrib = 512;

    imageID IDsim2Ddistrib;
    create_2Dimage_ID("sim2Ddistrib",
                      xsize2Ddistrib,
                      ysize2Ddistrib,
                      &IDsim2Ddistrib);

    for(k1 = 0; k1 < NBframe0; k1++)
        for(k2 = 0; k2 < k1; k2++)
        {
            if(((int) k1 - (int) k2) > dtmin)
            {
                float         x, y;
                unsigned long ii, jj;

                x = data.core.image[IDsimM0].array.F[k1 * NBframe0 + k2] /
                    mediansim0;
                y = data.core.image[IDsimM1].array.F[k1 * NBframe1 + k2] /
                    mediansim1;

                ii = (uint32_t)(0.5 * x * xsize2Ddistrib);
                jj = (uint32_t)(0.5 * y * ysize2Ddistrib);

                if((ii < xsize2Ddistrib) && (jj < ysize2Ddistrib))
                {
                    data.core.image[IDsim2Ddistrib]
                    .array.F[jj * xsize2Ddistrib + ii] += 1.0;
                }
            }
        }

    imageID IDsim0diff0;
    create_3Dimage_ID("sim0diff0", xsize0, ysize0, NBselected, &IDsim0diff0);

    imageID IDsim0diff1;
    create_3Dimage_ID("sim0diff1", xsize1, ysize1, NBselected, &IDsim0diff1);

    imageID IDsim0pair0;
    create_3Dimage_ID("sim0pair0",
                      xsize0 * 3,
                      ysize0,
                      NBselected,
                      &IDsim0pair0);
    imageID IDsim0pair1;
    create_3Dimage_ID("sim0pair1",
                      xsize1 * 3,
                      ysize1,
                      NBselected,
                      &IDsim0pair1);

    for(pair = 0; pair < NBselected; pair++)
    {
        unsigned long ii, jj;

        k1 = sim0pair_k1[pair];
        k2 = sim0pair_k2[pair];

        for(ii = 0; ii < xysize0; ii++)
        {
            data.core.image[IDsim0diff0].array.F[pair * xysize0 + ii] =
                data.core.image[IDstream0].array.F[k1 * xysize0 + ii] -
                data.core.image[IDstream0].array.F[k2 * xysize0 + ii];
        }
        for(ii = 0; ii < xysize1; ii++)
        {
            data.core.image[IDsim0diff1].array.F[pair * xysize1 + ii] =
                data.core.image[IDstream1].array.F[k1 * xysize1 + ii] -
                data.core.image[IDstream1].array.F[k2 * xysize1 + ii];
        }

        for(ii = 0; ii < xsize0; ii++)
            for(jj = 0; jj < ysize0; jj++)
            {
                data.core.image[IDsim0pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii] =
                                                    data.core.image[IDstream0]
                                                    .array.F[k1 * xysize0 + jj * xsize0 + ii];
                data.core.image[IDsim0pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii + xsize0] =
                                                    data.core.image[IDstream0]
                                                    .array.F[k2 * xysize0 + jj * xsize0 + ii];
                data.core.image[IDsim0pair0]
                .array.F[pair * ysize0 * xsize0 * 3 + jj * xsize0 * 3 + ii +
                              xsize0 * 2] =
                             data.core.image[IDstream0]
                             .array.F[k1 * xysize0 + jj * xsize0 + ii] -
                             data.core.image[IDstream0]
                             .array.F[k2 * xysize0 + jj * xsize0 + ii];
            }

        for(ii = 0; ii < xsize1; ii++)
            for(jj = 0; jj < ysize1; jj++)
            {
                data.core.image[IDsim0pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii] =
                                                    data.core.image[IDstream1]
                                                    .array.F[k1 * xysize1 + jj * xsize1 + ii];
                data.core.image[IDsim0pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii + xsize1] =
                                                    data.core.image[IDstream1]
                                                    .array.F[k2 * xysize1 + jj * xsize1 + ii];
                data.core.image[IDsim0pair1]
                .array.F[pair * ysize1 * xsize1 * 3 + jj * xsize1 * 3 + ii +
                              xsize1 * 2] =
                             data.core.image[IDstream1]
                             .array.F[k1 * xysize1 + jj * xsize1 + ii] -
                             data.core.image[IDstream1]
                             .array.F[k2 * xysize1 + jj * xsize1 + ii];
            }
    }

    imageID IDsim1diff0;
    create_3Dimage_ID("sim1diff0", xsize0, ysize0, NBselected, &IDsim1diff0);

    imageID IDsim1diff1;
    create_3Dimage_ID("sim1diff1", xsize1, ysize1, NBselected, &IDsim1diff1);

    imageID IDsim1pair0;
    create_3Dimage_ID("sim1pair0",
                      xsize0 * 3,
                      ysize0,
                      NBselected,
                      &IDsim1pair0);
    imageID IDsim1pair1;
    create_3Dimage_ID("sim1pair1",
                      xsize1 * 3,
                      ysize1,
                      NBselected,
                      &IDsim1pair1);

    for(pair = 0; pair < NBselected; pair++)
    {
        unsigned long ii, jj;

        k1 = sim1pair_k1[pair];
        k2 = sim1pair_k2[pair];

        for(ii = 0; ii < xysize0; ii++)
        {
            data.core.image[IDsim1diff0].array.F[pair * xysize0 + ii] =
                data.core.image[IDstream0].array.F[k1 * xysize0 + ii] -
                data.core.image[IDstream0].array.F[k2 * xysize0 + ii];
        }
        for(ii = 0; ii < xysize1; ii++)
        {
            data.core.image[IDsim1diff1].array.F[pair * xysize1 + ii] =
                data.core.image[IDstream1].array.F[k1 * xysize1 + ii] -
                data.core.image[IDstream1].array.F[k2 * xysize1 + ii];
        }

        for(ii = 0; ii < xsize0; ii++)
            for(jj = 0; jj < ysize0; jj++)
            {
                data.core.image[IDsim1pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii] =
                                                    data.core.image[IDstream0]
                                                    .array.F[k1 * xysize0 + jj * xsize0 + ii];
                data.core.image[IDsim1pair0].array.F[pair * ysize0 * xsize0 * 3 +
                                                jj * xsize0 * 3 + ii + xsize0] =
                                                    data.core.image[IDstream0]
                                                    .array.F[k2 * xysize0 + jj * xsize0 + ii];
                data.core.image[IDsim1pair0]
                .array.F[pair * ysize0 * xsize0 * 3 + jj * xsize0 * 3 + ii +
                              xsize0 * 2] =
                             data.core.image[IDstream0]
                             .array.F[k1 * xysize0 + jj * xsize0 + ii] -
                             data.core.image[IDstream0]
                             .array.F[k2 * xysize0 + jj * xsize0 + ii];
            }

        for(ii = 0; ii < xsize1; ii++)
            for(jj = 0; jj < ysize1; jj++)
            {
                data.core.image[IDsim1pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii] =
                                                    data.core.image[IDstream1]
                                                    .array.F[k1 * xysize1 + jj * xsize1 + ii];
                data.core.image[IDsim1pair1].array.F[pair * ysize1 * xsize1 * 3 +
                                                jj * xsize1 * 3 + ii + xsize1] =
                                                    data.core.image[IDstream1]
                                                    .array.F[k2 * xysize1 + jj * xsize1 + ii];
                data.core.image[IDsim1pair1]
                .array.F[pair * ysize1 * xsize1 * 3 + jj * xsize1 * 3 + ii +
                              xsize1 * 2] =
                             data.core.image[IDstream1]
                             .array.F[k1 * xysize1 + jj * xsize1 + ii] -
                             data.core.image[IDstream1]
                             .array.F[k2 * xysize1 + jj * xsize1 + ii];
            }
    }

    free(sim0pair_k1);
    free(sim0pair_k2);
    free(sim0pair_val);

    free(sim1pair_k1);
    free(sim1pair_k2);
    free(sim1pair_val);

    return RETURN_SUCCESS;
}
*/


/**
 *
 * PSF evaluation window is (x0,y0) to (x1,y1)
 *
 * Optional input: PSFmask, to be multiplied by PSF
 *
 *
 * EvalMode = 0  : Maximize Energy concentration
 * EvalMode = 1  : Maximize flux
 * EvalMode = 2  : Minimize flux
 *
 * output:
 *
 * imwfsbest
 * imwfsall
 *
 * impsfbest
 * impsfall
 *
 *
 *
 */

errno_t AOloopControl_perfTest_SelectWFSframes_from_PSFframes(char *IDnameWFS,
        char *IDnamePSF,
        float frac,
        long  x0,
        long  x1,
        long  y0,
        long  y1,
        int   EvalMode,
        float alpha)
{
    imageID IDwfs;
    imageID IDpsf;
    imageID IDpsfmask; // optional

    long NBframe;
    long xsizewfs, ysizewfs, xysizewfs;
    long xsizepsf, ysizepsf, xysizepsf;

    double *evalarray;
    long   *indexarray;

    IDwfs = image_ID(IDnameWFS, data.core.image, data.core.NB_MAX_IMAGE);
    IDpsf = image_ID(IDnamePSF, data.core.image, data.core.NB_MAX_IMAGE);

    xsizewfs  = data.core.image[IDwfs].md[0].size[0];
    ysizewfs  = data.core.image[IDwfs].md[0].size[1];
    xysizewfs = xsizewfs * ysizewfs;

    xsizepsf  = data.core.image[IDpsf].md[0].size[0];
    ysizepsf  = data.core.image[IDpsf].md[0].size[1];
    xysizepsf = xsizepsf * ysizepsf;

    NBframe = data.core.image[IDwfs].md[0].size[2];

    evalarray = (double *) malloc(sizeof(double) * NBframe);
    if(evalarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    indexarray = (long *) malloc(sizeof(long) * NBframe);
    if(indexarray == NULL)
    {
        PRINT_ERROR("malloc returns NULL pointer");
        abort(); // or handle error in other ways
    }

    long x0t, y0t, x1t, y1t;
    x0t = x0;
    x1t = x1;
    y0t = y0;
    y1t = y1;

    if(x0 < 0)
    {
        x0t = x0;
    }
    if(x1 > xsizepsf - 1)
    {
        x1t = xsizepsf - 1;
    }
    if(y0 < 0)
    {
        y0t = y0;
    }
    if(y1 > ysizepsf - 1)
    {
        y1t = ysizepsf - 1;
    }

    printf("WINDOW: %ld - %ld     %ld -%ld\n", x0t, x1t, y0t, y1t);

    long kk;
    IDpsfmask = image_ID("PSFmask", data.core.image, data.core.NB_MAX_IMAGE);
    if(IDpsfmask != -1)
    {
        for(kk = 0; kk < NBframe; kk++)
        {
            long ii, jj;

            for(ii = x0t; ii < x1t; ii++)
                for(jj = y0t; jj < y1t; jj++)
                {
                    data.core.image[IDpsf]
                    .array.F[kk * xysizepsf + jj * xsizepsf + ii] *=
                        data.core.image[IDpsfmask].array.F[jj * xsizepsf + ii];
                }
        }
    }

    for(kk = 0; kk < NBframe; kk++)
    {
        long   ii, jj;
        double sum  = 0.0;
        double ssum = 0.0;

        indexarray[kk] = kk;

        for(ii = x0t; ii < x1t; ii++)
            for(jj = y0t; jj < y1t; jj++)
            {
                float tval;
                tval = data.core.image[IDpsf]
                       .array.F[kk * xysizepsf + jj * xsizepsf + ii];
                if(tval < 0.0)
                {
                    tval = 0.0;
                }
                sum += tval;
                ssum += powf(tval, alpha);
            }

        // best frame
        switch(EvalMode)
        {
        case 0:
            evalarray[kk] = -(ssum / (powf(sum, alpha)));
            break;

        case 1:
            evalarray[kk] = -sum;
            break;

        case 2:
            evalarray[kk] = sum;
            break;

        default:
            evalarray[kk] = -sum;
            break;
        }
    }

    quick_sort2l(evalarray, indexarray, NBframe);

    long IDwfsbest, IDwfsall;
    long IDpsfbest, IDpsfall;

    create_2Dimage_ID("imwfsbest", xsizewfs, ysizewfs, &IDwfsbest);
    create_2Dimage_ID("imwfsall", xsizewfs, ysizewfs, &IDwfsall);

    create_2Dimage_ID("impsfbest", xsizepsf, ysizepsf, &IDpsfbest);
    create_2Dimage_ID("impsfall", xsizepsf, ysizepsf, &IDpsfall);

    long kklim;
    kklim = (long)(frac * NBframe);

    printf("kklim = %ld     %ld %ld\n", kklim, xysizewfs, xysizepsf);

    FILE *fp = fopen("fptest.txt", "w");
    for(kk = 0; kk < NBframe; kk++)
    {
        long ii;

        fprintf(fp, "%6ld  %6ld  %g\n", kk, indexarray[kk], evalarray[kk]);

        if(kk < kklim)
        {
            for(ii = 0; ii < xysizewfs; ii++)
            {
                data.core.image[IDwfsbest].array.F[ii] +=
                    data.core.image[IDwfs].array.F[indexarray[kk] * xysizewfs + ii];
            }

            for(ii = 0; ii < xysizepsf; ii++)
            {
                data.core.image[IDpsfbest].array.F[ii] +=
                    data.core.image[IDpsf].array.F[indexarray[kk] * xysizepsf + ii];
            }
        }

        for(ii = 0; ii < xysizewfs; ii++)
        {
            data.core.image[IDwfsall].array.F[ii] +=
                data.core.image[IDwfs].array.F[kk * xysizewfs + ii];
        }

        for(ii = 0; ii < xysizepsf; ii++)
        {
            data.core.image[IDpsfall].array.F[ii] +=
                data.core.image[IDpsf].array.F[kk * xysizepsf + ii];
        }
    }
    fclose(fp);

    long ii;

    for(ii = 0; ii < xysizewfs; ii++)
    {
        data.core.image[IDwfsbest].array.F[ii] /= kklim;
    }

    for(ii = 0; ii < xysizepsf; ii++)
    {
        data.core.image[IDpsfbest].array.F[ii] /= kklim;
    }

    for(ii = 0; ii < xysizewfs; ii++)
    {
        data.core.image[IDwfsall].array.F[ii] /= NBframe;
    }

    for(ii = 0; ii < xysizepsf; ii++)
    {
        data.core.image[IDpsfall].array.F[ii] /= NBframe;
    }

    free(evalarray);
    free(indexarray);

    return RETURN_SUCCESS;
}
