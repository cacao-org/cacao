/**
 * @file    AOloopControl_computeCalib_loDMmodes.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

#define _GNU_SOURCE



#include <math.h>

#include "CommandLineInterface/CLIcore.h"

#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "ZernikePolyn/ZernikePolyn.h"
#include "image_filter/image_filter.h"
#include "linopt_imtools/linopt_imtools.h"

#include "ZernikePolyn/zernike_value.h"



#include "linalgebra/linalgebra.h"




#define MAX_MBLOCK 20

#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

// make low order DM modes
imageID AOloopControl_computeCalib_mkloDMmodes(const char *ID_name,
        uint32_t    msizex,
        uint32_t    msizey,
        float       CPAmax,
        float       deltaCPA,
        double      xc,
        double      yc,
        double      r0,
        double      r1,
        int         MaskMode)
{
    imageID IDmask;
    imageID ID, ID0, IDtm, IDem, IDslaved;
    double  x, y, r, PA, xc1, yc1, totm, offset, rms;

    long NBZ, m;
    long zindex[10];
    double
    zcpa[10]; /// CPA for each Zernike (somewhat arbitrary... used to sort
    /// modes in CPA)
    long IDfreq, IDmfcpa;
    long k;

    double coeff;
    long   kelim = 20;

    zindex[0] = 1; // tip
    zcpa[0]   = 0.0;

    zindex[1] = 2; // tilt
    zcpa[1]   = 0.0;

    zindex[2] = 4; // focus
    zcpa[2]   = 0.25;

    zindex[3] = 3; // astig
    zcpa[3]   = 0.4;

    zindex[4] = 5; // astig
    zcpa[4]   = 0.4;

    zindex[5] = 7; // coma
    zcpa[5]   = 0.6;

    zindex[6] = 8; // coma
    zcpa[6]   = 0.6;

    zindex[7] = 6; // trefoil
    zcpa[7]   = 1.0;

    zindex[8] = 9; // trefoil
    zcpa[8]   = 1.0;

    zindex[9] = 12;
    zcpa[9]   = 1.5;

    printf("msizexy = %u %u\n", msizex, msizey);
    list_image_ID();
    IDmask = image_ID("dmmask");
    if(IDmask == -1)
    {
        double val0, val1;
        double a0 = 0.88;
        double b0 = 40.0;
        double a1 = 1.2;
        double b1 = 12.0;

        create_2Dimage_ID("dmmask", msizex, msizey, &IDmask);
        for(uint32_t ii = 0; ii < msizex; ii++)
            for(uint32_t jj = 0; jj < msizey; jj++)
            {
                x    = 1.0 * ii - xc;
                y    = 1.0 * jj - yc;
                r    = sqrt(x * x + y * y) / r1;
                val1 = 1.0 - exp(-pow(a1 * r, b1));
                r    = sqrt(x * x + y * y) / r0;
                val0 = exp(-pow(a0 * r, b0));
                data.image[IDmask].array.F[jj * msizex + ii] = val0 * val1;
            }
        save_fits("dmmask", "dmmask.fits");
        xc1 = xc;
        yc1 = yc;
    }
    else /// extract xc and yc from mask
    {
        xc1  = 0.0;
        yc1  = 0.0;
        totm = 0.0;
        for(uint32_t ii = 0; ii < msizex; ii++)
            for(uint32_t jj = 0; jj < msizey; jj++)
            {
                xc1 += 1.0 * ii * data.image[IDmask].array.F[jj * msizex + ii];
                yc1 += 1.0 * jj * data.image[IDmask].array.F[jj * msizex + ii];
                totm += data.image[IDmask].array.F[jj * msizex + ii];
            }
        // printf("xc1 yc1    %f  %f     %f\n", xc1, yc1, totm);
        xc1 /= totm;
        yc1 /= totm;
    }

    totm = arith_image_total("dmmask");
    if((msizex != data.image[IDmask].md[0].size[0]) ||
            (msizey != data.image[IDmask].md[0].size[1]))
    {
        printf(
            "ERROR: file dmmask size (%u %u) does not match expected size "
            "(%u %u)\n",
            data.image[IDmask].md[0].size[0],
            data.image[IDmask].md[0].size[1],
            msizex,
            msizey);
        exit(0);
    }

    NBZ = 0;

    for(m = 0; m < 10; m++)
    {
        if(zcpa[m] < CPAmax)
        {
            NBZ++;
        }
    }


    {
        IMGID imgoutm = mkIMGID_from_name("CPAmodes");

        // optional mask
        //
        IMGID imgCPAmask = mkIMGID_from_name("modesCPAmask");
        resolveIMGID(&imgCPAmask, ERRMODE_WARN);

        linopt_imtools_makeCPAmodes(&imgoutm,
                                    msizex,
                                    0.0,
                                    1.5*CPAmax,
                                    CPAmax,
                                    deltaCPA,
                                    0.5 * msizex,
                                    1.2,
                                    0,
                                    NULL,
                                    imgCPAmask,
                                    0.0,
                                    0.0
                                    );
    }

    ID0    = image_ID("CPAmodes");
    IDfreq = image_ID("cpamodesfreq");

    printf("  %u %u %ld\n",
           msizex,
           msizey,
           (long) data.image[ID0].md[0].size[2] - 1);
    create_3Dimage_ID(ID_name,
                      msizex,
                      msizey,
                      data.image[ID0].md[0].size[2] - 1 + NBZ,
                      &ID);

    create_2Dimage_ID("modesfreqcpa",
                      data.image[ID0].md[0].size[2] - 1 + NBZ,
                      1,
                      &IDmfcpa);

    /*** Create TTF first */
    zernike_init();
    printf("r1 = %f    %f %f\n", r1, xc1, yc1);
    for(k = 0; k < NBZ; k++)
    {
        data.image[IDmfcpa].array.F[k] = zcpa[k];
        for(uint32_t ii = 0; ii < msizex; ii++)
            for(uint32_t jj = 0; jj < msizey; jj++)
            {
                x  = 1.0 * ii - xc1;
                y  = 1.0 * jj - yc1;
                r  = sqrt(x * x + y * y) / r1;
                PA = atan2(y, x);
                data.image[ID].array.F[k * msizex * msizey + jj * msizex + ii] =
                    Zernike_value(zindex[k], r, PA);
            }
    }

    for(uint32_t k = 0; k < (uint32_t) data.image[ID0].md[0].size[2] - 1; k++)
    {
        data.image[IDmfcpa].array.F[k + NBZ] =
            data.image[IDfreq].array.F[k + 1];
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
            data.image[ID].array.F[(k + NBZ) * msizex * msizey + ii] =
                data.image[ID0].array.F[(k + 1) * msizex * msizey + ii];
    }

    for(uint32_t k = 0;
            k < (uint32_t)(data.image[ID0].md[0].size[2] - 1 + NBZ);
            k++)
    {
        /// Remove excluded modes
        long IDeModes = image_ID("emodes");
        if(IDeModes != -1)
        {
            create_2Dimage_ID("tmpmode", msizex, msizey, &IDtm);

            for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                data.image[IDtm].array.F[ii] =
                    data.image[ID].array.F[k * msizex * msizey + ii];
            linopt_imtools_image_fitModes("tmpmode",
                                          "emodes",
                                          "dmmask",
                                          1.0e-2,
                                          "lcoeff",
                                          0,
                                          NULL);
            linopt_imtools_image_construct("emodes", "lcoeff", "em00", NULL);
            delete_image_ID("lcoeff", DELETE_IMAGE_ERRMODE_WARNING);
            IDem = image_ID("em00");

            coeff = 1.0 - exp(-pow(1.0 * k / kelim, 6.0));
            if(k > 2.0 * kelim)
            {
                coeff = 1.0;
            }
            for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                data.image[ID].array.F[k * msizex * msizey + ii] =
                    data.image[IDtm].array.F[ii] -
                    coeff * data.image[IDem].array.F[ii];

            delete_image_ID("em00", DELETE_IMAGE_ERRMODE_WARNING);
            delete_image_ID("tmpmode", DELETE_IMAGE_ERRMODE_WARNING);
        }

        double totvm = 0.0;
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            //	  data.image[ID].array.F[k*msize*msize+ii] =
            // data.image[ID0].array.F[(k+1)*msize*msize+ii];
            totvm += data.image[ID].array.F[k * msizex * msizey + ii] *
                     data.image[IDmask].array.F[ii];
        }
        offset = totvm / totm;

        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            data.image[ID].array.F[k * msizex * msizey + ii] -= offset;
            data.image[ID].array.F[k * msizex * msizey + ii] *=
                data.image[IDmask].array.F[ii];
        }

        offset = 0.0;
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            offset += data.image[ID].array.F[k * msizex * msizey + ii];
        }

        rms = 0.0;
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            data.image[ID].array.F[k * msizex * msizey + ii] -=
                offset / msizex / msizey;
            rms += data.image[ID].array.F[k * msizex * msizey + ii] *
                   data.image[ID].array.F[k * msizex * msizey + ii];
        }
        rms = sqrt(rms / totm);
        printf("Mode %u   RMS = %lf  (%f)\n", k, rms, totm);
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            data.image[ID].array.F[k * msizex * msizey + ii] /= rms;
        }
    }

    for(k = 0; k < data.image[ID0].md[0].size[2] - 1 + NBZ; k++)
    {
        rms = 0.0;
        for(uint64_t ii = 0; ii < msizex * msizey; ii++)
        {
            data.image[ID].array.F[k * msizex * msizey + ii] -=
                offset / msizex / msizey;
            rms += data.image[ID].array.F[k * msizex * msizey + ii] *
                   data.image[ID].array.F[k * msizex * msizey + ii];
        }
        rms = sqrt(rms / totm);
        printf("Mode %ld   RMS = %lf\n", k, rms);
    }

    if(MaskMode == 1)
    {
        long kernsize = 5;
        long NBciter  = 200;
        long citer;

        if(2 * kernsize > msizex)
        {
            kernsize = msizex / 2;
        }
        for(citer = 0; citer < NBciter; citer++)
        {
            long IDg;

            printf("Convolution [%3ld/%3ld]\n", citer, NBciter);
            gauss_filter(ID_name,
                         "modeg",
                         4.0 * pow(1.0 * (NBciter - citer) / NBciter, 0.5),
                         kernsize);
            IDg = image_ID("modeg");
            for(uint32_t k = 0; k < data.image[ID].md[0].size[2]; k++)
            {
                for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                    if(data.image[IDmask].array.F[ii] < 0.98)
                        data.image[ID].array.F[k * msizex * msizey + ii] =
                            data.image[IDg].array.F[k * msizex * msizey + ii];
            }
            delete_image_ID("modeg", DELETE_IMAGE_ERRMODE_WARNING);
        }
    }

    /// SLAVED ACTUATORS
    IDslaved = image_ID("dmslaved");
    ID       = image_ID(ID_name);
    if((IDslaved != -1) && (IDmask != -1))
    {
        imageID IDtmp;
        create_2Dimage_ID("_tmpinterpol", msizex, msizey, &IDtmp);

        imageID IDtmp1;
        create_2Dimage_ID("_tmpcoeff1", msizex, msizey, &IDtmp1);

        imageID IDtmp2;
        create_2Dimage_ID("_tmpcoeff2", msizex, msizey, &IDtmp2);

        for(m = 0; m < data.image[ID].md[0].size[2]; m++)
        {
            // write input DM mode
            for(uint64_t ii = 0; ii < msizex * msizey; ii++)
            {
                data.image[IDtmp].array.F[ii] =
                    data.image[ID].array.F[m * msizex * msizey + ii];
                data.image[IDtmp1].array.F[ii] =
                    data.image[IDmask].array.F[ii] *
                    (1.0 - data.image[IDslaved].array.F[ii]);
                data.image[IDtmp2].array.F[ii] = data.image[IDtmp1].array.F[ii];
            }

            long  pixcnt = 1;
            float vxp, vxm, vyp, vym, cxp, cxm, cyp, cym;
            float ctot;

            while(pixcnt > 0)
            {
                pixcnt = 0;
                for(uint32_t ii = 1; ii < (uint32_t)(msizex - 1); ii++)
                    for(uint32_t jj = 1; jj < (uint32_t)(msizey - 1); jj++)
                    {
                        if((data.image[IDtmp1].array.F[jj * msizex + ii] <
                                0.5) &&
                                (data.image[IDslaved].array.F[jj * msizex + ii] >
                                 0.5))
                        {
                            pixcnt++;
                            vxp = data.image[IDtmp]
                                  .array.F[jj * msizex + (ii + 1)];
                            cxp = data.image[IDtmp1]
                                  .array.F[jj * msizex + (ii + 1)];

                            vxm = data.image[IDtmp]
                                  .array.F[jj * msizex + (ii - 1)];
                            cxm = data.image[IDtmp1]
                                  .array.F[jj * msizex + (ii - 1)];

                            vyp = data.image[IDtmp]
                                  .array.F[(jj + 1) * msizex + ii];
                            cyp = data.image[IDtmp1]
                                  .array.F[(jj + 1) * msizex + ii];

                            vym = data.image[IDtmp]
                                  .array.F[(jj - 1) * msizex + ii];
                            cym = data.image[IDtmp1]
                                  .array.F[(jj - 1) * msizex + ii];

                            ctot = (cxp + cxm + cyp + cym);

                            if(ctot > 0.5)
                            {
                                data.image[IDtmp].array.F[jj * msizex + ii] =
                                    (vxp * cxp + vxm * cxm + vyp * cyp +
                                     vym * cym) /
                                    ctot;
                                data.image[IDtmp2].array.F[jj * msizex + ii] =
                                    1.0;
                            }
                        }
                    }
                for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                    data.image[IDtmp1].array.F[ii] =
                        data.image[IDtmp2].array.F[ii];
            }
            for(uint64_t ii = 0; ii < msizex * msizey; ii++)
                data.image[ID].array.F[m * msizex * msizey + ii] =
                    data.image[IDtmp].array.F[ii];

            /*

                  IDtmp = create_2Dimage_ID("_tmpinterpol", msizex, msizey);
                  for(m=0; m<data.image[ID].md[0].size[2]; m++)
                  {
                      for(ii=0; ii<msizex*msizey; ii++)
                          data.image[IDtmp].array.F[ii] =
             data.image[ID].array.F[m*msizex*msizey+ii];

                      for(conviter=0; conviter<NBconviter; conviter++)
                      {
                          sigma = 0.5*NBconviter/(1.0+conviter);
                          gauss_filter("_tmpinterpol", "_tmpinterpolg", 1.0, 2);
                          IDtmpg = image_ID("_tmpinterpolg");
                          for(ii=0; ii<msizex*msizey; ii++)
                          {
                              if((data.image[IDmask].array.F[ii]>0.5)&&(data.image[IDslaved].array.F[ii]<0.5))
                                  data.image[IDtmp].array.F[ii] =
             data.image[ID].array.F[m*msizex*msizey+ii]; else
                                  data.image[IDtmp].array.F[ii] =
             data.image[IDtmpg].array.F[ii];
                          }
                          delete_image_ID("_tmpinterpolg",
             DELETE_IMAGE_ERRMODE_WARNING);
                      }
                      for(ii=0; ii<msizex*msizey; ii++)
                          if(data.image[IDmask].array.F[ii]>0.5)
                              data.image[ID].array.F[m*msizex*msizey+ii] =
             data.image[IDtmp].array.F[ii];
                  */
        }
        delete_image_ID("_tmpinterpol", DELETE_IMAGE_ERRMODE_WARNING);
        delete_image_ID("_tmpcoeff1", DELETE_IMAGE_ERRMODE_WARNING);
        delete_image_ID("_tmpcoeff2", DELETE_IMAGE_ERRMODE_WARNING);
    }

    return ID;
}
