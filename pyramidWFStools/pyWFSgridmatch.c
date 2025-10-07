/**
 * @file    acquireWFS.c
 * @brief   acquire and preprocess WFS image
 *
 */


#include <math.h>

#include "CommandLineInterface/CLIcore.h"


// define struct type for grid parameters
typedef struct
{
    uint32_t dmxsize;
    uint32_t dmysize;

    // Separation betwen pupil images (square 1/2 size) in WFS pix unit
    // allowing for different x and y offsets
    float pupsquare_xoffset;
    float pupsquare_yoffset;
    // Angle
    float pupsquare_angle;
    // Center of square
    float pupsquare_center_x;
    float pupsquare_center_y;

    // Actuator pitch
    float actpitch_x;
    float actpitch_y;
    // Angle
    float actpitch_angle;

} pyrWFSgrid;




static char *inimname;

// kernel size for actuator response
static double *spotsize;
static long      fpi_spotsize = -1;


// DM array x size
static uint32_t *dmxsize;
static long fpi_dmxsize;

// DM array y size
static uint32_t *dmysize;
static long fpi_dmysize;




static CLICMDARGDEF farg[] =
{
    {
        CLIARG_IMG,
        ".in_name",
        "input image",
        "im1",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &inimname,
        NULL
    },
    {
        CLIARG_FLOAT64,
        ".spotsize",
        "spot size",
        "3.0",
        CLIARG_HIDDEN_DEFAULT,
        (void **) &spotsize,
        &fpi_spotsize
    },
    {   // DM x size
        CLIARG_UINT32,
        ".dmxsize",
        "DM x size",
        "50",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmxsize,
        &fpi_dmxsize
    },
    {   // DM y size
        CLIARG_UINT32,
        ".dmysize",
        "DM y size",
        "50",
        CLIARG_VISIBLE_DEFAULT,
        (void **) &dmysize,
        &fpi_dmysize
    }
};



// Optional custom configuration setup.
// Runs once at conf startup
//
static errno_t customCONFsetup()
{
    if(data.fpsptr != NULL)
    {

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
    "pyWFSgridmatch", "Match pyramid WFS zrespM to grid", CLICMD_FIELDS_DEFAULTS
};

// detailed help
static errno_t help_function()
{
    return RETURN_SUCCESS;
}



// compute spot positions from grid params
static int compute_grid_spotpos(
    pyrWFSgrid grid,
    float* spotxpos,
    float* spotypos
)
{
    int nbspot = 0;

    for(uint32_t dmii=0; dmii<grid.dmysize; dmii++)
    {
        for(uint32_t dmjj=0; dmjj<grid.dmysize; dmjj++)
        {

            

            for(int spotindex = 0; spotindex<4; spotindex++)
            {
                float spotx = grid.pupsquare_center_x;
                float spoty = grid.pupsquare_center_y;

                float spotxoffset = 0.0;
                float spotyoffset = 0.0;



                switch (spotindex)
                {
                case 0: // bottom left
                    spotxoffset = -grid.pupsquare_xoffset;
                    spotyoffset = -grid.pupsquare_yoffset;
                    break;

                case 1: // top left
                    spotxoffset = -grid.pupsquare_xoffset;
                    spotyoffset = grid.pupsquare_yoffset;
                    break;

                case 2: // bottom right
                    spotxoffset = grid.pupsquare_xoffset;
                    spotyoffset = -grid.pupsquare_yoffset;
                    break;

                case 3: // top right
                    spotxoffset = grid.pupsquare_xoffset;
                    spotyoffset = grid.pupsquare_yoffset;
                    break;

                default:
                    break;
                }

                // apply rotation
                spotx += spotxoffset * cos(grid.pupsquare_angle) - spotyoffset * sin(grid.pupsquare_angle);
                spoty += spotxoffset * sin(grid.pupsquare_angle) + spotyoffset * cos(grid.pupsquare_angle);

                spotxpos[(dmjj * grid.dmxsize + dmii) * 4 + spotindex] = spotx;
                spotypos[(dmjj * grid.dmxsize + dmii) * 4 + spotindex] = spoty;


                nbspot++;
            }
        }
    }

    return nbspot;
}


static double eval_gridmatch(
    pyrWFSgrid grid,
    float* spotxpos,
    float* spotypos,
    float* spotw
)
{
    double val = 0.0;

    float* gridspotxpos = (float*) malloc(sizeof(float) * grid.dmxsize * grid.dmysize * 4);
    float* gridspotypos = (float*) malloc(sizeof(float) * grid.dmxsize * grid.dmysize * 4);

    // Compute spots pos for grid
    compute_grid_spotpos(grid, gridspotxpos, gridspotypos);

    for(uint32_t dmii=0; dmii<grid.dmysize; dmii++)
    {
        for(uint32_t dmjj=0; dmjj<grid.dmysize; dmjj++)
        {

            for(int spotindex = 0; spotindex<4; spotindex++)
            {
                int index = (dmjj * grid.dmxsize + dmii) * 4 + spotindex;

                float dx = spotxpos[index] - gridspotxpos[index];
                float dy = spotypos[index] - gridspotypos[index];
                val += (dx*dx + dy*dy)*spotw[index];
            }
        }
    }

    free(gridspotxpos);
    free(gridspotypos);

    return val;
}







static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    // resolve image and create IMGID
    IMGID zrmimg = mkIMGID_from_name(inimname);
    resolveIMGID(&zrmimg, ERRMODE_ABORT);

    //printf("naxes = %d\n", zrmimg.md->naxis);



    INSERT_STD_PROCINFO_COMPUTEFUNC_START
    {
        uint32_t xsize = zrmimg.md->size[0];
        uint32_t ysize = zrmimg.md->size[1];
        // xysize is product of both
        uint32_t xysize = xsize * ysize;

        // allocate mem for working array to detect peaks
        float *imarray = (float *) malloc(sizeof(float) * xysize);
        float *imcarray = (float *) malloc(sizeof(float) * xysize);

        int ksizeint = (int) *spotsize;

        double pixval2sum_max = 0.0;
        double *pixval2sum = (double *) malloc(sizeof(double) * zrmimg.md->size[2]);

        // for each slice of zrm
        for(uint32_t slice = 0; slice < zrmimg.md->size[2]; slice++)
        {
            pixval2sum[slice] = 0.0;

            // scan 2D pixels oc imarray for current slize
            for(uint32_t ii = 0; ii < xsize; ii++)
            {
                for(uint32_t jj = 0; jj < ysize; jj++)
                {
                    float pixval = zrmimg.im->array.F[slice * xysize + jj * xsize + ii];
                    float pixval2 = pixval*pixval;
                    pixval2sum[slice] += pixval2;
                }
            }
            if(pixval2sum[slice] > pixval2sum_max)
            {
                pixval2sum_max = pixval2sum[slice];
            }
        }


        // spot positions and values
        float *spotpos_x = (float *) malloc(sizeof(float) * *dmxsize * *dmysize * 4);
        float *spotpos_y = (float *) malloc(sizeof(float) * *dmxsize * *dmysize * 4);
        float *spotval = (float*) malloc(sizeof(float) * *dmxsize * *dmysize * 4);



        int sliceproc_cnt = 0;
        int sliceskip_cnt = 0;

        for(uint32_t dmii=0; dmii<*dmysize; dmii++)
        {
            for(uint32_t dmjj=0; dmjj<*dmysize; dmjj++)
            {
                uint32_t slice = dmjj * *dmxsize + dmii;

                if(pixval2sum[slice] > 0.1*pixval2sum_max)
                {
                    //printf("Processing DM actuator %3u x %3u (slice %5u)\n", dmii, dmjj, slice);
                    //fflush(stdout);

                    // scan 2D pixels of imarray for current slize
                    for(uint32_t ii = 0; ii < xsize; ii++)
                    {
                        for(uint32_t jj = 0; jj < ysize; jj++)
                        {
                            float pixval = zrmimg.im->array.F[slice * xysize + jj * xsize + ii];
                            float pixval2 = pixval*pixval;
                            imarray[jj * xsize + ii] = pixval2;
                            imcarray[jj * xsize + ii] = 0.0;
                        }
                    }

                    // sum over ksizeint radius
                    //printf("    Convolve\n");
                    //fflush(stdout);

                    for(uint32_t ii = 0; ii < xsize; ii++)
                    {
                        int ii1min = ii - ksizeint;
                        if(ii1min < 0)
                        {
                            ii1min = 0;
                        }
                        int ii1max = ii + ksizeint;
                        if(ii1max > (int) xsize)
                        {
                            ii1max = xsize;
                        }
                        for(uint32_t jj = 0; jj < ysize; jj++)
                        {
                            int jj1min = jj - ksizeint;
                            if(jj1min < 0)
                            {
                                jj1min = 0;
                            }
                            int jj1max = jj + ksizeint;
                            if(jj1max > (int) ysize)
                            {
                                jj1max = ysize;
                            }

                            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
                            {
                                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                                {
                                    imcarray[jj * xsize + ii] += imarray[jj1 * xsize + ii1];
                                }
                            }
                        }
                    }

                    // look for spots
                    //printf("    Find spots\n");
                    //fflush(stdout);

                    for(int spotindex = 0; spotindex<4; spotindex++)
                    {
                        //printf("        processing quadrant %d\n", spotindex);
                        //fflush(stdout);

                        uint32_t iistart = 0;
                        uint32_t jjstart = 0;
                        uint32_t iistop = xsize;
                        uint32_t jjstop = ysize;

                        switch (spotindex)
                        {
                        case 0: // bottom left
                            iistart = 0;
                            iistop = xsize/2;
                            jjstart = 0;
                            jjstop = ysize/2;
                            break;

                        case 1: // top left
                            iistart = 0;
                            iistop = xsize/2;
                            jjstart = ysize/2;
                            jjstop = ysize;
                            break;

                        case 2: // bottom right
                            iistart = xsize/2;
                            iistop = xsize;
                            jjstart = 0;
                            jjstop = ysize/2;
                            break;

                        case 3: // top right
                            iistart = xsize/2;
                            iistop = xsize;
                            jjstart = ysize/2;
                            jjstop = ysize;
                            break;

                        default:
                            break;
                        }

                        // Find strongest peak
                        //
                        float vpeak = 0.0;
                        uint32_t iipeak = 0;
                        uint32_t jjpeak = 0;
                        for(uint32_t ii = iistart; ii < iistop; ii++)
                        {
                            for(uint32_t jj = jjstart; jj < jjstop; jj++)
                            {
                                if(imcarray[jj * xsize + ii] > vpeak)
                                {
                                    iipeak = ii;
                                    jjpeak = jj;
                                    vpeak = imcarray[jj * xsize + ii];
                                }
                            }
                        }
                        //printf("        SPOT %2d   %4u x %4u    %f\n", spotindex, iipeak, jjpeak, vpeak);
                        //fflush(stdout);



                        // measure spot photocenter
                        {
                            double xsum = 0.0;
                            double ysum = 0.0;
                            double wsum = 0.0;

                            int ii1min = iipeak - (int)(*spotsize + 1);
                            if(ii1min < 0)
                            {
                                ii1min = 0;
                            }
                            int ii1max = iipeak + (int)(*spotsize + 1);
                            if(ii1max > (int) xsize)
                            {
                                ii1max = xsize;
                            }

                            int jj1min = jjpeak - (int)(*spotsize + 1);
                            if(jj1min < 0)
                            {
                                jj1min = 0;
                            }
                            int jj1max = jjpeak + (int)(*spotsize + 1);
                            if(jj1max > (int) ysize)
                            {
                                jj1max = ysize;
                            }
                            //printf("        range = [%d, %d] x [%d, %d]\n", ii1min, ii1max, jj1min, jj1max);
                            //fflush(stdout);

                            for(int ii1 = ii1min; ii1 < ii1max; ii1++)
                            {
                                for(int jj1 = jj1min; jj1 < jj1max; jj1++)
                                {
                                    float dx = 1.0 * iipeak - ii1;
                                    float dy = 1.0 * jjpeak - jj1;
                                    float r2 = dx * dx + dy * dy;
                                    if (r2 < *spotsize * *spotsize)
                                    {
                                        float val = imcarray[jj1 * xsize + ii1];
                                        xsum += val * ii1;
                                        ysum += val * jj1;
                                        wsum += val;
                                    }
                                }
                            }

                            printf("[%5u %5u] SPOT %2d", dmii, dmjj, spotindex);
                            printf("      %8.3f  %8.3f    %g\n", xsum/wsum, ysum/wsum, wsum);
                            fflush(stdout);


                            spotpos_x[slice * 4 + spotindex] = xsum/wsum;
                            spotpos_y[slice * 4 + spotindex] = ysum/wsum;
                            spotval[slice * 4 + spotindex] = wsum;
                        }

                    }


                    sliceproc_cnt++;
                    printf("\n");
                }
                else
                {
                    //printf("Skipping   DM actuator %3u x %3u (slice %5u)\n", dmii, dmjj, slice);
                    //fflush(stdout);
                    for(int spoti=0; spoti<4; spoti++)
                    {
                        spotpos_x[slice * 4 + spoti] = 0.0;
                        spotpos_y[slice * 4 + spoti] = 0.0;
                        spotval[slice * 4 + spoti] = 0.0;
                    }
                    sliceskip_cnt++;
                }
            }
        }
        // print number of slices processed and skipped
        printf("slices processed: %d, skipped: %d\n", sliceproc_cnt, sliceskip_cnt);
        fflush(stdout);

        free(pixval2sum);
        free(imcarray);

        // fit to model
        //
        pyrWFSgrid grid;
        pyrWFSgrid gridbest;
        pyrWFSgrid gridstep;

        grid.dmxsize = *dmxsize;
        grid.dmysize = *dmysize;


        grid.pupsquare_xoffset = 0.4 * xsize;
        grid.pupsquare_yoffset = 0.4 * ysize;
        grid.pupsquare_angle = 0.0;
        grid.pupsquare_center_x = 0.5*xsize;
        grid.pupsquare_center_y = 0.5*ysize;

        grid.actpitch_x = (0.4*xsize)/(*dmxsize);
        grid.actpitch_y = (0.4*xsize)/(*dmxsize);
        grid.actpitch_angle = 0.0;

        // copy grid to gridbest
        memcpy(&gridbest, &grid, sizeof(pyrWFSgrid));

        //float* gridspotpos_x = (float*) malloc(sizeof(float) * *dmxsize * *dmysize * 4);
        //float* gridspotpos_y = (float*) malloc(sizeof(float) * *dmxsize * *dmysize * 4);

        // Compute spots pos for grid
        //compute_grid_spotpos(grid, gridspotpos_x, gridspotpos_y);

        double gridvalbest = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
        printf("grid value = %g\n", gridvalbest);
        fflush(stdout);

        gridstep.pupsquare_xoffset = 0.2;
        gridstep.pupsquare_yoffset = 0.2;
        gridstep.pupsquare_angle = 0.1;
        gridstep.pupsquare_center_x = 0.01*xsize;
        gridstep.pupsquare_center_y = 0.01*ysize;

        gridstep.actpitch_x = 0.5;
        gridstep.actpitch_y = 0.5;
        gridstep.actpitch_angle = 0.1;

        double gridvaleps = 1.0e-6;

        {
            // optimize pupsquare_xoffset
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.pupsquare_xoffset += direction * gridstep.pupsquare_xoffset;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("pupsquare_xoffset dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.pupsquare_xoffset, gridbest.pupsquare_xoffset, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.pupsquare_xoffset = grid.pupsquare_xoffset;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.pupsquare_xoffset *= 0.3;
        }


        {
            // optimize pupsquare_yoffset
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.pupsquare_yoffset += direction * gridstep.pupsquare_yoffset;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("pupsquare_yoffset dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.pupsquare_yoffset, gridbest.pupsquare_yoffset, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.pupsquare_yoffset = grid.pupsquare_yoffset;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.pupsquare_yoffset *= 0.3;
        }

        {
            // optimize pupsquare_angle
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.pupsquare_angle += direction * gridstep.pupsquare_angle;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("pupsquare_angle dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.pupsquare_angle, gridbest.pupsquare_angle, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.pupsquare_angle = grid.pupsquare_angle;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.pupsquare_angle *= 0.3;
        }

        {
            // optimize pupsquare_center_x
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.pupsquare_center_x += direction * gridstep.pupsquare_center_x;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("pupsquare_center_x dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.pupsquare_center_x, gridbest.pupsquare_center_x, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.pupsquare_center_x = grid.pupsquare_center_x;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.pupsquare_center_x *= 0.3;
        }


        {
            // optimize pupsquare_center_y
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.pupsquare_center_y += direction * gridstep.pupsquare_center_y;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("pupsquare_center_y dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.pupsquare_center_y, gridbest.pupsquare_center_y, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.pupsquare_center_y = grid.pupsquare_center_y;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.pupsquare_center_y *= 0.3;
        }


        {
            // optimize actpitch_x
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.actpitch_x += direction * gridstep.actpitch_x;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                printf(" (( %g %g  %g )) ", gridval_old, gridval_new, gridval_new - gridval_new);

                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    printf("[FLIP %d]  ", dirflipcnt);
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("actpitch_x dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.actpitch_x, gridbest.actpitch_x, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.actpitch_x = grid.actpitch_x;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.actpitch_x *= 0.3;
        }
 


        {
            // optimize actpitch_y
            int direction = 1;
            int dirflipcnt = 0;
            long loopcnt = 0;

            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.actpitch_y += direction * gridstep.actpitch_y;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("actpitch_y dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.actpitch_y, gridbest.actpitch_y, gridval_new, gridvalbest);
                gridval_old = gridval_new;

                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.actpitch_y = grid.actpitch_y;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.actpitch_y *= 0.3;
        }

        {
            // optimize actpitch_angle
            int direction = 1;
            int dirflipcnt = 0;


            double gridval_old = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
            while((dirflipcnt < 2)&&(loopcnt < 1000))
            {
                grid.actpitch_angle += direction * gridstep.actpitch_angle;
                double gridval_new = eval_gridmatch(grid, spotpos_x, spotpos_y, spotval);
                if(gridval_new > gridval_old - gridvaleps)
                {
                    // flip direction
                    direction *= -1;
                    dirflipcnt++;
                }
                printf("actpitch_angle dir%+2d %6.3f [%6.3f]   %8.3f [%8.3f]  ", direction, grid.actpitch_angle, gridbest.actpitch_angle, gridval_new, gridvalbest);
                gridval_old = gridval_new;
                if(gridval_new < gridvalbest)
                {
                    printf(" -> UPDATE");
                    gridvalbest = gridval_new;
                    gridbest.actpitch_angle = grid.actpitch_angle;
                }
                printf("\n");
                loopcnt++;
            }
            gridstep.actpitch_angle *= 0.3;
        }








        free(spotpos_x);
        free(spotpos_y);
        free(imarray);
        free(spotval);


    }
    INSERT_STD_PROCINFO_COMPUTEFUNC_END


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}



INSERT_STD_FPSCLIfunctions




// Register function in CLI
errno_t
CLIADDCMD_cacao_pyramidWFStools__pyWFSgridmatch()
{

    CLIcmddata.FPS_customCONFsetup = customCONFsetup;
    CLIcmddata.FPS_customCONFcheck = customCONFcheck;
    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
