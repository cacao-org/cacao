/**
 * @file    AOloopControl_DM_turbusimul.c
 * @brief   DM control
 * 
 * To be used for AOloopControl module
 * 
 */

#include <math.h>
#include <sys/file.h>
#include <sys/mman.h>

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl_DM/AOloopControl_DM.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "image_gen/image_gen.h"
#include "image_filter/image_filter.h"
#include "image_basic/image_basic.h"

#ifdef __MACH__
#include <mach/mach_time.h>
#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 0
int clock_gettime(int clk_id, struct mach_timespec *t){
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



extern long NB_DMindex ;

extern AOLOOPCONTROL_DM_DISPCOMB_CONF *dmdispcombconf; // configuration
extern int dmdispcomb_loaded ;
extern int SMfd;


extern AOLOOPCONTROL_DMTURBCONF *dmturbconf; // DM turbulence configuration
extern int dmturb_loaded ;
extern int SMturbfd;




/* =============================================================================================== */
/* =============================================================================================== */
/*                                                                                                 */
/* 4. TURBULENCE SIMULATOR                                                                         */
/*                                                                                                 */
/* =============================================================================================== */
/* =============================================================================================== */




errno_t AOloopControl_printDMturbconf()
{
    long DMindex;
    
    if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

//    AOloopControl_DMturb_loadconf(0);
    
    printf("ind on  ampl [um]  tint [us]  simtime [s]  wspeed [m/s]  LOcoeff\n");
    for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
            printf("%ld  %d  %10f  %10ld  %10f %5f  %5f\n", DMindex, dmturbconf[DMindex].on, dmturbconf[DMindex].ampl, dmturbconf[DMindex].tint, dmturbconf[DMindex].simtime, dmturbconf[DMindex].wspeed, dmturbconf[DMindex].LOcoeff);
        }
    
    return 0;
}




//
// create configuration shared memory structure for moving turbulence screen for DM
// one configuration per DM
//
errno_t AOloopControl_DMturb_createconf()
{
    int result;
    //    imageID IDc1;
    imageID DMindex;
    char errstr[STRINGMAXLEN_DEFAULT];

    printf("ENTERING FUNCTION AOloopControl_DMturb_createconf\n");
    fflush(stdout);

    if( dmdispcomb_loaded == 0 )
    {
        printf("============== AOloopControl_DM_loadconf\n");
        fflush(stdout);
        AOloopControl_DM_loadconf();
        printf("=====>\n");
        fflush(stdout);
    }


    if( dmturb_loaded == 0 )
    {
        printf("=============== AOloopControl_DMturb_loadconf\n");
        fflush(stdout);
        AOloopControl_DMturb_loadconf();
        printf("=====>\n");
        fflush(stdout);
    }


    if( dmturb_loaded == 0 )
    {
        printf("Create/read DMturb configuration\n");
        fflush(stdout);

        SMturbfd = open(DMTURBCONF_FILENAME, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
        if (SMturbfd == -1) {
            snprintf(errstr, STRINGMAXLEN_DEFAULT,
                     "Error opening (O_RDWR | O_CREAT | O_TRUNC) file \"%s\"", DMTURBCONF_FILENAME);
            perror(errstr);
            exit(EXIT_FAILURE);
        }

        result = lseek(SMturbfd, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex-1, SEEK_SET);
        if (result == -1) {
            close(SMturbfd);
            perror("Error calling lseek() to 'stretch' the file");
            exit(EXIT_FAILURE);
        }

        result = write(SMturbfd, "", 1);
        if (result != 1) {
            close(SMturbfd);
            perror("Error writing last byte of the file");
            exit(EXIT_FAILURE);
        }

        dmturbconf = (AOLOOPCONTROL_DMTURBCONF*)mmap(0, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMturbfd, 0);
        if (dmturbconf == MAP_FAILED) {
            close(SMturbfd);
            perror("Error mmapping the file");
            exit(EXIT_FAILURE);
        }


        for(DMindex=0; DMindex<NB_DMindex; DMindex++)
        {
            dmturbconf[DMindex].on = 0;

            dmturbconf[DMindex].wspeed = 10.0; // [m/s]
            dmturbconf[DMindex].ampl = 0.01; // [um]
            dmturbconf[DMindex].LOcoeff = 0.2;

            dmturbconf[DMindex].tint = 100; // [us]

            dmturbconf[DMindex].simtime = 0.0; // sec
        }
        dmturb_loaded = 1;

    }
    AOloopControl_printDMturbconf();

    printf("EXITING FUNCTION AOloopControl_DMturb_createconf\n");
    fflush(stdout);

    return RETURN_SUCCESS;
}








errno_t AOloopControl_DMturb_loadconf()
{
    char errstr[200];

    if( dmturb_loaded == 0 )
    {
        printf("Read configuration\n");

        SMturbfd = open(DMTURBCONF_FILENAME, O_RDWR, (mode_t)0600);
        if (SMturbfd == -1) {
            sprintf(errstr, "Error opening (O_RDWR) file \"%s\" in function AOloopControl_DMturb_loadconf", DMTURBCONF_FILENAME);
            perror(errstr);
		}
        else
        {
        //    exit(EXIT_FAILURE);
        

			dmturbconf = (AOLOOPCONTROL_DMTURBCONF*)mmap(0, sizeof(AOLOOPCONTROL_DMTURBCONF)*NB_DMindex, PROT_READ | PROT_WRITE, MAP_SHARED, SMturbfd, 0);
			if (dmturbconf == MAP_FAILED) {
				close(SMturbfd);
				printf("Error mmapping the file -> creating it\n");
				AOloopControl_DMturb_createconf();
			}
			dmturb_loaded = 1;
		}
	}

    return RETURN_SUCCESS;
}





int AOloopControl_DM_dmturboff(long DMindex)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

    dmturbconf[DMindex].on = 0;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_wspeed(long DMindex, double wspeed)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].wspeed = wspeed;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_ampl(long DMindex, double ampl)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].ampl = ampl;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_LOcoeff(long DMindex, double LOcoeff)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();

    dmturbconf[DMindex].LOcoeff = LOcoeff;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}

int AOloopControl_DM_dmturb_tint(long DMindex, long tint)
{
//    AOloopControl_DMturb_loadconf(DMindex);
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
    
    dmturbconf[DMindex].tint = tint;
    AOloopControl_DM_dmturb_printstatus(DMindex);

    return 0;
}



int AOloopControl_DM_dmturb_printstatus(long DMindex)
{
	if( dmturb_loaded == 0 )
		AOloopControl_DMturb_createconf();
//    AOloopControl_DMturb_loadconf(DMindex);

    printf("Run time = %.3f sec\n", dmturbconf[DMindex].simtime);
    printf("\n");
    printf("cnt              : %ld   (ave frequ = %.2f kHz)\n", dmturbconf[DMindex].cnt, 0.001*dmturbconf[DMindex].cnt/dmturbconf[DMindex].simtime);
    printf("\n");

    if(dmturbconf[DMindex].on == 1)
        printf("LOOP IS ON\n");
    else
        printf("LOOP IS OFF\n");

    printf("ampl    =  %.2f um\n", dmturbconf[DMindex].ampl);
    printf("wspeed  =  %.2f m/s\n", dmturbconf[DMindex].wspeed);
    printf("tint    =  %ld us\n", dmturbconf[DMindex].tint);
    printf("LOcoeff =  %.2f\n", dmturbconf[DMindex].LOcoeff);
    printf("Requested uptdate frequ = %.2f kHz\n", 0.001/(1.0e-6*dmturbconf[DMindex].tint));
    printf("\n");
    printf("\n");

    return(0);
}










// mode = 0 : send to DM
// mode = 1 : write to file, so that it can be later sent to DM


int AOloopControl_DM_dmturb(long DMindex, int mode, const char *IDout_name, long NBsamples)
{
	float DMsizeM = 10.0; // DM size in meter
    long size_sx; // screen size
    long size_sy;
    long IDs1, IDs2;
    char name[200];
    long imsize = 2048;
    struct timespec tlast;
    struct timespec tdiff;
    struct timespec tdiff1;
    double tdiff1v;

    float screen0_X;
    float screen0_Y;
    long ii, jj, ii1;
    float x, y;
    float xpix, ypix;
    float xpixf, ypixf;
    long xpix1, xpix2, ypix1, ypix2;
    float ave;

    double angle = 1.0;
    double coeff = 0.001;
    double x1, fx1;
    
    double RMSval;
    long RMSvalcnt;
    double r;
    
    float pixscale = 0.1; // [m/pix]
    // Subaru pupil ~ 80 pix diam
    // Single actuator ~7 pix
    
    long DM_Xsize, DM_Ysize;

    long IDturbs1;
    long IDturb;
    double totim;
    long IDk;

	long k0 = 100;
	int k0init = 0;
	
	long k;
	int turbON;
	long IDout;
	double dX, dY;
	double wspeedx, wspeedy;
	double RMSvaltot;
	long RMSvaltotcnt;


	int tint;

	if( dmturb_loaded == 0 )
	{
		printf("========= START AOloopControl_DMturb_createconf\n");
		fflush(stdout);
		AOloopControl_DMturb_createconf();
		printf("========= END AOloopControl_DMturb_createconf\n");
		fflush(stdout);
	}
	
    IDs1 = load_fits("turbscreen1.fits", "screen1", 1);
    IDs2 = load_fits("turbscreen2.fits", "screen2", 1);
    list_image_ID();
    
    if(IDs1==-1)
    {
        make_master_turbulence_screen_local("screen1", "screen2", imsize, 200.0, 1.0);
        IDs1 = image_ID("screen1");
        IDk = make_gauss("kernim", imsize, imsize, 20.0, 1.0);
        totim = 0.0;
        for(ii=0;ii<imsize*imsize;ii++)
            totim += data.image[IDk].array.F[ii];
        for(ii=0;ii<imsize*imsize;ii++)
            data.image[IDk].array.F[ii] /= totim;
        IDs2 = fconvolve("screen1", "kernim", "screen2");
        delete_image_ID("kernim");
        save_fits("screen1", "!turbscreen1.fits");
        save_fits("screen2", "!turbscreen2.fits");
    }
    

    printf("ARRAY SIZE = %ld %ld\n", (long) data.image[IDs1].md[0].size[0], (long) data.image[IDs1].md[0].size[1]);
    size_sx = data.image[IDs1].md[0].size[0];
    size_sy = data.image[IDs1].md[0].size[1];

	if(mode==0)
	{
		clock_gettime(CLOCK_REALTIME, &dmturbconf[DMindex].tstart);
		dmturbconf[DMindex].tend = dmturbconf[DMindex].tstart;
	}

    DM_Xsize = dmdispcombconf[DMindex].xsize;
    DM_Ysize = dmdispcombconf[DMindex].ysize;
    printf("DM %ld array size : %ld %ld\n", DMindex, DM_Xsize, DM_Ysize);
    list_image_ID();
    sprintf(name, "dm%02lddisp10", DMindex);
    read_sharedmem_image(name);
    list_image_ID();
    
    if(mode==0)
		dmturbconf[DMindex].on = 1;

    IDturbs1 = create_2Dimage_ID("turbs1", DM_Xsize, DM_Ysize);
    IDturb = create_2Dimage_ID("turbs", DM_Xsize, DM_Ysize);


	if(mode==1)
		IDout = create_3Dimage_ID(IDout_name, DM_Xsize, DM_Ysize, NBsamples);
		

	k = 0;
	if(mode == 0)
		turbON = dmturbconf[DMindex].on;
	else
		turbON = 1;
		
	printf("MODE = %d\n  DMindex = %ld", mode, DMindex);
	
	
	if(mode==1) // force periodic sequence if wind speed is sufficiently large
	{
		printf("Wind speed = %f m/s\n", dmturbconf[DMindex].wspeed);
		printf("angle      = %f rad\n", angle);
		printf("time interval = %.6f sec\n", 1.0e-6*dmturbconf[DMindex].tint);
		printf("NBsamples = %ld\n", NBsamples);
		printf("sequence time = %f sec\n", 1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		dX = dmturbconf[DMindex].wspeed*1.0e-6*dmturbconf[DMindex].tint*NBsamples*cos(angle);
		dY = dmturbconf[DMindex].wspeed*1.0e-6*dmturbconf[DMindex].tint*NBsamples*sin(angle);
		printf("dX x dY  =    %20f x %20f m\n", dX, dY);
		printf("turb screen size = %f m\n", size_sx*pixscale);
		printf("->   %10.5f  x  %10.5f screen\n", dX/(size_sx*pixscale), dY/(size_sy*pixscale));
	
		dX = (floor( dX/(size_sx*pixscale) + 1000.5 ) - 1000.0) * size_sx*pixscale;
		dY = (floor( dY/(size_sy*pixscale) + 1000.5 ) - 1000.0) * size_sy*pixscale;

		printf("dX x dY  =    %20f x %20f m\n", dX, dY);
		wspeedx = dX / (1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		wspeedy = dY / (1.0e-6*dmturbconf[DMindex].tint*NBsamples);
		
		if(sqrt(wspeedx*wspeedx+wspeedy*wspeedy)<0.0001)
			{				
				wspeedx = dmturbconf[DMindex].wspeed*cos(angle);
				wspeedy = dmturbconf[DMindex].wspeed*sin(angle);
			}
		
		printf("wspeed = %f x %f m/s  -> %f m/s\n", wspeedx, wspeedy, sqrt(wspeedx*wspeedx+wspeedy*wspeedy));
	}

	
//	fp = fopen("test.txt", "w");
	
	RMSvaltot = 0.0;
	RMSvaltotcnt = 0;
	tint = dmturbconf[DMindex].tint;
	
    while(turbON == 1) // computation loop
    {

		if(mode==0)
		{
			usleep(dmturbconf[DMindex].tint);

			tlast = dmturbconf[DMindex].tend;
			clock_gettime(CLOCK_REALTIME, &dmturbconf[DMindex].tend);
			tdiff = time_diff(dmturbconf[DMindex].tstart, dmturbconf[DMindex].tend);
			tdiff1 =  time_diff(tlast, dmturbconf[DMindex].tend);
			tdiff1v = 1.0*tdiff1.tv_sec + 1.0e-9*tdiff1.tv_nsec;
		
			 dmturbconf[DMindex].simtime = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;
		}
		else
		{
			tdiff1v = 1.0e-6*tint*k;
			dmturbconf[DMindex].simtime = tdiff1v;
		}
		
		if(mode == 0)
		{
			screen0_X += dmturbconf[DMindex].wspeed*tdiff1v*cos(angle); // [m]
			screen0_Y += dmturbconf[DMindex].wspeed*tdiff1v*sin(angle); // [m]
		}
		else
		{
			screen0_X = wspeedx*tdiff1v; // [m]
			screen0_Y = wspeedy*tdiff1v; // [m]
		}

		

        //dmturbconf[DMindex].simtime = 1.0*tdiff.tv_sec + 1.0e-9*tdiff.tv_nsec;


        for(ii=0; ii<DM_Xsize; ii++)
            for(jj=0; jj<DM_Ysize; jj++)
            {
                ii1 = jj*DM_Xsize+ii;

                x = DMsizeM*ii/DM_Xsize + screen0_X; // [m]
                y = DMsizeM*jj/DM_Ysize + screen0_Y; // [m]

                xpix = 0.5*size_sx + x/pixscale;
                ypix = 0.5*size_sy + y/pixscale;

                xpix1 = ((long) xpix)%size_sx;
                xpix2 = (xpix1+1)%size_sx;
                xpixf = xpix- (long) xpix;
                ypix1 = ((long) ypix)%size_sy;
                ypix2 = (ypix1+1)%size_sy;
                ypixf = ypix - (long) ypix;

                while(xpix1<0)
                    xpix1 = 0;
                while(xpix1>size_sx-1)
                    xpix1 = size_sx-1;

                if(ypix1<0)
                    ypix1 = 0;
                if(ypix1>size_sy-1)
                    ypix1 = size_sy-1;

			//	if((k%100==0))
			//	{
			//		if(((ii==0)&&(jj==0))) //||((ii==DM_Xsize-1)&&(jj==DM_Ysize-1)))
			//			printf("%05ld %20f (%6ld)   %5ld %5ld   %20f %20f   %4ld %4ld   %ld  [%ld %ld] [%ld %ld]\n", k, tdiff1v, dmturbconf[DMindex].tint, ii, jj, screen0_X, screen0_Y, xpix1, ypix1, IDs1, DM_Xsize, DM_Ysize, size_sx, size_sy);
			//	}

				
                data.image[IDturbs1].array.F[ii1] = 1.0*xpix1;

                data.image[IDturb].array.F[ii1] = (1.0-xpixf)*(1.0-ypixf)*(data.image[IDs1].array.F[ypix1*size_sx+xpix1] - (1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix1*size_sx+xpix1]);
                data.image[IDturb].array.F[ii1]  +=  (xpixf)*(1.0-ypixf)*(data.image[IDs1].array.F[ypix1*size_sx+xpix2]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix1*size_sx+xpix2]);
                data.image[IDturb].array.F[ii1]  += (1.0-xpixf)*(ypixf)*(data.image[IDs1].array.F[ypix2*size_sx+xpix1]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix2*size_sx+xpix1]);
                data.image[IDturb].array.F[ii1]  += xpixf*ypixf*(data.image[IDs1].array.F[ypix2*size_sx+xpix2]-(1.0-dmturbconf[DMindex].LOcoeff)*data.image[IDs2].array.F[ypix2*size_sx+xpix2]);
            }

        // proccess array
        
        ave = 0.0;
        for(ii1=0; ii1<DM_Xsize*DM_Ysize; ii1++)
            ave += data.image[IDturb].array.F[ii1];
        ave /= dmdispcombconf[DMindex].xysize;
        for(ii1=0; ii1<DM_Xsize*DM_Ysize; ii1++)
        {
            data.image[IDturb].array.F[ii1] -= ave;
            data.image[IDturb].array.F[ii1] *= coeff;
        }

        RMSval = 0.0;
        RMSvalcnt = 0;

        for(ii=0; ii<DM_Xsize; ii++)
            for(jj=0; jj<DM_Ysize; jj++)
            {
                ii1 = DM_Xsize*jj+ii;
                x = 0.5*DM_Xsize - 0.5 - ii;
                y = 0.5*DM_Ysize - 0.5 - jj;
                r = sqrt(x*x+y*y);
                if(r<DM_Xsize*0.5-1.0)
                {
                    RMSval += data.image[IDturb].array.F[ii1]*data.image[IDturb].array.F[ii1];
                    RMSvalcnt++;
                }
            }
        RMSval = sqrt(RMSval/RMSvalcnt);

	if(mode == 0)
	{
        x1 = log10(RMSval/dmturbconf[DMindex].ampl);
        fx1 = 1.0 + 50.0*exp(-5.0*x1*x1);
        coeff /= pow(10.0,x1/fx1);
	}
	else
	{	if(k0init==1)
		{
			RMSvaltot += RMSval;
			RMSvaltotcnt ++;
		}
	}
        
//        printf("STEP 001  %f %f\n", screen0_X, screen0_Y);
//        fflush(stdout);
        
		if(mode == 0)
		{
			sprintf(name, "dm%02lddisp10", DMindex);
			copy_image_ID("turbs", name, 0);
		}
		else
		{
			if(k0init==1)
			{
			//printf("STEP %5ld / %5ld       time = %12.6f    coeff = %18g   RMSval = %18g    %18f x %18f\n", k, NBsamples, tdiff1v, coeff, RMSval, screen0_X, screen0_Y);
			//fflush(stdout);
			//fprintf(fp, "%5ld  %12.6f      %18g     %18g    %18f  %18f  %18f\n", k, tdiff1v, coeff, RMSval, screen0_X, screen0_Y, dmturbconf[DMindex].wspeed);
			
			for(ii=0;ii<DM_Xsize*DM_Ysize;ii++)
				data.image[IDout].array.F[k*DM_Xsize*DM_Ysize+ii] = data.image[IDturb].array.F[ii];
			}
		//	usleep(dmturbconf[DMindex].tint);
		//	sprintf(name, "dm%02lddisp10", DMindex);
		//	copy_image_ID("turbs", name, 0);
		}
   
   //     save_fits("turbs", "!turbs.fits");
   //     save_fits("turbs1", "!turbs1.fits");
    
		
		if(mode==0)
			turbON = dmturbconf[DMindex].on;
		else
			{
				k ++;
				if((k==k0)&&(k0init==0))
				{
					printf("------ k->0 ----------\n");
					k0init = 1;
					k = 0;
				}
				
				if(k<NBsamples)
					turbON = 1;
				else
					turbON = 0;
			}
		
    }
	//fclose(fp);


	RMSval = RMSvaltot/RMSvaltotcnt;
	for(k=0;k<NBsamples;k++)
		for(ii=0;ii<DM_Xsize*DM_Ysize;ii++)
			data.image[IDout].array.F[k*DM_Xsize*DM_Ysize+ii] *= dmturbconf[DMindex].ampl/RMSval;


//	for(iter=0;iter<100;iter++)
	//	{
	AOloopControl_printDMturbconf();
		//	usleep(1000000);
	//	}

	//dmturbconf[DMindex].tint = tint;


    return(0);
}
