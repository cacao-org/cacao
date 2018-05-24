/**
 * @file    AOloopControl_RTstreamLOG.c 
 * @brief   Logging of real-time streams
 * 
 * Real-time stream logging
 *  
 * @author  O. Guyon
 *
 * 
 * 
 */



#define _GNU_SOURCE


#include <string.h>


#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"



#include "AOloopControl.h"
extern AOLOOPCONTROL_CONF *AOconf; 





int AOloopControl_RTstreamLOG_init(int loop)
{
	long i;
	
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
        {
			AOconf[loop].RTSLOGarray[i].active = 0;
			strcpy(AOconf[loop].RTSLOGarray[i].name, "NULL");
			AOconf[loop].RTSLOGarray[i].ENABLE = 0;
			AOconf[loop].RTSLOGarray[i].INIT = 0;
			AOconf[loop].RTSLOGarray[i].ON = 0;
			AOconf[loop].RTstreamLOG_wfsim_save = 0;
		 AOconf[loop].RTSLOGarray[i].saveToggle = 0;
        }
	
	       i = RTSLOGindex_wfsim;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "wfsim");
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
     
        i = RTSLOGindex_imWFS0;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS0");
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
     
        i = RTSLOGindex_imWFS1;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS1");
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
     
        i = RTSLOGindex_imWFS2;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS2");
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
        
        i = RTSLOGindex_modeval;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval"); // U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
                
        i = RTSLOGindex_modeval_ol;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_ol");// U 
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
        AOconf[loop].RTSLOGarray[i].ON = 1;                

        i = RTSLOGindex_modeval_dm;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
                
        i = RTSLOGindex_modeval_dm_corr;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_corr");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
                        
        i = RTSLOGindex_modeval_dm_now;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
                                
        i = RTSLOGindex_modeval_dm_now_filt;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now_filt");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;

        i = RTSLOGindex_modevalPF;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPF");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;
                                        
        i = RTSLOGindex_modevalPFsync;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFsync");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;        
                
        i = RTSLOGindex_modevalPFres;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFres");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;        

        i = RTSLOGindex_dmC;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "dmC");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 1;                        
	
	return 0;
}




int AOloopControl_RTstreamLOG_setup(long loop, long rtlindex, char *streamname)
{
	long IDstream;
	uint32_t *imsize;
	int retval = 0;
	char imname[500];
	
	long infosize = 5;
	
	IDstream = image_ID(streamname);
	imsize = (uint32_t*) malloc(sizeof(uint32_t)*3);
	imsize[0] = data.image[IDstream].md[0].size[0];
	imsize[1] = data.image[IDstream].md[0].size[1];
	imsize[2] = AOconf[loop].RTLOGsize;
	
	if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE == 1)&&(AOconf[loop].RTSLOGarray[rtlindex].INIT == 0))
	{	
		if(sprintf(imname, "aol%ld_%s_logbuff0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff0 = create_image_ID(imname, 3, imsize, _DATATYPE_FLOAT, 1, 0);

		if(sprintf(imname, "aol%ld_%s_logbuff1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff1 = create_image_ID(imname, 3, imsize, _DATATYPE_FLOAT, 1, 0);
		
		imsize[1] = infosize;
		imsize[2] = 1;
			
		if(sprintf(imname, "aol%ld_%s_logbuffinfo0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);

		if(sprintf(imname, "aol%ld_%s_logbuffinfo1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo1 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);

		
		AOconf[loop].RTSLOGarray[rtlindex].srcptr = data.image[IDstream].array.F;
		AOconf[loop].RTSLOGarray[rtlindex].memsize = sizeof(float)*imsize[0]*imsize[1];
		AOconf[loop].RTSLOGarray[rtlindex].destptr0 = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.F;
		AOconf[loop].RTSLOGarray[rtlindex].destptr1 = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.F;

		AOconf[loop].RTSLOGarray[rtlindex].buffindex = 0;
		AOconf[loop].RTSLOGarray[rtlindex].frameindex = 0;
		AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
		
		retval = 1;
		AOconf[loop].RTSLOGarray[rtlindex].INIT = 1;
	}
	
	free(imsize);
	
	return(retval);
}






void AOloopControl_RTstreamLOG_update(long loop, long rtlindex, struct timespec tnow)
{
	char *dataptr;

	
	if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE==1) && (AOconf[loop].RTSLOGarray[rtlindex].ON==1))
	{

		dataptr = AOconf[loop].RTSLOGarray[rtlindex].destptr;// + AOconf[loop].RTSLOGarray[rtlindex].memsize * AOconf[loop].RTSLOGarray[rtlindex].frameindex;

/*
		
		memcpy((void*) dataptr, 
		(void*) AOconf[loop].RTSLOGarray[rtlindex].srcptr, 
		AOconf[loop].RTSLOGarray[rtlindex].memsize);
*/


		long IDinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo;
		list_image_ID();
		printf("IDinfo = %ld\n", IDinfo);
		sleep(1000);//TEST
		
		
/*		
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5  ] = AOconf[loop].LOOPiteration;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+1] = (long) tnow.tv_sec;;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+2] = (long) tnow.tv_nsec;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+3] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt0;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+4] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt1;


		AOconf[loop].RTSLOGarray[rtlindex].frameindex++;
		if(AOconf[loop].RTSLOGarray[rtlindex].frameindex == AOconf[loop].RTLOGsize)
		{
			AOconf[loop].RTSLOGarray[rtlindex].frameindex = 0;
		
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].cnt0++;
			data.image[IDinfo].md[0].cnt0++;
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].write = 0;
			data.image[IDinfo].md[0].write = 0;
			COREMOD_MEMORY_image_set_sempost_byID(AOconf[loop].RTSLOGarray[rtlindex].IDbuff, -1);
			COREMOD_MEMORY_image_set_sempost_byID(IDinfo, -1);
		
			if(AOconf[loop].RTSLOGarray[rtlindex].buffindex==0)
			{
				AOconf[loop].RTSLOGarray[rtlindex].buffindex = 1;
				AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr1;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff1;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo1;
				AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 1;
			}
			else
			{
				AOconf[loop].RTSLOGarray[rtlindex].buffindex = 0;
				AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
				AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 2;
			}
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].write = 1;
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo].md[0].write = 1;
		
		}
		*/
		
	
	}
}




int AOloopControl_RTstreamLOG_printstatus(int loop)
{
	long NBstreams = 0;
	
	char ENstring[20];
	char ONstring[20];
	char INstring[20];
	char SAstring[20];
	int i;
	
	printf("\n");
	printf("RTLOGsize = %ld\n", AOconf[loop].RTLOGsize);
	printf("%2s  %20s  %3s %3s %3s %6s %4s %10s %10s\n", "id", "streamname", "ENA", " ON", "INI", "SAVE", "buff", "frame", "memsize");
	printf("----------------------------------------------------------\n");
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
	{
		if(AOconf[loop].RTSLOGarray[i].active == 1)
		{
			
			if(AOconf[loop].RTSLOGarray[i].ENABLE == 1)
				sprintf(ENstring, "\033[1;32m ON\033[0m");
			else
				sprintf(ENstring, "OFF");

			if(AOconf[loop].RTSLOGarray[i].ON == 1)
				sprintf(ONstring, "\033[1;32m ON\033[0m");
			else
				sprintf(ONstring, "OFF");
			
			if(AOconf[loop].RTSLOGarray[i].INIT == 1)
				sprintf(INstring, "\033[1;32m ON\033[0m");
			else
				sprintf(INstring, "OFF");

			if(AOconf[loop].RTSLOGarray[i].save == 1)
				sprintf(SAstring, "\033[1;32m ON[%1d]\033[0m", AOconf[loop].RTSLOGarray[i].saveToggle);
			else
				sprintf(SAstring, "OFF   ");						
  
  
			printf("%2d  %20s  %3s %3s %3s %6s %4d %10ld %10ld\n", i,  
			AOconf[loop].RTSLOGarray[i].name, 
			ENstring, 
			ONstring, 
			INstring, 
			SAstring,			
			AOconf[loop].RTSLOGarray[i].buffindex,
			AOconf[loop].RTSLOGarray[i].frameindex,
			AOconf[loop].RTSLOGarray[i].memsize
			);
			NBstreams++;			
		}
	}
	printf("----------------------------------------------------------\n");
	printf("%ld RTstreamLOGs active\n", NBstreams);
	
	return NBstreams;
}



int AOloopControl_RTstreamLOG_saveloop(int loop, char *dirname)
{
	long i;
	
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
	{
		if(AOconf[loop].RTSLOGarray[i].save == 1)
		{
		}
	}
	
	return 0;
}


