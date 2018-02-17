# CONF DIRECTORY CONTENT {#page_cacao_conf_directory}

## Overview

The conf directory contains the following file types:

File Type                            | Description
-------------------------------------|----------------------------------------
conf_XXXX.txt                        | Information about the configuration (name etc...)
param_XXXXX.txt                      | cacao parameter, usually written by GUI            
streamlink_XXXX.txt                  | shared memory link name for AOCCE stream XXXX     
shmim_XXXX.name.txt                  | Name of FITS file to be loaded to shared memory
shmim_XXXX.imsize.txt                | Image size of FITS file loaded to shared memory (written by Fits2shm)
shmim_XXXX.fits                      | Sym link to FITS file loaded to shared memory (created by Fits2shm)
instconf_XXXXX.txt                   | Instrument-specific configuration (filter wheels, stages.. )

This page does not list instconf type files, as they are instrument-specific.


## Parameters

### Overall and misc

	param_CPUset_mode.txt                 1 if using CPUset
	param_wfslambdanm.txt                 WFS wavelength \[nm\]



### Linear Hardware Simulator

	param_linsimDelay.txt                 Linear Hardware Simulator (LHS) hardware latency \[us\]
	param_linsimdt.txt                    Linear Hardware Simulator (LHS) loop interval \[us\]
	param_GPUlinsim.txt                   Linear Hardware Simulator (LHS) GPU index



### DM 

	param_DMindex.txt                     DM index (00, 01, 02 ... )
	param_DMMODE.txt                      ZONAL (default) or MODAL
	param_DMxsize.txt                     DM x size
	param_DMysize.txt                     DM y size
	param_DM2DM_mode.txt                  DM-to-DM more. 1 if this DM drives another DM, 0 otherwise
	param_DMwfsref_mode.txt               1 for CPU-based dmcomb DM ouput applied as WFS offset
	param_DMvolt_mode.txt                 1 if DM voltages applied (requires stream "dmvolt")
	param_DMcombave_mode.txt              DM averaging mode (default = 0)
	param_DMdelayON.txt                   1 if DM delay introduced
	param_DMdelayus.txt                   DM delay value [us]


### Timing

	param_loopfrequ.txt                   WFS camera frequency [Hz] (value set by WFS)
	param_mloopfrequ.txt                  WFS camera frequency [Hz] (measured value)
	param_hardwlatency.txt                Hardware latency [s]
	param_hardwlatency_frame.txt          Hardware latency [frame]
	param_complatency.txt                 Computation latency [s]
	param_complatency_frame.txt           Computation latency [frame]
	param_wfsmextrlatency.txt             Modal reconstruction latency [s]
	param_wfsmextrlatency_frame.txt       Modal reconstruction latency [frame]
	param_nblatm.txt                      Number of cycles in latency measurement


### Response Matrix acquisition

	param_ACzrmtime.txt                   Max running time for zonal RM acquisition [s]
	param_ACzrmNBcycle.txt                Max number of cycles for zonal RM acquition
	param_ACmrmtime.txt                   Max running time for monal RM acquisition [s]
	param_ACmrmNBcycle.txt                Max number of cycles for modal RM acquition
	param_RMdelayfr.txt                   RM acquisition delay [frame]
	param_RMfrave.txt                     RM acquisition number of frames averaged per poke
	param_RMexfr.txt                      RM acquisition number of frames excluded during poke transition
	param_RMamplum.txt                    RM acquisition amplitude [um]
	param_delayRM1us.txt                  RM acquisition delay 1 [s]
	param_RMpokeMode.txt                  RM poking mode. 0=zonal, 1=Hadamard.
	param_RMMamplum.txt                   RM Low Order Modal acquisition amplitude [um]
	param_RMMcpa.txt                      RM Low Order Modal max CPA
	param_WFSnorm.txt                     Normalize WFS frames
	param_DMmaskRMp0.txt                     
	param_DMmaskRMc0.txt                     
	param_DMmaskRMp1.txt                     
	param_DMmaskRMc1.txt                     
	param_WFSmaskRMp0.txt                     
	param_WFSmaskRMc0.txt                     
	param_WFSmaskRMp1.txt                     
	param_WFSmaskRMc1.txt                     
	param_WFSmaskSNRr.txt                     
	param_RMCalibReuseMasks.txt            1 if calibration masks reused (0 if re-computed)                     


### Modess and Control Matrix computation

	param_MASKS_LOCK.txt                  1 if WFS and DM masks are locked
	param_NBmodeblocks.txt                Number of mode blocks (default=1)


### Loop Control

	param_DMprimWriteON.txt               Primary Write ON/OFF (0 or 1)
	param_CMMODE.txt                      Combined Matrix (0 or 1)
	param_GPU0.txt                        Number of GPUs in set 0
	param_GPUset0devXX.txt                GPU set 0 device XX
	param_GPU1.txt                        Number of GPUs in set 1
	param_GPUset1devXX.txt                GPU set 1 device XX
	param_GPUmodesextrwfs.txt             WFS mode coefficients extraction: GPU device
	param_GPUdmfwb.txt                    DM modal write (post-filtering): GPU device
	param_GPUzpoffsetZ.txt                Zonal WFS zero point offset loop: GPU device
	param_GPUzpoffsetM.txt                Modal WFS zero point offset loop: GPU device
	param_LOOPPROCESS_EXTRWFSMODES.txt 
	param_LOOPPROCESS_EXTROLMODES.txt 
	param_LOOPPROCESS_DMFILTWB.txt 
	param_LOOPPROCESS_ZPO.txt 
	param_LOOPPROCESS_DMCAVE.txt 
	param_LOOPPROCESS_WFSRESAVE.txt 
	param_AUTOTUNELIMITS_ON.txt           Autotuning ON/OFF (ON or OFF)
	param_AUTOTUNELIMITmcoeff.txt         Autotuning limits
	param_AUTOTUNELIMITdelta.txt          Autotuning limits
	param_AUTOTUNEGAINS_ON.txt            Autotuning gain (ON or OFF)
	param_ARPFon.txt                      Auto-regressive predictive filter (ON or OFF)
	param_ARPFg.txt                       Auto-regressive predictive filter gain
	param_loopgain.txt
	param_loopmmultcoeff.txt
	param_loopmaxlim.txt                  Limit of DM actuators (direct write only)


## Stream Links

	streamlink_dmC.name.txt               DM correction stream
	streamlink_dmO.name.txt               DM offset stream (DM flat channel)
	streamlink_dmZP0.name.txt             DM zero point offset #0 stream
	streamlink_dmZP1.name.txt             DM zero point offset #1 stream
	streamlink_dmZP2.name.txt             DM zero point offset #2 stream
	streamlink_dmZP3.name.txt             DM zero point offset #3 stream
	streamlink_dmZP4.name.txt             DM zero point offset #4 stream
	streamlink_dmZP5.name.txt             DM zero point offset #5 stream
	streamlink_dmZP6.name.txt             DM zero point offset #6 stream
	streamlink_dmZP7.name.txt             DM zero point offset #7 stream
	streamlink_dmdisp.name.txt            DM total displacement stream
	streamlink_dmRM.name.txt              DM response matrix stream
	streamlink_dm2dmM.name.txt            DM modes controlled in DM-to-DM mode (optional)
	streamlink_dm2dmO.name.txt            DM modes output in DM-to-DM mode (optional)
	streamlink_dmwrefRM.name.txt          WFS ref RM (optional)
	streamlink_dmwrefO.name.txt           WFS output (optional)
	streamlink_wfsim.name.txt             WFS image


## Shared memory FITS file initializations to streams

File shmim_\<stream\>.name.txt contain the FITS file names that are loaded into shared memory streams aol\<loop\>_\<stream\>.

	shmim_wfsdark.name.txt                WFS dark
	shmim_wfsmap.name.txt                 WFS response map
	shmim_wfsmask.name.txt                WFS mask
	shmim_dmmap.name.txt                
	shmim_dmmaskRM.name.txt                
	shmim_dmslaved.name.txt                
	shmim_dmmask.name.txt                
	shmim_wfsref0.name.txt                
	shmim_zrespM.name.txt                
	shmim_LODMmodes.name.txt                
	shmim_LOrespM.name.txt                
	shmim_DModes.name.txt                
	shmim_respM.name.txt                
	shmim_contrM.name.txt                
	shmim_DMmodesXX.name.txt                
	shmim_respMXX.name.txt                
	shmim_contrMXX.name.txt                
	shmim_contrMcXX.name.txt                
	shmim_contrMcactXX_00.name.txt                



## FITS files

	RMpokeCubeZ.fits.gz                   Simple Zonal poke mode
	RM_DMmask.fits.gz                     mask for creating RM Hadamard pokes
	Hpoke.fits.gz                         Hadamard poke modes
	Hmat.fits.gz                          Hadamard-zonal transformation matrix
	Hpixindex.fits.gz                     Hadamard DM pixel index 
	RMmat.fits.gz                         Current poke transformation matrix (optional)
	RMpixindex.fits.gz                    Current DM pixel index
	RMpokeCube.fits.gz                    Current poke modes




# REFERENCE: Content of ./status directory

The status directories contrains the current state of the cacao processes.

	stat_DMcombON.txt                     " ON" if DMcomb is running, "OFF" otherwise
	stat_lsimON.txt                       " ON" if linear hardware simulator is running, "OFF" otherwise

