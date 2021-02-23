

# APPENDIX A: SHARED MEMORY STREAMS  {#page_cacao_sharedmem_streams}



## Linking to existing stream 

Creates a sym link from an existing stream to a new stream. This is frequently used to connect to hardware (for example pointing aol8_wfsim to the WFS camera stream).

Within AOCCE, setting (new) stream aol#_\<deststream\> to point to \<srcstream\> involves:

- writing "\<srcstream\>" to file conf/streamlink_\<deststream\>.name.txt
- removing any existing aol#_\<deststream\> stream 
- establishing sym link from \<srcstream\> to aol#_\<deststream\>

The corresponding commands, to point aol8_wfsim to imcam, would be:

	echo "imcam" > conf/streamlink_wfsim.name.txt
	rm /tmp/aol8_wfsim.im.shm
	ln -s /tmp/imcam.im.shm /tmp/aol8_wfsim.im.shm

This is implemented in script :

	./aolfuncs/aolfunc_StreamLink




## Loading images from FITS to shared memory stream

### Overview

Loading FITS file into a new or existing stream.

The recommended command to load a FITS image to shared memory in AOCCE is for example:

	echo "./wfsref0/wfsref0_today.fits" > ./conf/shmim_wfsref0.name.txt
	Fits2shm -c -p aol${LOOPNUMBER}_ wfsref0

This command will skip loading if the stream does not need to be updated from the last load command. Add the -f option to force loading.

Options used:

     -c           read FITS file name from ./conf/shmim_<streamname>.name.txt directory
     -p <pref>    stream prefix




### Detailed description

File name can be :

- option A: read from ./conf/shmim_\<stream\>.name.txt
- option B: specfied and written to ./conf/shmim_\<stream\>.name.txt

Command for option A:

	Fits2shm <FITS file> <stream>

Command for option B:

	Fits2shm -c <stream>

A prefix is usually added to the shared memory file with the -p option:

	Fits2shm -c -p aol8_ <stream>

The -r removes/clears any previous instance of the stream and associated files: it is a stronger option than -f.

Fits2shm is located in the src/scripts/ directory of the AOCCE source code.


For both options, FITS files and shared memory files are inspected to assess need to load the image. If repetitive load requests are issued on the same file and shared memory, the script may decide that no action is required as the shared memory is already up-to-date.

Local directory ./loadedSM/ is used to keep track of streams loaded from FITS files. 


Important files:

------------------------------------- ----------------------------------------
File                                  Description
------------------------------------- ----------------------------------------
./loadedSM/\<stream\>.FITSinfo          FITS file name, size, time of last modification

./loadedSM/\<stream\>.FITSinfo.old      previous version of above file - to be used for comparison

./loadedSM/\<stream\>.FITSsame          File exists if the two FITS files are identical

./loadedSM/\<stream\>.FITSchanged       File exists if the two FITS files are different

./loadedSM/\<stream\>.SMinfo            stream file name, size, time of last modfification

./loadedSM/\<stream\>.SMinfo.old        previous version of above file - to be used for comparison

./loadedSM/\<stream\>.SMsame            File exists if the two SM files are identical

./loadedSM/\<stream\>.SMchanged         File exists if the two SM files are different

./loadedSM/\<stream\>.imsize            Image size - updated upon SM load

./loadedSM/\<stream\>.new               File exists if last load request updated or created stream

./loadedSM/\<stream\>.kept              File exists if last load request kept stream unchanged

./loadedSM/\<stream\>.missing           File exists if last load request created new stream (stream was missing)

./conf/shmim_\<stream\>.name.txt         FITS file name for stream

./conf/shmim_\<stream\>.imsize.txt       Stream size

./conf/shmim_\<stream\>.fits            (symbolic link to) FITS file

./tmp/\<prefix\>\<stream\>.im.shm       File-mapped shared memory
------------------------------------- -----------------------------------------------------------





The logic behind Fits2shm is as follows:

- Assume LOAD=0 (do not load FITS file in memory)
- Check if FITS file has changed. If yes, set LOAD=1
- Check if SM file has changed. If yes, set LOAD=1
- Check if SM file exists, if not:
	- set LOAD=1
	- create \<stream\>.missing file (otherwise, rm \<stream\>.missing file)
- Check if FORCE option, if yes, set LOAD=1
- If LOAD=1:
	- load file to shared memory
	
	
When loading :	

- OPTION A only: update ./conf/shmim_\<stream\>_name.txt to the FITS file name 	
- update ./loadedSM/\<stream\>.FITSinfo file
- update ./loadedSM/\<stream\>.SMinfo file
- create ./loadedSM/\<stream\>.kept OR ./loadedSM/\<stream\>.new file, remove the other one
- create ./loadedSM/\<stream\>.missing file if the SM file did not exist, otherwise remove ./loadedSM/\<stream\>.missing
- copy ./loadedSM/\<stream\>.imsize file to ./conf/shmim_\<stream\>.imsize.txt
- load FITS file to ./tmp/\<prefix\>\<stream\>.im.shm
- create sym link ./conf/shmim_\<stream\>.fits pointing to FITS file
	


