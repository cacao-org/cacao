# STEP-BY-STEP EXAMPLE {#page_cacao_stepbystep_example}

## 1. Important tips before getting started

### 1.1. Inspecting and viewing streams

cacao uses shared memory streams to store real-time data and communicate through processes. At any time, you can inspect the content of a stream by launching:

	shmimmon <streamname>

You can also view the stream :

	shmimview /tmp/<streamname>.im.shm
	
As you go through the example below, you will make frequent use of these commands to check status.




### 1.2. Checking processes

Processes run in tmux sessions. To list all tmux sessions:

	tmux ls
	
To enter a tmux session

	tmux a -t <sessionname>
	
To detach from the session, type CTRL-B and then D. Do not type CTRL-C or exit, this will kill the process.



## 2. Starting the linear hardware simulator


In this example, we will set a hardware simulator of the AO system. The simulator takes DM shapes and computes corresponding WFS images, so it replaces the actual hardware. cacao will then connect to this simulator, as if it was connected to actual hardware.



**STEP 1**: Create a working directory `<workdir>`

**STEP 2**: Install cacao scripts into `<workdir>`:

	cd <srcdir>/src/AOloopControl/scripts
	./syncscripts -e <workdir>
	cd <workdir>
	./syncscripts
	
Note: on scexao computer, <srcdir> = /home/scexao/src/cacao

**STEP 3**: Download a calibration, consisting of zonal response matrix and a WFS reference. Place the file in a new directory `<workdir>/simLHS`

Calibration files : 
- WFS reference: https://drive.google.com/file/d/1LnYfc8mKYyERc9cNMCiVTvwX-3wRsdFU 
- WFS zonal response: https://drive.google.com/open?id=1rBkLllb6rR0z-D9YfysHedAWFE_Umwil




**STEP 4**: Launch aolconf, loop number 5, loop name simtest:

	./aolconf -L 5 -N simtest
	
Note that you subsequent calls to aolconf should then be without the -L and -N options.

**STEP 5**: Set DM :
- set DM index to 01
- size to match the calibration. `dmxs` and `dmys` GUI top menu.


**STEP 6**: Autoconfigure DM: `dmnolink` in GUI top menu. This will configure the DM in its simplest setting (no link to other loops).

**STEP 7**: Turn off dmvolt, as we are not connected to actual hardware: `dmvolt0` in GUI top menu.

**STEP 8**: Set DM averaging mode to 2: `dmcombam` in GUI top menu.

**STEP 9**: Start DMcomb. `initDM` in GUI top menu. This will start DM channels and the process that monitors them and adds them to a single total displacement channel.

**STEP 10**: Load all memory. `M` in GUI top menu

**STEP 11**: Link LHS files.
From main menu, go to "Test mode" submenu. Select `zrespMlinsim` and `wfsref0linsim`.

**STEP 12**: Start the LHS process.\n
Select GPU device and start LHS process.\n
To check that output WFS image is updating:

	shmimmon aol5_linsimWFS
	
To check GPU useage:

	nvidia-smi



**OPTIONAL**: Start artificial turbulence



## 3. Acquiring calibration, compute control matrix

Under configuration GUI menu (Configure/link AO loop):

- Measure hardware timing: `mlat`. You will bye asked how many frames to use for sampling. Keep the default value (100). This command will update timing parameters (measured loop frequ, hardware latency). 
- set Hadamard mode to ON: `Hon`
- set modal RM to ON: `RMMon`
- set modal RM amplitude to 0.05um: `rmMamp`
- set modal RM cycle per aperture to 3: `rmMcpa`
- set RM excluded frames to 1: `rmexfr`
- Acquire automatic calibration: `nAUTOc`

Under control matrix GUI menu :

- Select maximum control spatial frequency: `modeCPA`
- Create modes and control matrix: `mkModes0`
- update configuration: `confUp`

Load and manage configuration :

- load configuration
- save configuration


## 4. Run loop

- select GPU0 set
- Turn off GPU1 set
- GPUall OFF
- CMMODE OFF
- Toggle all processes to ON
- Turn on DM filtered write




