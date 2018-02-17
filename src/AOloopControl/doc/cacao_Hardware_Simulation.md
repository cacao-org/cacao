





# cacao Hardware Simulation {#page_cacao_Hardware_Simulation}

## Overview

There are 3 methods for users to simulate hardware

- METHOD 1: Provide an external simulation that adheres to AOloopControl input/output conventions

- METHOD 2: Use the physical hardware simulation provided by the package

- METHOD 3: Use the linear hardware simulation: this option is fastest, but only captures linear relationships between DM actuators and WFS signals



## METHOD 1: Provide an external simulation that adheres to AOloopControl input/output conventions

The user runs a loop that updates the wavefront sensor image when the DM input changes. Both the DM and WFS are represented as shared memory image streams. When a new DM shape is written, the DM stream semaphores are posted by the user, triggering the WFS image computation. When the WFS image is computed, its semaphores are posted.




## METHOD 2: Physical hardware simulation

The AOsim simulation architecture relies on individual processes that simulate subsystems. Each process is launched by a bash script. ASCII configuration files are read by each process. Data I/O can be done with low latency using shared memory and semaphores: a process operation (for example, the wavefront sensor process computing WFS signals) is typically triggered by a semaphore contained in the shared memory wavefront stream. A low-speed file system based alternative to shared memory and semaphores is also provided.

Method 2 simulates incoming atmospheric WFs, a pyramid WFS based loop feeding a DM, a coronagraphic LOWFS and coronagraphic PSFs.

### Running Method 2

Launch the simulator with the following steps:

- Create a series of atmospheric wavefronts (do this only once, this step can take several hrs):
	
		./aohardsim/aosimmkwfser

	Stop the process when a few wavefront files have been created (approximately 10 minimum). The AO code will loop through the list of files created, so a long list is preferable to reduce the frequency at which the end-of-sequence discontinuity occurs. The current wavefront file index is displayed as the process runs; in this example, the process is working on file #2:
	
		Layer  0/ 7, Frame   99/ 100, File      0/100000000  [TIME =     0.0990 s]  WRITING SCIENCE WAVEFRONT ... - 
		Layer  0/ 7, Frame   99/ 100, File      1/100000000  [TIME =     0.1990 s]  WRITING SCIENCE WAVEFRONT ... - 
		Layer  1/ 7, Frame   42/ 100, File      2/100000000  [TIME =     0.2420 s]  

	Type `CTRL-C` to stop the process. Note that you can relaunch the script later to build additional wavefront files.
	
	By default, the wavefront files are stored in the work directory. You may choose to move them to another location (useful if you have multiple work directories sharing the same wavefront files). You can then create a symbolic link `atmwf` to an existing atmospheric wavefront simulation directory. For example:

		ln -s /data/AtmWF/wdir00/ atmwf

- Execute master script `./aohardsim/runAOhsim`

- To stop the physical simulator: `./aohardsim/runAOhsim -k`


Important notes:

- Parameters for the simulation can be changed by editing the `.conf` files in the `aohardsim` directory

- You may need to kill and relaunch the main script twice after changing parameters


### Method 2 output streams


------------------------ -----------------------------------------------------------------------
Stream                   Description
------------------------ -----------------------------------------------------------------------
**wf0opd**               Atmospheric WF OPD

**wf0amp**               Atmospheric WF amplitude

**wf1opd**               Wavefront OPD after correction [um] ( = wf0opd - 2 x dm05dispmap )

**dm05disp**             DM actuators positions

**dm05dispmap**          DM OPD map

**WFSinst**              Instantaneous WFS intensity

**pWFSint**              WFS intensity frame, time averaged to WFS frame rate and sampled to WFS camera pixels

**aosim_foc0_amp**       First focal plane (before coronagraph), amplitude

**aosim_foc0_pha**       First focal plane (before coronagraph), phase

**aosim_foc1_amp**       First focal plane (after coronagraph), amplitude

**aosim_foc1_pha**       First focal plane (after coronagraph), phase

**aosim_foc2_amp**       Post-coronagraphic focal plane, amplitude

**aosim_foc2_pha**       Post-coronagraphic focal plane, phase
------------------ -----------------------------------------------------------------------






### Processes and scripts details

#### Process `aosimmkWF`


`aosimmkWF` reads precomputed wavefronts and formats them for the simulation parameters (pixel scale, temporal sampling).

Parameters for `aosimmkWF` are stored in configuration file:

File `aosimmkWF.conf.default` :

~~~~ {.numberLines}
!INCLUDE "../scripts/aohardsim/aosimmkWF.conf.default"
~~~~


#### Process `aosimDMrun`


File `aosimDMrun.conf.default` :

~~~~ {.numberLines}
!INCLUDE "../scripts/aohardsim/aosimDMrun.conf.default"
~~~~




#### Process `aosimPyrWFS`

File `aosimPyrWFS.conf.default` :

~~~~ {.numberLines}
!INCLUDE "../scripts/aohardsim/aosimPyrWFS.conf.default"
~~~~




### AO loop control

The ``aolconf`` script is used to configure and launch the AO control loop. It can be configured with input/output from real hardware or a simulation of real hardware.



#### Shared memory streams

------------------------------ -----------------------------------------------------------
Script                         Description
------------------------------ -----------------------------------------------------------
**wf0opd**                     Wavefront OPD prior to wavefront correction [um]

**wf1opd**                     Wavefront OPD after correction [um] ( = wf0opd - 2 x dm05dispmap )

**dm05disp**                   DM actuators positions

**dm05dispmap**                DM OPD map

**WFSinst**                    Instantaneous WFS intensity

**pWFSint**                    WFS intensity frame, time averaged to WFS frame rate and sampled to WFS camera pixels
------------------------------ -----------------------------------------------------------





#### Hardware simulation architecture

![data flow](./figures/aosimlink.jpg "aosim data flow")


Close-loop simulation requires the following scripts to be launched to simulate the hardware, in the following order :

* ``aosimDMstart``: This script creates DM channels (uses dm index 5 for simulation). Shared memory arrays ``dm05disp00`` to ``dm05disp11`` are created, along with the total displacement ``dm05disp``. Also creates the ``wf1opd`` shared memory stream which is needed by `aosimDMrun` and will be updated by runWF. ``wf1opd`` is the master clock for the whole simulation, as it triggers DM shape computation and WFS image computation.
* ``aosimDMrun``: Simulates physical deformable mirror (DM)
* ``aosimmkWF``: Creates atmospheric wavefronts
* ``aosimWFS``: Simulates WFS

Some key script variables need to coordinated between scripts. The following WF array size should match :

* ``WFsize`` in script ``aosimDMstart``
* ``ARRAYSIZE`` in ``aosimmkWF.conf``
* ``ARRAYSIZE`` in ``aosimDMrun.conf``


The main hardware loop is between ``aosimmkWF`` and ``aosimWFS``: computation of a wavefront by ``aosimmkWF`` is *triggered* by completion of a WFS instantaneous image computation by ``aosimWFS``. The configuration files are configured for this link.



#### DM temporal response

The DM temporal response is assumed to be such that the distance between the current position $p$ and desired displacement $c$ values is multiplided by coefficient $a<1$ at each time step $dt$. The corresponding step response is :

$c - p((k+1) dt) = (c - p(k dt)) a$

$c - p(k dt) = (c-p0) a^k$

$p(k dt) = 1-a^k$

The corresponding time constant is

$a^{\frac{t0}{dt}} = 0.5$

$\frac{t0}{dt} ln(a) = ln(0.5)$

$ln(a) = ln(0.5) dt/t0$

$a = 0.5^{\frac{dt}{t0}}$


### Processes and scripts: system ouput


The output (corrected) wavefront is processed to compute ouput focal plane images, and optionally LOWFS image.

#### Process `aosimcoroLOWFS`

Computes coronagraphic image output and LOWFS image

File `aosimcoroLOWFS.conf.default`:

~~~~ {.numberLines}
!INCLUDE "../scripts/aohardsim/aosimcoroLOWFS.conf.default"
~~~~

#### Ouput simulation architecture

![coroLOWFS data flow](./figures/aosimlink_coroLOWFS.jpg "coroLOWFS data flow")
















## METHOD 3: Linear Hardware Simulation

### Overview

The Linear Hardware Simulation (LHS) uses a linear response matrix to compute the WFS image from the DM state. It is significantly faster than the Physical Hardware Simulation (PHS) but does not capture non-linear effects.

### Setup


- Create directory LHScalib:

~~~~
mkdir LHScalib
~~~~

- Download response matrix and reference, place them in directory `LHScalib`.

- Start GUI, loop 5, name simLHS

~~~~
./aolconf -L 5 -N simLHS
~~~~

- Start DM: index 04, 50 x 50; Auto-configure: main DM (no link); STOP -> (re-)START DM comb process

- Go to `TEST MODE` GUI

- Enter linear simulation zonal response matrix and linear simulation WFS reference (`zrespMlinsim` and `wfsref0linsim` selections at top of screen). 

- **Start linear simulator** (`lsimon` selection). The simulator reacts to changes in aol5_dmdisp (= dm04disp)





