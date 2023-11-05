# Overview

KalAO system

cacao-task-manager tasks for this example :

~~~
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
~~~
Subsequent tasks can perform specific parts of the AO loop.


# Running the example

:warning: Check the [instructions](https://github.com/cacao-org/cacao/tree/dev/AOloopControl/examples) before running these steps



## Setting up processes


```bash
# Deploy configuration :
# download from source to current directory
cacao-loop-deploy -c KalAO-ttmloop

# OPTIONAL: Change loop number, name, DM index, simulation DM index:
# CACAO_LOOPNUMBER=7 cacao-loop-deploy -c KalAO-dmloop
# CACAO_LOOPNUMBER=7 CACAO_DMINDEX="03" cacao-loop-deploy -c KalAO-dmloop

# OPTIONAL: Edit file KalAO-ttmloop-conf/cacaovars.bash as needed
# For example, change loop index, DM index, etc ...

# Run deployment (starts conf processes)
cacao-loop-deploy -r KalAO-ttmloop

# Note: the copy and run steps can be done at once with :
# cacao-loop-deploy KalAO-dmloop


# Go to rootdir, from which user controls the loop
cd kalaottmloop-rootdir

# select hardware mode
./scripts/aorun-setmode-hard
# (the alternative simulation mode is not yet implemented)
```

## Run DM and WFS simulators

```bash
# Run hardware DM
# cacao-aorun-000-dm start

# Run simulation DM (simulation mode only)
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs start
```





## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS start
```

## Measure DM to WFS latency

```bash
# Measure latency
cacao-aorun-020-mlat
```



## Acquire response matrix


### Prepare DM poke modes

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes
```
The following files are written to ./conf/DMmodes/ :
- DMmask.fits    : DM mask
- Fmodes.fits    : Fourier modes
- Zmodes.fits    : Zernike modes
- HpokeC.fits    : Hadamard modes
- Hmat.fits      : Hadamard matrix (to convert Hadamard-zonal)
- Hpixindex.fits : Hadamard pixel index
- SmodesC.fits    : 



### Run acquisition


```bash
# Acquire response matrix - Simple modes
cacao-fpsctrl setval measlinresp procinfo.loopcntMax 20
cacao-aorun-030-acqlinResp SmodesC

# NOTE: Alternate option is Hadamard modes
# Acquire response matrix - Hadamard modes
#cacao-fpsctrl setval measlinresp procinfo.loopcntMax 3
#cacao-aorun-030-acqlinResp HpokeC
cacao-aorun-031-RMHdecode
cacao-aorun-032-RMmkmask
```
:warning: DM and WFS masks will be required to compute control modes. They can be computed from a zonal RM (as shown above), or written by hand (single precision floats, 0.0 and 1.0 values).

### Take reference

```bash
# Acquire reference
cacao-aorun-026-takeref
```


## Compute control matrix (straight)

Compute control modes, in both WFS and DM spaces.

```bash
cacao-fpsctrl setval compstrCM RMmodesDM "../conf/RMmodesDM/SmodesC.fits"
cacao-fpsctrl setval compstrCM RMmodesWFS "../conf/RMmodesWFS/SmodesC.WFSresp.fits"
cacao-fpsctrl setval compstrCM svdlim 0.2
```
Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```



## Running the loop

Unselect GPU to run on CPU
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 99
cacao-fpsctrl setval mvalC2dm GPUindex 99
```


From directory kalaottmloop-rootdir, start 3 processes :

```bash
# start WFS -> mode coefficient values
cacao-aorun-050-wfs2cmval start

# start modal filtering
cacao-aorun-060-mfilt start

# start mode coeff values -> DM
cacao-aorun-070-cmval2dm start

```

Closing the loop and setting loop parameters with mfilt:

```bash
# Set loop gain
cacao-fpsctrl setval mfilt loopgain 0.04

# Set loop mult
cacao-fpsctrl setval mfilt loopmult 0.95

# close loop
cacao-fpsctrl setval mfilt loopON ON

```


THE END
