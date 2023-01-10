# Overview

MMT MAPS project.
Visible light Pyramid WFS. Slopes input.

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
# download from source and start conf processes
cacao-loop-deploy MAPS-vispyr

# Go to rootdir, from which user controls the loop
cd maps-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)
```

## Run DM, DM+WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs start
```



## Measure DM to WFS latency


```bash
# Measure latency
cacao-aorun-020-mlat -w
```

## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS start
```

## Acquire response matrix


### Prepare DM poke modes

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes
```
The following files are written to ./conf/RMmodesDM/ :
- DMmask.fits    : DM mask
- Fmodes.fits    : Fourier modes
- Smodes.fits    : Simple zonal modes
- HpokeC.fits    : Hadamard modes
- Hmat.fits      : Hadamard matrix (to convert Hadamard-zonal)
- Hpixindex.fits : Hadamard pixel index


### Run acquisition


```bash
# Acquire response matrix - Hadamard modes
cacao-fpsctrl setval measlinresp procinfo.loopcntMax 3
cacao-aorun-030-acqlinResp HpokeC
```


## Compute control matrix (straight)


Compute control modes, in both WFS and DM spaces.

```bash
mkdir conf/CMmodesDM
mkdir conf/CMmodesWFS
cacao-fpsctrl setval compstrCM RMmodesDM "../conf/RMmodesDM/HpokeC.fits"
cacao-fpsctrl setval compstrCM RMmodesWFS "../conf/RMmodesWFS/HpokeC.WFSresp.fits"
cacao-fpsctrl setval compstrCM CMmodesDM "../conf/CMmodesDM/CMmodesDM.fits"
cacao-fpsctrl setval compstrCM CMmodesWFS "../conf/CMmodesWFS/CMmodesWFS.fits"
cacao-fpsctrl setval compstrCM svdlim 0.2
```
Then run the compstrCM process :
```bash
cacao-aorun-039-compstrCM
```


## Running the loop

Load the CM
```bash
milk-FITS2shm "conf/CMmodesWFS/CMmodesWFS.fits" aol2_modesWFS
milk-FITS2shm "conf/CMmodesDM/CMmodesDM.fits" aol2_DMmodes
```

Configuring to CPU mode
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 99
cacao-fpsctrl setval mvalC2dm GPUindex 99
```


From directory vispyr-rootdir, start 3 processes :

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
cacao-fpsctrl setval mfilt loopgain 0.1

# Set loop mult
cacao-fpsctrl setval mfilt loopmult 0.98

# close loop
cacao-fpsctrl setval mfilt loopON ON

```


THE END
