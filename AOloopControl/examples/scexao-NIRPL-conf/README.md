# Overview

Near-IR Photonic Lantern

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
cacao-loop-deploy scexao-NIRPL

# Go to rootdir, from which user controls the loop
cd NIRPL-rootdir

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


## Take full frame dark -> will store it in aolX_wfsdark

Turn off light source, and run:
```bash
cacao-aorun-005-takedark
```
Then turn light source back on.

## Find spots

```bash
./scripts/scexao-NIRPL-findspots 1000
```

## Start remapping

```bash
cacao-aorun-003-wfsmapping
```

## Remap dark frame

```bash
./scripts/scexao-NIRPL-mapdark
```


## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS start
```

## Measure DM to WFS latency


```bash
# Measure latency
cacao-aorun-020-mlat -w
```


## Acquire response matrix


### Prepare DM poke modes


We will use Fourier modes, with maximum spatial frequency of 3.0 cycles per aperture (CPA).

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes -c 3.0
```

The following files are written to ./conf/DMmodes/ :
- DMmask.fits    : DM mask
- Fmodes.fits    : Fourier modes
- Smodes.fits    : Simple zonal modes
- HpokeC.fits    : Hadamard modes
- Hmat.fits      : Hadamard matrix (to convert Hadamard-zonal)
- Hpixindex.fits : Hadamard pixel index


### Run acquisition


```bash
# Acquire response matrix - Fourier modes
cacao-fpsctrl setval measlinresp procinfo.loopcntMax 4
cacao-aorun-030-acqlinResp Fmodes
```


### Take reference

```bash
# Acquire reference
cacao-aorun-026-takeref
```


## Compute control matrix (straight)


Compute control modes, in both WFS and DM spaces.

```bash
cacao-fpsctrl setval compstrCM RMmodesDM "../conf/RMmodesDM/Fmodes.fits"
cacao-fpsctrl setval compstrCM RMmodesWFS "../conf/RMmodesWFS/Fmodes.WFSresp.fits"
cacao-fpsctrl setval compstrCM svdlim 0.2
```
Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```


## Running the loop

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


