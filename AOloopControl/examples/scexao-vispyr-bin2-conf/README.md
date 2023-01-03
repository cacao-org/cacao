# Overview

SCExAO system pyramid WFS.
Low-resolution WFS mode (120x120)

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
cacao-loop-deploy scexao-vispyr-bin2

# Go to rootdir, from which user controls the loop
cd vispyr2-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)
```

## Run DM and WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs start
```



## Measure WFS dark and DM to WFS latency


```bash
cacao-aorun-005-takedark

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
The following files are written to ./conf/DMmodes/ :
- DMmask.fits    : DM mask
- Fmodes.fits    : Fourier modes
- HpokeC.fits    : Hadamard modes
- Hmat.fits      : Hadamard matrix (to convert Hadamard-zonal)
- Hpixindex.fits : Hadamard pixel index



### Run acquisition


```bash
# Acquire response matrix - Hadamard modes
cacao-aorun-030-acqlinResp HpokeC

# Acquire response matrix - Fourier modes
cacao-aorun-030-acqlinResp Fmodes
```

## Compute control matrix

Compute control modes, in both WFS and DM spaces.

```bash
# Compute control matrix using Fourier modes
cacao-aorun-040-compfCM

# Make directory for storing calibrations
mkdir -p ../AOcalibs
cd ..; ln -s $(pwd)/AOcalibs $(pwd)/vispyr2-rootdir/AOcalibs; cd -

# Save current calibration
cacao-calib-archive cal000


# Apply calibration
cacao-calib-apply cal000
```


## Running the loop

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


cacao-fpsctrl setval compsCM fname_respM "../../AOcalibs/cal000_2022-12-29T11:26:54/aol0_zrespM.fits"


THE END
