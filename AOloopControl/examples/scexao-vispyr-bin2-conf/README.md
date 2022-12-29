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

## Setup and Calibration


```bash
# Deploy configuration :
# download from source and start conf processes
cacao-loop-deploy scexao-vispyr-bin2

# Go to rootdir, from which user controls the loop
cd vispyr2-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)

# Run hardware DM (optional if running in simulation mode)
cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs start
cacao-aorun-005-takedark

# Measure latency
cacao-aorun-020-mlat -w

# Acquire WFS frames
cacao-aorun-025-acqWFS start

# Acquire zonal response matrix
cacao-aorun-030-acqzRM start

# Acquire low-order response matrix
cacao-aorun-035-acqloRM start

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

From directory vispyr-rootdir :







THE END
