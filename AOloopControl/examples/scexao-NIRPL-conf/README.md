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

