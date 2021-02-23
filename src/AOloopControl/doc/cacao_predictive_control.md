# Predictive control {#page_cacao_predictive_control}

## Overview

Predictive control is implemented in two processes:

- The optimal auto-regressive (AR) filter predicting the current state from previous states is computed. The AR filter is computed from open-loop estimates, so the processes computing open-loop telemetry need to be running.

- the AR filter is applied to write a prediction buffer, which can be written asynchronously from the main loop steps.

The predictive filter is modal, and adopts the same modes as the main control loop.


## Scripts

File                       | Description
---------------------------|-----------------------------------------------------------
aolARPF                    | find auto-regressive predictive filter
aolARPFblock               | AO find optimal AR linear predictive filter 


---
---



## Data flow

Predictive control is set up by blocks of modes. A block is configured through the aolconf predictive control sub-panel, which writes to configuration files `conf/conf_PFblock_XXX.txt`, where XXX is the block number (000, 001, 002 etc...). Configuration files specify the modes within each block (index min to index max), the predictive filter order, time lag and and averaging gain.

For each block, there are 3 main processes involved in running the predictive control:

### Collecting input

**Watching input telemetry** this process listens to the input telemetry stream and periodically writes data to be used to compute a filter. This runs function AOloopControl_PredictiveControl_builPFloop_WatchInput() in AOloopControl_PredictiveControl.c.

Runs in tmux sessions: aol0PFb0watchin, aol0PFb1watchin ...

### Computing Filter

**Computing filter**. Runs CLI command `mkARpfilt`, which runs function LINARFILTERPRED_Build_LinPredictor() in linARfilterPred.c.

Runs in tmux sessions: aol0PFb0comp, aol1PFb0comp ...


## Running real-time prediction

**Prediction engine** (= apply filter). Runs script `./auxscripts/predFiltApplyRT`.

All 3 processes work in a chain, and can be turned on/off from the GUI.


