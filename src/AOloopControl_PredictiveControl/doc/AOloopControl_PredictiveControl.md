# AOloopControl PredictiveControl {#page_module_AOloopControl_PredictiveControl}

---

# Overview

Predictive control is implemented in two processes:

- The optimal auto-regressive (AR) filter predicting the current state from previous states is computed. The AR filter is computed from open-loop estimates, so the processes computing open-loop telemetry need to be running.
- The AR filter is applied to write a prediction buffer, which can be written asynchronously from the main loop steps.

The predictive filter is modal, and adopts the same modes as the main control loop.



## Scripts


File          | Description
--------------|------------------------------------------------------
aolARPF 	  | Find auto-regressive predictive filter
aolARPFblock  | AO find optimal AR linear predictive filter


---
---


# Data flow for real-time operation

Predictive control is set up by blocks of modes. A block is configured through the aolconf predictive control sub-panel, which writes to configuration files conf/conf_PFblock_XXX.txt, where XXX is the block number (000, 001, 002 etc...). Configuration files specify the modes within each block (index min to index max), the predictive filter order, time lag and and averaging gain.

For each block, there are 3 main processes involved in running the predictive control:

- Collect data from input telemetry
- Compute prediction filter
- Apply prediction


All 3 processes work in a chain, and can be turned on/off from the GUI.


---


## Collect data from input telemetry

Watching input telemetry this process listens to the input telemetry stream and periodically writes data to be used to compute a filter. This runs function AOloopControl_PredictiveControl_builPFloop_WatchInput() called from AOloopControl_PredictiveControl.c.

Output is a 3D image, of size: NBmodes x 1 x NBsteps.

Output is shared memory image stream, named:

	aol<loop>_modevalol_PFb<blocknumber>


---


## Compute filter

Computing filter. Runs CLI command mkARpfilt, which runs function LINARFILTERPRED_Build_LinPredictor() in linARfilterPred.c.

Input to function: aol<loop>_modevalol_PFb<blocknumber>

Output to function: aol<loop>_modevalol_outPFb<blocknumber>


### Packaging input data matrix 

The routine packages a data matrix PFmatD with dimension (size[0] = number of time samples) (size[1] = dimension of each sample). Usually, size[0] > size[1]. 

In the column-major matrix representation, PFmatD data array is the transpose of the data matrix. Predictive control requires the pseudoinverse of the transpose of the data matrix to be computed, so the pseudoinverse of PFmatD is computed by calling function CUDACOMP_magma_compute_SVDpseudoInverse().

### Computing Pseudoinverse

Performed by calling function CUDACOMP_magma_compute_SVDpseudoInverse().


### Assembling Predictive Filter




---

## Apply prediction

Prediction engine (= apply filter). Runs script ./auxscripts/predFiltApplyRT.

---

