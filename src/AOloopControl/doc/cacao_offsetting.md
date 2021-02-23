

# Offsetting {#page_cacao_offsetting}



Input channels are provided to offset the AO loop convergence point. By default, **DM channels 04-11 can be dedicated to zero-point offsetting**. The DM channels are sym-linked to `aolN_dmZP0` - `aolN_dmZP7`.




Zero-point offsetting relies on two separate processes :

- Converting DM offsets to WFS offsets (can be done by CPU or GPU): aolN_dmZP -> aolN_wfszpo
- Summing and applying WFS offsets aolN_wfszpo to aolN_wfsref

Zonal offsetting takes a DM map, multiplies it by the response matrix (in CPU or GPU). 
With Modal offsetting, this multiplication is pre-computed.



---
---


## 1. Converting DM offsets to WFS offsets (zonal, CPU mode)

CPU-based zero point offsets will compute WFS offsets from the zero point offset DM channels (04-11) and apply them to the `aolN_wfszpo#` stream. 

- **Activate CPU individual zero point offset channels** (`zplon0` to `zplon7`) to convert dm zero point displacements to wfs offsets.

Every time one of the activated DM channel changes, the corresponding wfs `aolN_wfszpo#` zero point offset is CPU-computed.

The process runs inside tmux session `aolNzploop#`


---
---


## 2. Converting DM offsets to WFS offsets (zonal, GPU mode)

A faster GPU-based zero point offset from DM to WFS is provided for each of the 8 offset channels. GPU-based and CPU-based offsetting for a single channel are mutually exclusive.

- **Activate GPU individual zero point offset channels** (`GPUzplon0` to `GPUzplon7`) to convert dm zero point displacements to wfs offsets.

Every time one of the activated DM channel changes, the corresponding wfs `aolN_wfszpo#` zero point offset is GPU-computed.

The process runs inside tmux session `aolNGPUzploop#`


---
---

## 3. Modal offsetting from another loop

The two methods above are zonal offsetting: the DM map is multiplied by the zonal WFS response to compute WFS offset. 

Modal offsetting, instead, relies on a pre-computed set of WFS modal offets. This is most useful when a separate control loop is driving modal offsets to the current loop.

To implement modal offsetting from a separate loop (refered to as the offsetting loop) : 

- Configure the DM of the offsetting loop to write WFS zero point offsets to the current loop (see setting up DM section)

- Do not activate either of the previous two zonal offsetting schemes

- Activate the WFS offsets process (see next subsection)



---
---



## 4. Summing and applying WFS offsets to aolN_wfsref

To activate WFS offsets to aolN_wfsref, the user needs to :

- **Toggle the zero point offset loop process ON** (`LPzpo`) prior to starting the loop. 

Command `aolzpwfscloop` (function AOloopControl_WFSzeropoint_sum_update_loop() ) launches a loop that monitors shared memory streams `aolN_wfszpo0` to `aolN_wfszpo7`, and updates the WFS reference when one of these has changed.

The loop is running inside tmux session `aolNwfszpo`, and is launched when the loop is closed (`Floopon`) if the loop zero point offset flag is toggled on (`LPzpo`)




---
---




## WFS average offset

Measures average WFS residual with script :

	./auxscripts/aolmkWFSres 0.0005
	
Running average is in stresm aol_wfsres_ave







---
---





## Controlling offsets from another loop
 

## Running the loop

The next steps are similar to the ones previously described, with the following important differences:

- The control matrix should be computed in zonal mode (no modal CPA block decomposition)

