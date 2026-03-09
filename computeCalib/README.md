# Module: computeCalib

Module responsible for `computeCalib` functionality.

## Source Files

| File | Description |
|------|-------------|
| `AOloopControl_computeCalib_dm.c` | Adaptive Optics Control loop engine compute calibration |
| `AOloopControl_computeCalib_loDMmodes.c` | Adaptive Optics Control loop engine compute calibration |
| `AOloopControl_computeCalib_processRM.c` | Adaptive Optics Control loop engine compute calibration |
| `RM2zonal.c` | No description available. |
| `actmap_sample2D.c` | No description available. |
| `computeHadamard.c` | Compute Hadamard modes |
| `compute_control_modes.c` | Compute AO control modes in both input (WFS) and output (DM) space |
| `compute_masksWFSDM.c` | No description available. |
| `compute_straight_CM.c` | No description available. |
| `generateRMWFS.c` | No description available. |
| `maskextrapolate.c` | No description available. |
| `modes_spatial_extrapolate.c` | No description available. |

## Standalone Executables

| Executable | Source File | Description |
|------------|-------------|-------------|
| `cacao-fpsexec-cacaocc-maskextrapolate` | `maskextrapolate.c` | No description available. |
| `cacao-fpsexec-cacaocc-actsamp2d` | `actmap_sample2D.c` | No description available. |
| `cacao-fpsexec-cacaocc-rm2zonal` | `RM2zonal.c` | No description available. |
| `cacao-fpsexec-cacaocc-comphadamard` | `computeHadamard.c` | Compute Hadamard modes |
| `cacao-fpsexec-cacaocc-genrmwfs` | `generateRMWFS.c` | No description available. |
| `cacao-fpsexec-cacaocc-compmaskwfsdm` | `compute_masksWFSDM.c` | No description available. |
| `cacao-fpsexec-cacaocc-compstrcm` | `compute_straight_CM.c` | No description available. |
| `cacao-fpsexec-cacaocc-compctrlmodes` | `compute_control_modes.c` | Compute AO control modes in both input (WFS) and output (DM) space |

## Dependencies
- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
