# Module: computeCalib

Module responsible for `computeCalib` functionality.

## Source Files

| File | Description |
|------|-------------|
| `AOloopControl_computeCalib_dm.c` | Adaptive Optics Control loop engine compute calibration |
| `AOloopControl_computeCalib_loDMmodes.c` | Adaptive Optics Control loop engine compute calibration |
| `AOloopControl_computeCalib_processRM.c` | Adaptive Optics Control loop engine compute calibration |
| `RM2zonal.c` | Rm2zonal module |
| `actmap_sample2D.c` | Actmap sample2d module |
| `computeHadamard.c` | Compute Hadamard modes |
| `compute_control_modes.c` | Compute AO control modes in both input (WFS) and output (DM) space |
| `compute_masksWFSDM.c` | Compute maskswfsdm module |
| `compute_straight_CM.c` | Compute straight cm module |
| `generateRMWFS.c` | Generatermwfs module |
| `maskextrapolate.c` | Maskextrapolate module |
| `modes_spatial_extrapolate.c` | Modes spatial extrapolate module |

## Standalone Executables

| Executable | Source File | Description |
|------------|-------------|-------------|
| `cacao-fpsexec-cacaocc-maskextrapolate` | `maskextrapolate.c` | Maskextrapolate module |
| `cacao-fpsexec-cacaocc-actsamp2d` | `actmap_sample2D.c` | Actmap sample2d module |
| `cacao-fpsexec-cacaocc-rm2zonal` | `RM2zonal.c` | Rm2zonal module |
| `cacao-fpsexec-cacaocc-comphadamard` | `computeHadamard.c` | Compute Hadamard modes |
| `cacao-fpsexec-cacaocc-genrmwfs` | `generateRMWFS.c` | Generatermwfs module |
| `cacao-fpsexec-cacaocc-compmaskwfsdm` | `compute_masksWFSDM.c` | Compute maskswfsdm module |
| `cacao-fpsexec-cacaocc-compstrcm` | `compute_straight_CM.c` | Compute straight cm module |
| `cacao-fpsexec-cacaocc-compctrlmodes` | `compute_control_modes.c` | Compute AO control modes in both input (WFS) and output (DM) space |

## Dependencies
- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
