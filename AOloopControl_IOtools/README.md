# Module: AOloopControl_IOtools

Module responsible for `AOloopControl_IOtools` functionality.

## Source Files

| File | Description |
|------|-------------|
| `WFScamsim.c` | camera simulation for WFS |
| `WFSmap.c` | remap WFS image |
| `acquireWFSim.c` | No description available. |
| `acquireWFSspec.c` | acquire spectra - a stripped-down version of acquireWFSim for dispersed WFS |
| `ao188_preprocessor.c` | Convert ao188 APD data into curvature + SH data |
| `findspots.c` | Find spots in WFS image |
| `spotpos.c` | Measure spot position, photocenter |

## Standalone Executables

| Executable | Source File | Description |
|------------|-------------|-------------|
| `cacao-fpsexec-cacaoiot-acquireWFS` | `acquireWFSim.c` | No description available. |

## Dependencies
- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
