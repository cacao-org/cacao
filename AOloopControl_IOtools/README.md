# Module: AOloopControl_IOtools

Module responsible for `AOloopControl_IOtools` functionality.

## Source Files

| File                   | Description                                                                 |
| ---------------------- | --------------------------------------------------------------------------- |
| `PyWFSsim.c`           | cheap pyramid WFS simulation: DM slopes to four pupil images                |
| `WFScamsim.c`          | camera simulation for WFS                                                   |
| `WFSmap.c`             | remap WFS image                                                             |
| `acquireWFSim.c`       | Acquirewfsim module                                                         |
| `acquireWFSspec.c`     | acquire spectra - a stripped-down version of acquireWFSim for dispersed WFS |
| `ao188_preprocessor.c` | Convert ao188 APD data into curvature + SH data                             |
| `findspots.c`          | Find spots in WFS image                                                     |
| `spotpos.c`            | Measure spot position, photocenter                                          |

## Standalone Executables

| Executable                          | Source File      | Description         |
| ----------------------------------- | ---------------- | ------------------- |
| `cacao-fpsexec-cacaoiot-acquireWFS` | `acquireWFSim.c` | Acquirewfsim module |
| `cacao-fpsexec-cacaoiot-pywfssim`   | `PyWFSsim.c`     | cheap PyWFS simulation |

## Dependencies

- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
