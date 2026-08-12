# Module: AOloopControl_DM

Module responsible for `AOloopControl_DM` functionality.

## Source Files

| File                      | Description                              |
| ------------------------- | ---------------------------------------- |
| `AOloopControl_DM_comb.c` | DM control - Combine DM channels         |
| `DMturbulence.c`          | DM turbulence simulation                 |
| `mk3Ddmgrid.c`            | Create DM grid patterns for calibrations |
| `pokerndmodes.c`          | poke mode values                         |

## Standalone Executables

| Executable             | Source File               | Description                      |
| ---------------------- | ------------------------- | -------------------------------- |
| `cacao-fpsexec-dmcomb` | `AOloopControl_DM_comb.c` | DM control - Combine DM channels |
| `cacao-fpsexec-dmturb` | `DMturbulence.c`          | DM turbulence simulation         |

## Dependencies

- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
