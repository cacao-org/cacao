# Module: AOloopControl

Module responsible for `AOloopControl` functionality.

## Standalone Executables

| Executable                        | Source File     | Description                                   |
| --------------------------------- | --------------- | --------------------------------------------- |
| `cacao-fpsexec-cacao-zonalfilter` | `zonalfilter.c` | Apply zonal filtering in DM space             |
| `cacao-fpsexec-mfilt`             | `modalfilter.c` | Apply modal filtering following FPS practices |
| `cacao-fpsexec-modalwhite`        | `modalwhite.c`  | Closed-loop residual whiteness & OOS monitor  |
| `cacao-fpsexec-modalog`           | `modalog.c`     | Online closed-loop DM-poke optical gain (IV)  |
| `cacao-fpsexec-modalopt`          | `modalopt.c`    | Online per-mode gain & leak autotuner         |

## Dependencies

- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
