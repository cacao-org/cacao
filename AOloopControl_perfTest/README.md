# Module: AOloopControl_perfTest

Module responsible for `AOloopControl_perfTest` functionality.

## Source Files

| File                    | Description                                   |
| ----------------------- | --------------------------------------------- |
| `compRMsensitivity.c`   | mcompute response matrix sensitivity          |
| `mlat.c`                | Mlat module                                   |
| `mlat_decode.c`         | Mlat decode module                            |
| `streamlogtimesample.c` | measure hardware latency                      |
| `wfsrefoptimselect.c`   | Optimize WFS reference by PSF-based selection |
| `zoptsearch.c`          | zonal optimizatoin search                     |

## Standalone Executables

| Executable                          | Source File             | Description                                   |
| ----------------------------------- | ----------------------- | --------------------------------------------- |
| `cacao-fpsexec-cacaopt-compRMsens`  | `compRMsensitivity.c`   | mcompute response matrix sensitivity          |
| `cacao-fpsexec-cacaopt-mlat`        | `mlat.c`                | Mlat module                                   |
| `cacao-fpsexec-cacaopt-zoptsearch`  | `zoptsearch.c`          | zonal optimizatoin search                     |
| `cacao-fpsexec-cacaopt-wfsroptsel`  | `wfsrefoptimselect.c`   | Optimize WFS reference by PSF-based selection |
| `cacao-fpsexec-cacaopt-mlatdecode`  | `mlat_decode.c`         | Mlat decode module                            |
| `cacao-fpsexec-cacaopt-slogtsample` | `streamlogtimesample.c` | measure hardware latency                      |

## Dependencies

- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
