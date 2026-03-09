# Module: AOloopControl_perfTest

Module responsible for `AOloopControl_perfTest` functionality.

## Source Files

| File | Description |
|------|-------------|
| `AOloopControl_perfTest_LinSim.c` | Adaptive Optics Control loop linear simulator |
| `compRMsensitivity.c` | mcompute response matrix sensitivity |
| `mlat.c` | No description available. |
| `mlat_decode.c` | No description available. |
| `streamlogtimesample.c` | measure hardware latency |
| `wfsrefoptimselect.c` | Optimize WFS reference by PSF-based selection |
| `zoptsearch.c` | zonal optimizatoin search |

## Standalone Executables

| Executable | Source File | Description |
|------------|-------------|-------------|
| `cacao-fpsexec-cacaopt-compRMsens` | `compRMsensitivity.c` | mcompute response matrix sensitivity |
| `cacao-fpsexec-cacaopt-mlat` | `mlat.c` | No description available. |
| `cacao-fpsexec-cacaopt-zoptsearch` | `zoptsearch.c` | zonal optimizatoin search |
| `cacao-fpsexec-cacaopt-wfsroptsel` | `wfsrefoptimselect.c` | Optimize WFS reference by PSF-based selection |
| `cacao-fpsexec-cacaopt-mlatdecode` | `mlat_decode.c` | No description available. |
| `cacao-fpsexec-cacaopt-slogtsample` | `streamlogtimesample.c` | measure hardware latency |

## Dependencies
- Implicit standard: `milkdata`, `ImageStreamIO`, `CLIcore`
