# cacao: Compute And Control for Adaptive Optics

cacao is a real-time computation engine for adaptive optics (AO) control. It is
written in C for low latency, runs on multi-core CPUs and GPUs, and holds data
in shared-memory image streams with low-latency inter-process signaling.

cacao is a plugin of [milk](https://github.com/milk-org/milk): it is compiled
inside milk's source tree and adds AO-specific modules, CLI commands and
scripts on top of milk's image streams, process management (processinfo) and
function parameter structure (FPS) frameworks.

```{toctree}
:maxdepth: 2
:caption: User guide

install
examples
```

```{toctree}
:maxdepth: 2
:caption: Reference

modules
```

```{toctree}
:maxdepth: 2
:caption: Development

development
```

## Related tools

cacao and milk share a common shared-memory stream format, defined by
[ImageStreamIO](https://github.com/milk-org/ImageStreamIO).

Real-time stream viewers:

- [shmimviewGTK](https://github.com/milk-org/shmimviewGTK): lightweight GTK viewer
- [rtimv](https://github.com/jaredmales/rtimv): Qt-based viewer
- [milk2ds9](https://github.com/jaredmales/milk2ds9): view streams in ds9
- [shmimviewqt](https://github.com/milk-org/shmimviewqt): Qt-based viewer
- [xaosim](https://github.com/fmartinache/xaosim): includes `shmview`

Python access to streams:

- [pyMilk](https://github.com/milk-org/pyMilk)
- [xaosim](https://github.com/fmartinache/xaosim)
