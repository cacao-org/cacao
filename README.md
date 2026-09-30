# cacao: Compute And Control for Adaptive Optics

[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
[![CMake](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/cmake.yml/badge.svg?branch=dev)](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/cmake.yml)
[![Docker](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/docker-image.yml/badge.svg?branch=dev)](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/docker-image.yml)
[![Docs](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/docs.yml/badge.svg?branch=dev)](https://github.com/CaltechOpticalObservatories/cacao/actions/workflows/docs.yml)

<img align="left" src="cacao-logo-250pix.png" alt="cacao logo">

cacao is a real-time computation engine for adaptive optics control. It is
written in C for low latency, uses multi-core CPUs and GPUs, and keeps data in
shared-memory image streams with low-latency inter-process signaling.

cacao is a plugin of [milk](https://github.com/milk-org/milk). It is built
inside milk's source tree and adds AO modules, CLI commands and loop setup
scripts on top of milk.

This repository is the Caltech Optical Observatories fork of
[cacao-org/cacao](https://github.com/cacao-org/cacao).

<br clear="left"/>

## Documentation

The documentation is published at
<https://caltechopticalobservatories.github.io/cacao/>. Its sources are in
[`docs/`](docs/index.md):

- [Installation](docs/install.md)
- [Running a loop from an example](docs/examples.md)
- [Modules](docs/modules.md)
- [Development](docs/development.md)

To build the HTML documentation, run `tox -e docs`.

## Installing

Install the prerequisites first ([list](docs/install.md#prerequisites)). Then:

```bash
git clone https://github.com/milk-org/milk.git
cd milk
git clone https://github.com/CaltechOpticalObservatories/cacao.git plugins/cacao-src

cmake -S . -B _build -DCMAKE_BUILD_TYPE=Release -DINSTALLMAKEDEFAULT=ON
cmake --build _build -j "$(nproc)"
sudo cmake --install _build
```

Run `cmake` from the milk source root as shown. Configuring from inside
`_build/` (`cmake ..`, or milk's `compile.sh`) silently leaves cacao out of the
build.

Then set `MILK_ROOT`, `MILK_INSTALLDIR`, `PATH` and `PKG_CONFIG_PATH`, and set
up the shared-memory directory, as described in the
[installation guide](docs/install.md#environment).

To build in Docker instead: `docker build -t cacao .`

## Getting started

```bash
cacao-loop-deploy -h                      # list example configurations
cacao-loop-deploy -c scexao-vispyr-bin2   # copy an example into the current directory
cacao-loop-deploy -r scexao-vispyr-bin2   # run its setup
```

See [Running a loop from an example](docs/examples.md).

## Help and issues

Report bugs on the
[issue tracker](https://github.com/CaltechOpticalObservatories/cacao/issues).
