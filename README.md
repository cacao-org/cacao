[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)

Latest Version (off main branch): [![latesttag](https://img.shields.io/github/tag/cacao-org/cacao.svg)](https://github.com/milk-org/cacao/tree/main)

| Branch    | Build   | Docker Deployment    | Travis-CI    | Activity   |
|-------------|-------------|-------------|-------------|-------------|
**main**|[![CMake badge](https://github.com/cacao-org/cacao/actions/workflows/cmake.yml/badge.svg?branch=main)](https://github.com/cacao-org/cacao/actions/workflows/cmake.yml)|[![CMake badge](https://github.com/cacao-org/cacao/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/cacao-org/cacao/actions/workflows/docker-image.yml)|[![Build Status](https://www.travis-ci.com/cacao-org/cacao.svg?branch=main)](https://www.travis-ci.com/cacao-org/cacao)|![lastcommit](https://img.shields.io/github/last-commit/cacao-org/cacao/main.svg)|
**dev**|[![CMake badge](https://github.com/cacao-org/cacao/actions/workflows/cmake.yml/badge.svg?branch=dev)](https://github.com/cacao-org/cacao/actions/workflows/cmake.yml)|[![CMake badge](https://github.com/cacao-org/cacao/actions/workflows/docker-image.yml/badge.svg?branch=dev)](https://github.com/cacao-org/cacao/actions/workflows/docker-image.yml)|[![Build Status dev](https://www.travis-ci.com/cacao-org/cacao.svg?branch=dev)](https://www.travis-ci.com/cacao-org/cacao)|![lastcommit](https://img.shields.io/github/last-commit/cacao-org/cacao/dev.svg)|


Code metrics (dev branch) :
[![CodeScene Code Health](https://codescene.io/projects/14780/status-badges/code-health)](https://codescene.io/projects/14780)
[![CodeScene System Mastery](https://codescene.io/projects/14780/status-badges/system-mastery)](https://codescene.io/projects/14780)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/6eefa0e1c1254889b1e2f6fda55930ca)](https://www.codacy.com/gh/cacao-org/cacao/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=cacao-org/cacao&amp;utm_campaign=Badge_Grade)
[![CodeFactor](https://www.codefactor.io/repository/github/cacao-org/cacao/badge)](https://www.codefactor.io/repository/github/cacao-org/cacao)


# cacao : Compute And Control for Adaptive Optics


<img align="left" src="cacao-logo-250pix.png">

cacao is a computation engine for adaptive optics control.

**Compute Performance**: Uses multi-core CPUs and GPGPUs for [high computing throughput](https://github.com/cacao-org/cacao/wiki/Compute-Performance-Benchmarks). Written in C, optimized for performance. Holds images in RAM, with image stream support (shared memory with low-latency IPC support). cacao uses [milk](https://github.com/milk-org/milk).


**User input**: Executable launches a [command line interface (CLI)](https://cacao-org.github.io/cacao/page_userinput.html) from which functions are accessible. Type "help" in the CLI to get started.


**Modular**, [easy to add functions](https://cacao-org.github.io/cacao/page_LoadingModules.html), loaded at runtime as shared objects.



---


## Getting help, Documentation

Topic                        |  Chat room                             |  Documentation             |
-----------------------------|----------------------------------------|--------------------|
How to use cacao ?           | [![Gitter](https://badges.gitter.im/cacao-org/community.svg)](https://gitter.im/cacao-org/community)  Community | [Online documentation]( http://CACAO-org.github.io/cacao/index.html )  |
Configuring computer system  | [![Gitter](https://badges.gitter.im/cacao-org/RTCconfig.svg)](https://gitter.im/cacao-org/RTCconfig)  Hardware / OS setup | [RTC setup]( https://github.com/cacao-org/cacao/wiki/Seeting-up-a-RTC-system ) |
Software development         | [![Gitter](https://badges.gitter.im/cacao-org/codedev.svg)](https://gitter.im/cacao-org/codedev)      Developers | [coding standards]( http://CACAO-org.github.io/cacao/page_coding_standards.html ) and  [Online documentation]( http://CACAO-org.github.io/cacao/index.html )|



Report bugs and issues on [this page]( https://github.com/cacao-org/cacao/issues ) and discuss them on the [Developers chat](https://gitter.im/cacao-org/codedev).


---

## Installing cacao

Install pre-requisite packages as needed. Check the [Dockerfile](https://github.com/cacao-org/cacao/blob/dev/Dockerfile) for list.

cacao is a plugin of [milk](https://github.com/milk-org/milk).

To install :

```bash
git clone https://github.com/milk-org/milk.git
cd milk
./fetch_cacao_dev.sh
mkdir _build
cd _build
cmake ..
make
sudo make install
```

Alternatively, the ./compile.sh script for can be run to install. Check compilation options with compile.sh -h option.

---

## Documentation

See [cacao's wiki](https://github.com/cacao-org/cacao/wiki) for detailed instructions to install, configure and use cacao.


---

## Getting Started

All functions are accessible from the command line interface (CLI). Enter the CLI and type "help" for instructions.

```bash
cacao
```

To set up a cacao AO loop, use the cacao-loop-deploy script. Consult help with :

```bash
cacao-loop-deploy -h
```

---

## Tools

Both cacao and milk use a common shared memory data stream format. See [ImageStreamIO module](https://github.com/milk-org/ImageStreamIO) for details.

### Viewing real-time data streams

Additional software is required to view real-time data streams. Several options exist:

  * [shmimviewGTK](https://github.com/milk-org/shmimviewGTK), a lightweight efficient GTK-based viewer.
  * [milk2ds9](https://github.com/jaredmales/milk2ds9) uses ds9 to view data streams (convenient for ds9 users)
  * [rtimv](https://github.com/jaredmales/rtimv), a qt-based image viewer, higher performance than ds9 option
  * [shmimviewqt](https://github.com/milk-org/shmimviewqt), another qt-based option (less polished than rtimv)
  * [xaosim](https://github.com/fmartinache/xaosim)'s shmview image viewer, using python qt interface

### Python interface to data streams

Python users can read/write milk/cacao's data streams using additional packages:

  * [pyMilk](https://github.com/milk-org/pyMilk) provides an interface to data streams.
  * [xaosim](https://github.com/fmartinache/xaosim) includes a python interface to data streams.. and much more
Both stream interfaces above are cross-compatible.



---
