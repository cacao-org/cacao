[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/8fc93c97bde340078b02340e71b10580)](https://www.codacy.com/app/oguyon/CACAO?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=CACAO-org/CACAO&amp;utm_campaign=Badge_Grade)

![](AlphaRelease.svg)


# cacao : Compute And Control for Adaptive Optics

![](cacao-logo.png =100x100)

**This is an alpha release**


## Overview


Computation engine for adaptive optics control; functions accessible through a command line interface (CLI). Holds images in RAM, with image stream support (shared memory with low-latency IPC support).

Written in C, optimized for performance.
Executable launches a command line interface (CLI). Type "help" in the CLI to get started.


CACAO uses [MILK](https://github.com/milk-org/MILK)


## Downloading and installing 

The CACAO package follows the standard git clone steps and GNU build process :

```bash
git clone --recursive -j 4 https://github.com/CACAO-org/CACAO
cd CACAO
autoreconf -vif
./configure
make
make install
```

Note: On OS X you need to use gcc-mp-5 for openMP:

```bash
./configure "CC=/opt/local/bin/gcc-mp-5" CPPFLAGS="-I/usr/include/malloc/ -I/opt/local/include/readline" LDFLAGS="-L/opt/local/lib/"
(Replace "/opt/local/" is the location of your installed libraries. )
```

## Reporting bugs, issues

Report bugs and issues on [this page]( https://github.com/CACAO-org/CACAO/issues )

## Contributing to project

See [coding standards]( http://CACAO-org.github.io/CACAO/html/page_coding_standards.html ) 

## Documentation

[Online documentation]( http://CACAO-org.github.io/CACAO/html/index.html ) 

## Libraries

The following libraries are used:

- libtool
- automake
- readline, for reading the command line input
- ncurses-dev
- flex, for parsing the command line input
- bison, to interpret the command line input
- fftw, for performing Fourier Transforms
- gsl, for math functions and tools
- fitsio, for reading and writing FITS image files
- CUDA, CuBLAS, MAGMA for GPU acceleration (optional)

If you use NVIDIA GPUs, install cuda and magma libraries, and add "--enable-cuda and --enable-magma" options to the configure command.

## Getting Started

All functions are accessible from the command line interface (CLI). Enter the CLI and type "help" for instructions.

```bash
./bin/CACAO
```

## LICENCE

[GNU General Public License v3.0]( https://github.com/CACAO-org/CACAO/blob/master/LICENCE.txt )
