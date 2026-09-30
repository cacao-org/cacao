# Installation

cacao does not build on its own. It is a plugin of
[milk](https://github.com/milk-org/milk): the cacao sources are placed in
`milk/plugins/cacao-src/` and compiled as part of milk.

cacao targets Linux. It is built and tested on Ubuntu 24.04.

## Prerequisites

On Ubuntu/Debian:

```bash
sudo apt-get install \
    git cmake make pkg-config gcc g++ gfortran \
    bison flex libbison-dev libfl-dev \
    libcfitsio-dev libgsl-dev libfftw3-dev libncurses-dev libreadline-dev \
    libopenblas-dev liblapacke-dev \
    python3-dev pybind11-dev python3-pybind11
```

This is the same list the repository's `Dockerfile` installs, which CI
builds on every change.

Optional:

- CUDA toolkit, for GPU computation (configure with `-DUSE_CUDA=ON`)
- MAGMA, for GPU linear algebra (`-DUSE_MAGMA=ON`, requires CUDA)
- Intel MKL, used instead of OpenBLAS when found by pkg-config (`mkl-sdl`)
- `tmux`, used by the cacao loop setup scripts

## Build and install

```bash
git clone https://github.com/milk-org/milk.git
cd milk
git clone https://github.com/CaltechOpticalObservatories/cacao.git plugins/cacao-src

cmake -S . -B _build -DCMAKE_BUILD_TYPE=Release -DINSTALLMAKEDEFAULT=ON
cmake --build _build -j "$(nproc)"
sudo cmake --install _build
```

:::{important}
Run `cmake` from the milk source root, as shown. milk finds plugins with a
`find plugins` relative to the directory `cmake` is run from, so configuring
from inside `_build/` (`cd _build; cmake ..`, which is what milk's
`compile.sh` does) silently builds milk without cacao.
:::

milk installs to `/usr/local/milk-<version>`. `-DINSTALLMAKEDEFAULT=ON` also
points the symlink `/usr/local/milk` at that version.

milk's `fetch_cacao_dev.sh` script can clone cacao for you. By default it
clones the upstream `cacao-org/cacao` `dev` branch. To use this fork instead:

```bash
CACAO_REPOSITORY=https://github.com/CaltechOpticalObservatories/cacao.git \
CACAO_BRANCH=dev \
./fetch_cacao_dev.sh
```

The script only clones when `plugins/cacao-src` does not exist. Otherwise it
runs `git pull` in the existing checkout.

## Environment

Add to `~/.bashrc` (or equivalent):

```bash
export MILK_ROOT=$HOME/src/milk          # milk source directory (edit as needed)
export MILK_INSTALLDIR=/usr/local/milk
export PATH=${PATH}:${MILK_INSTALLDIR}/bin
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${MILK_INSTALLDIR}/lib/pkgconfig
```

`MILK_ROOT` must be set. `cacao-loop-deploy` reads the example
configurations from `$MILK_ROOT/plugins/cacao-src`.

### Shared memory directory

Image streams and their semaphores are files in a shared-memory directory. The
directory is chosen in this order:

1. `$MILK_SHM_DIR`, if it is set and exists
2. `/milk/shm`
3. `/tmp`

For real-time use, mount a tmpfs at `/milk/shm`:

```bash
sudo mkdir -p /milk/shm
echo "tmpfs /milk/shm tmpfs rw,nosuid,nodev" | sudo tee -a /etc/fstab
sudo mount /milk/shm
```

## Docker

The repository's `Dockerfile` builds milk with the cacao sources from the
current checkout:

```bash
docker build -t cacao .
docker run --rm -it cacao
```

Build against a different milk branch with
`--build-arg MILK_REF=<branch>`.

## Check the installation

```bash
milk-check
cacao-loop-deploy -h
```
