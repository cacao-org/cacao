# Builds milk with this cacao checkout as a plugin.
#
# The cacao sources come from the Docker build context (this repository), not
# from a fresh clone, so the image always reflects the working tree / commit
# being built. milk itself is cloned; override with --build-arg:
#
#   docker build -t cacao .
#   docker build --build-arg MILK_REF=main -t cacao .

FROM ubuntu:24.04

ARG MILK_REPOSITORY=https://github.com/milk-org/milk.git
ARG MILK_REF=dev

RUN useradd -ms /bin/bash milkuser
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        make \
        dpkg-dev \
        libc6-dev \
        cmake \
        pkg-config \
        python3-dev \
        libcfitsio-dev \
        pybind11-dev \
        python3-pybind11 \
        libgsl-dev \
        libfftw3-dev \
        libncurses-dev \
        bison \
        flex \
        libbison-dev \
        libfl-dev \
        libreadline-dev \
        gcc \
        g++ \
        gfortran \
        libopenblas-dev \
        liblapacke-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${MILK_REF} ${MILK_REPOSITORY} /build
WORKDIR /build
COPY . /build/plugins/cacao-src
# Configure from the milk source root: milk discovers plugins with a
# `find plugins` relative to the working directory, so configuring from inside
# _build (as milk's compile.sh does) silently skips cacao.
RUN cmake -S . -B _build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build _build -j "$(nproc)" && \
    cmake --install _build
RUN ln -s /usr/local/milk-* /usr/local/milk

RUN mkdir /work
WORKDIR /work
ENV MILK_ROOT=/build
ENV MILK_INSTALLDIR=/usr/local/milk
ENV PATH=${PATH}:/usr/local/milk/bin
ENV PKG_CONFIG_PATH=/usr/local/milk/lib/pkgconfig
USER milkuser
