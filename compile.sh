#!/bin/sh

# Compile script for milk_package
# Customize / add you own options
# Do not commit

mkdir -p _build
cd _build

# cmake ..
cmake .. -DUSE_MAGMA=ON -DUSE_CUDA=ON
# cmake .. -DUSE_MAGMA=ON -Dpython_build=ON -DEXTRAMODULES="milk_module_example"`

make -j$(nproc)

sudo make install
