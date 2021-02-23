#!/bin/bash

FILENAME="cacaoloop01.wfsref.fits"

if [ ! -f ${FILENAME} ]; then
echo "Downloading $FILENAME"
wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1x2O_ZVMkEzTT66bthU8t_g9mGs2g_dQK' -O ${FILENAME}
fi


