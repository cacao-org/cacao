#!/usr/bin/env bash

FILENAME="wfsref.fits"

if [ ! -f ${FILENAME} ]; then
    # uncompressing file
    funpack scexao-NIRPL-wfsref.fits.fz
    mv scexao-NIRPL-wfsref.fits ${FILENAME}
fi

