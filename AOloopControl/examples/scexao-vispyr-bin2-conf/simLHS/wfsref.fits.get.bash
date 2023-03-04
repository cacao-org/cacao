#!/usr/bin/env bash

FILENAME="wfsref.fits"

if [ ! -f ${FILENAME} ]; then
    echo "Downloading $FILENAME"
    wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=15V2iMIeflC3gCdic9hTSNOpiuFAAKifa' -O ${FILENAME}
fi
