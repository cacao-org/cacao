#!/usr/bin/env bash

FILENAME="wfsref.fits"

if [ ! -f ${FILENAME} ]; then
    echo "Downloading $FILENAME"
    wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1wg9is7ZGZao2OZsgktmGTJgvFi4_XKAE' -O ${FILENAME}.gz
    gunzip ${FILENAME}.gz
fi

