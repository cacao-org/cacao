#!/usr/bin/env bash

FILENAME="wfsref.fits"

if [ ! -f ${FILENAME} ]; then
    echo "Downloading $FILENAME"
    wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1JxfPrckTb-rPysNHKtHFkr9g0dv7IfN4' -O ${FILENAME}.gz
    gunzip ${FILENAME}.gz
fi

