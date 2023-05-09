#!/usr/bin/env bash

FILENAME="respM.fits"

if [ ! -f $FILENAME ]; then
    funpack ${FILENAME}.fz
fi
