#!/usr/bin/env bash

FILENAME="respM.fits"

if [ ! -f $FILENAME ]; then
	# upack compressed file
	funpack scexao-NIRPL-zrespM.fits.fz
    mv scexao-NIRPL-zrespM.fits ${FILENAME}
fi
