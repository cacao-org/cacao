#!/usr/bin/bash

FILENAME="respM.fits"

if [ ! -f $FILENAME ]; then

	# unpack compressed file
	funpack respM.fits.fz
fi
