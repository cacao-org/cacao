#!/usr/bin/bash

FILENAME="wfsref.fits"

if [ ! -f $FILENAME ]; then

	# unpack compressed file
	funpack wfsref.fits.fz
fi
