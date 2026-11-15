#!/usr/bin/env bash

MAPSIZE=50
ZONERADIUS="0.07"
ZONEGAP="0.003"

# Build a FITS format actuator list from the input ASCII list of actuators,
# save actpos.fits (2 x 336), and compute the Voronoi tessellation mapping.fits
#
{
  echo '@calc "actpos = zeros(2, 336)"'
  awk '{printf("@calc \"actpos[0, %d] = %.4f; actpos[1, %d] = %.4f\"\n", $1, $2*0.95, $1, $3*0.95)}' act2Dpos.txt
  echo 'iofits.saveFITS actpos "!actpos.fits"'
  echo "imgen.voronoi actpos actmap $MAPSIZE $MAPSIZE $ZONERADIUS $ZONEGAP"
  echo 'iofits.saveFITS actmap "!mapping.fits"'
  echo 'exit'
} | cacao-cli
