#!/usr/bin/env bash

MAPSIZE=50
ZONERADIUS="0.07"
ZONEGAP="0.003"

# Build a FITS format actuator list from the input ASCII list of actuators
# The FITS file, named actpos.fits, has size 2 x 336, for 2 dimensions and 336 actuators
#
echo "milk-all << EOF" > mk2DactposFITS
echo "mk2Dim actpos 2 336" >> mk2DactposFITS
awk '{printf("setpix actpos %.4f 0 %d\nsetpix actpos %.4f 1 %d\n", $2*0.95, $1, $3*0.95, $1)}' act2Dpos.txt >> mk2DactposFITS
echo "saveFITS actpos actpos.fits" >> mk2DactposFITS
echo "exitCLI" >> mk2DactposFITS
echo "EOF" >> mk2DactposFITS

chmod +x mk2DactposFITS
./mk2DactposFITS

# Build the Voronoi tessellation
#
milk-all << EOF
loadfits actpos.fits actpos
imgen.voronoi actpos actmap $MAPSIZE $MAPSIZE $ZONERADIUS $ZONEGAP
saveFITS actmap mapping.fits
listim
exitCLI
EOF

