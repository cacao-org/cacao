#!/bin/bash

FILENAME="cacaoloop01.respM.fits"

if [ ! -f $FILENAME ]; then

ggID='1D1SfU2gZpzEU-tRZK0bJsL6MUYZ09Ils'
ggURL='https://drive.google.com/uc?export=download'
filename="$(curl -sc /tmp/gcokie "${ggURL}&id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
getcode="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"
curl -Lb /tmp/gcokie "${ggURL}&confirm=${getcode}&id=${ggID}" -o "${filename}"
mv LHS_zrespM_2018-01-25_05:09:52.fits ${FILENAME}

fi

