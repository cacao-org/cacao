#!/usr/bin/env bash

FILENAME="respM.fits"

if [ ! -f $FILENAME ]; then

    ggID='1XEAKFsXBH6ppJhKbZdUlCygO1YfVLd1O'
    ggURL='https://drive.google.com/uc?export=download'
    filename="$(curl -sc /tmp/gcokie "${ggURL}&id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
    getcode="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"
    curl -Lb /tmp/gcokie "${ggURL}&confirm=${getcode}&id=${ggID}" -o "${filename}"
    gunzip scexao-NIRPL-zrespM.fits.gz
    mv scexao-NIRPL-zrespM.fits ${FILENAME}
fi
