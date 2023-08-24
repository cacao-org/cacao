#!/usr/bin/env bash

FILENAME="respM.fits"

if [ ! -f $FILENAME ]; then

    ggID='1MnNo9WfRHuKik3gpEHEjZHgt8_PM0T1a'
    ggURL='https://drive.google.com/uc?export=download'
    filename="$(curl -sc /tmp/gcookie "${ggURL}&id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
    getcode="$(awk '/_warning_/ {print $NF}' /tmp/gcookie)" || touch /tmp/gcookie
    curl -Lb /tmp/gcookie "${ggURL}&confirm=${getcode}&id=${ggID}" -o "${filename}"
    mv scexao-vispyr-bin2-zrespM.fits ${FILENAME}

fi

