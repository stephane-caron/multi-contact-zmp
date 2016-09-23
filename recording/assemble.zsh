#!/bin/zsh

if [[ $# == 0 ]] then
    echo "Usage: $0 <camera_folder>"
    exit
fi

CAMERA=$1
FRAMERATE=33  # assumes dt=3e-2 [s]
EXT=mp4

avconv -r ${FRAMERATE} -qscale 1 -i ${CAMERA}/%05d.png ./${CAMERA}.${EXT}
