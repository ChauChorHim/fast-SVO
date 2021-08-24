#!/bin/bash

echo This scripts will run sequences 00 - 10 of KIITI on the algrithm 

SEQUENCE=0

while [ $SEQUENCE -lt 11 ]; do
    if [ $SEQUENCE = "0" ] || [ $SEQUENCE = "1" ] || [ $SEQUENCE = "2"]; then
        ./stereo_kitti ../KITTI00-02.yaml ../../dataset/ $SEQUENCE
    elif [ $SEQUENCE = "3" ]; then
        ./stereo_kitti ../KITTI03.yaml ../../dataset/ $SEQUENCE
    else
        ./stereo_kitti ../KITTI04-12.yaml ../../dataset/ $SEQUENCE
    fi
        
    let SEQUENCE=SEQUENCE+1

done
