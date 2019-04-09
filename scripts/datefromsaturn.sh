#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
INCLUDE=$2

if [ $INCLUDE = "--include" ]; then
  rsync -avI  -e ssh benjamin@saturn.local:/Users/benjamin/ros/data/$DATE/ ~/ros/data/$DATE/ 
else
  rsync -avI --exclude='*.png' -e ssh benjamin@saturn.local:/Users/benjamin/ros/data/$DATE/ ~/ros/data/$DATE/ 
fi
