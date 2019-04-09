#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
INCLUDE=$2

if [ $INCLUDE = "--include" ]; then
  rsync -avI  -e ssh ~/ros/data/$DATE/ benjamin@saturn.local:/Users/benjamin/ros/data/$DATE/
else
  rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@saturn.local:/Users/benjamin/ros/data/$DATE/
fi

rsync -avI  -e ssh ~/ros/data/csv/ benjamin@saturn.local:/Users/benjamin/ros/data/csv/