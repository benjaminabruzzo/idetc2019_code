#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
RUN=$2
INCLUDE=$3

if [ $INCLUDE = "--include" ]; then
  rsync -avI  -e ssh benjamin@neptune.local:/media/benjamin/devsdb/hast/data/$DATE/$RUN ~/ros/data/$DATE/
fi
if [ $INCLUDE = "--exclude" ]; then
  rsync -avI --exclude='*.png' -e ssh benjamin@neptune.local:/media/benjamin/devsdb/hast/data/$DATE/$RUN ~/ros/data/$DATE/
fi
