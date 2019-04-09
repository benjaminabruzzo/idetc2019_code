#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
RUN=$2
EXCLUDE=$3

if [ $EXCLUDE = "--exclude" ]; then
  rsync -aI --exclude='*.png' -e ssh /home/$USER/ros/data/$DATE/$RUN benjamin@neptune.local:/media/benjamin/devsdb/hast/data/$DATE/
fi
if [ $EXCLUDE = "--include" ]; then
  rsync -aI -e ssh /home/$USER/ros/data/$DATE/$RUN benjamin@neptune.local:/media/benjamin/devsdb/hast/data/$DATE/
fi
