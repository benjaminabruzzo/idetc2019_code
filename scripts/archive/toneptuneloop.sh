#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
START=$2
STOP=$3
echo "Start = $2"
echo "Stop = $3"

i=$START
while [ $i -le $STOP ]
do
  RUN="$(printf "%03d" $i)"
  rsync -avI -e ssh /home/$USER/ros/data/$DATE/$RUN benjamin@neptune.local:/media/benjamin/devsdb/hast/data/$DATE/$RUN
  # rsync -avI --exclude='*/' -e ssh /home/$USER/ros/data/$DATE/$RUN/ benjamin@neptune.local:/home/benjamin/ros/data/$DATE/$RUN/
	i=$((i+=1))
done
