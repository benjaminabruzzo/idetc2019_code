#!/bin/bash

DATE=$1
START=$2
STOP=$3

i=$2
echo "i = $2"
echo "Stop = $3"
while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"
  roslaunch hast simckf.launch date:=$DATE run:=$RUN
	i=$((i+=1))
done


. scripts/datetoneptune.sh $1
