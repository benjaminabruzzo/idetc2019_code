#!/bin/bash
# . loopOverCSV.sh 20170911 39 60


FOLDER="/Users/$USER/ros/data"
DATE=$1

DATAFOLDER="$FOLDER/$DATE/csv"

i=$2
STOP=$3

echo "i = $2"
echo "Stop = $3"
while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"

	# CSVFILE="$DATAFOLDER/hast_$RUN.csv"
	# 	python splitcsv.py "$CSVFILE"
	# CSVFILE="$DATAFOLDER/csv_helical/hast_$RUN.csv"
	# 	python splitcsv.py "$CSVFILE"
	CSVROOT="$DATAFOLDER/"
		python splitcsv_q.py "$CSVROOT" "$DATE" "$RUN"

	i=$((i+=1))
done
