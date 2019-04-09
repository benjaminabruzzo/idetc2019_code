#!/bin/bash
# . gazebofrom.sh ceres 20180504

WHO=$1
DATE=$2
i=$3
STOP=$4

rsync -avI --exclude='*.png' -e ssh benjamin@$WHO.local:~/ros/data/$DATE/ ~/ros/data/$DATE/
rsync -avmI --include='*.csv' --include='*/' --exclude='*' -e ssh benjamin@$WHO.local:~/ros/data/csv/ ~/ros/data/csv/

DATAFOLDER="/Users/benjamin/ros/data/csv/"

echo "i = $2"
echo "Stop = $3"
while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"
	python splitcsv_q.py "$DATAFOLDER" "$DATE" "$RUN"
	i=$((i+=1))
done

# %% garbage text
