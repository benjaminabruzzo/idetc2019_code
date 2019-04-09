#!/bin/bash
DATE=$1
# rsync -avI -e ssh tbrl@192.168.100.25:~/vicon/hast/$DATE/csv/ $SDB/hast/data/$DATE/csv/
rsync -avmI --include='*.csv' --include='*/' --exclude='*' tbrl@192.168.100.25:~/vicon/hast/$DATE/csv $SDB/hast/data/$DATE/data/csv


# Calling a bash script in windows: maybe: C:\cygwin\bin\bash testit.sh
