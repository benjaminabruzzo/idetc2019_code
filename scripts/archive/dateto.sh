#!/bin/bash
# . dateto mars 20170731 --include

HOST=$1
DATE=$2
INCLUDE=$3

if [ $INCLUDE = "--include" ]; then
  rsync -avI  -e ssh ~/ros/data/$DATE/ benjamin@$HOST.local:~/ros/data/$DATE/
fi
if [ $INCLUDE = "--exclude" ]; then
  rsync -avI --exclude='*.png' -e ssh ~/ros/data/$DATE/ benjamin@$HOST.local:~/ros/data/$DATE/
fi
