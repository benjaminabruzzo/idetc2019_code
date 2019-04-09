#!/bin/bash 

DATE=$1
RUN=$2
cp -r ~/ros/src/metahast/hast/ ~/ros/data/$DATE/$RUN/src
rm -rf ~/ros/data/$DATE/$RUN/src/.git
rm -rf ~/ros/data/$DATE/$RUN/src.zip
zip -r ~/ros/data/$DATE/$RUN/src.zip ~/ros/data/$DATE/$RUN/src
rm -rf ~/ros/data/$DATE/$RUN/src
#	REMEBER chmod +x zipper.sh

zip -r ~/dotros.zip ~/.ros/camera_info
cp ~/dotros.zip ~/ros/data/$DATE/$RUN/
rm ~/dotros.zip