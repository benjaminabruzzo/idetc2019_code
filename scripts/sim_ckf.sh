#!/bin/bash
# call from neptune

echo "removing old .mat files"
rm ~/ros/data/$1/$2/*.mat
echo "pull and remake packages"
git pull bb master
catkin_remake
echo "launch"
# roslaunch hast simckf.launch user:=$USER date:=$1 run:=$2
roslaunch hast sim.launch date:=$1 run:=$2
# echo "call matlab to generate mat files"
# matlab -r 'clas_terminal(); quit'
# matlab nodisplay -nodesktop -r "run('/home/benjamin/ros/src/hast/scripts/clas_terminal.m');"
# echo "download new .mat files"
. scripts/datetosaturn.sh $1 --exclude
echo -e "\a"

