#!/bin/bash
# sudo apt-get install imagemagick
# . loopconvert [date] [run] 
cd ~/ros/data/$1/$2/original
for file in *.png; do
 convert -enhance -equalize -contrast $file "${file%.png}_eec.png"
done