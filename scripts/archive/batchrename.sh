DATE=$1
RUN=$2

SCRIPTHOME=/home/$USER/ros/src/hast/scripts
HOME=/home/$USER/ros/data/$DATE/$RUN
FOLDER1=$HOME/bad/original
FOLDER2=$HOME/rect

cd $FOLDER1
n=$(ls -l | wc -l)
echo $n
m=1

for j in $FOLDER1/*
do
  extension="${j##*.}"
  filename="${j%.*}"
  if [ $extension = "png" ]
  then
    echo $filename.$extension
#     RUN="$(printf "%06d" $n)"
#     cp "$j" "$FOLDER1/bulk_$RUN.$extension"
    m=$((m+=1))
#     rm "$j"
  fi
done
echo "DONE! $m"
cd $SCRIPTHOME
