# call : $ . macdirs.sh 20160930 1 5
git pull bb master
DATE=$1
START=$2
STOP=$3

echo " "
echo $DATADIR$DATE
echo " "
echo "Start = $2"
echo "Stop = $3"

i=$START
while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"
  # echo $DATE/$RUN
	# echo roslaunch hast rebag.launch date:=$DATE run:=$RUN
	roslaunch hast rebag.launch date:=$DATE run:=$RUN

	i=$((i+=1))
done

#
# cd /home/benjamin/ros/data/20151217/
# # cd 006 && rm viconData_006.mat
# # cd ../007 && rm viconData_007.mat
# # cd ../008 && rm viconData_008.mat
# # cd ../009 && rm viconData_009.mat
#
#
# cd 001 && rm *.mat
# cd ../002 && rm *.mat
# cd ../003 && rm *.mat
# cd ../004 && rm *.mat
# cd ../005 && rm *.mat
# cd ../006 && rm *.mat
# cd /home/benjamin/ros/data/
