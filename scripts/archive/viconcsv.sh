#!/bin/bash
# . sendtomac 20151206 001

DATE=$1
# RUN=$2

# use to send image files
rsync -avI -e ssh tbrl@192.168.100.25:~/vicon/hast/$DATE/date/ $SDB/hast/data/$DATE/csv/
# rsync -avI -e ssh /home/$USER/ros/data/$DATE/$RUN/ benjamin@155.246.218.23:/home/benjamin/ros/data/$DATE/$RUN/

# use to ignore image files
# rsync -avI --exclude='*/' -e ssh /home/$USER/ros/data/$DATE/$RUN/ benjamin@155.246.218.61:/home/benjamin/ros/data/$DATE/$RUN/
# rsync -avI --exclude='*/' -e ssh /home/$USER/ros/data/$DATE/$RUN/ benjamin@neptune.local:/home/benjamin/ros/data/$DATE/$RUN/


# echo "rsync -avI -e ssh /home/$USER/ros/data/$DATE/$RUN/ benjamin@neptune.local:/home/benjamin/ros/data/$DATE/$RUN/"

#rsync –av ––delete -e ssh root@192.168.1.5:/home/techgage/
#scp /home/$USER/ros/data/$DATE/$RUN/* benjamin@neptune.local:/home/benjamin/ros/data/$DATE/$RUN/
# rsync -avI -e ssh /home/$USER/ros/data/$DATE/ benjamin@neptune.local:/home/benjamin/ros/data/$DATE/
