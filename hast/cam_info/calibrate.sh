### To calibrate cameras
# cut and paste between the lines:
# -------------------------------- Stereo
# Gatorboard:

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/pgrstereo/right/image_raw left:=/pgrstereo/left/image_raw right_camera:=/pgrstereo/right left_camera:=/pgrstereo/left --approximate=0.005
rosrun camera_calibration_parsers convert  kobuki_chameleon_20180208a.ini kobuki_chameleon_20180208a.yaml

mv /tmp/calibrationdata.tar.gz ~/ros/data/kobuki_chameleon_20180208c.tar.gz

matlab -nojvm -nodesktop -r "cd('/home/benjamin/ros/src/hast/cam_info');kobukiCalChameleonPGR('linux')" 
cp /home/benjamin/ros/src/hast/cam_info/16369047.yaml /home/benjamin/.ros/camera_info/ 
cp /home/benjamin/ros/src/hast/cam_info/16306423.yaml /home/benjamin/.ros/camera_info/



# -------------------------------- Stereo
# Gatorboard:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/pgrstereo/right/image_raw left:=/pgrstereo/left/image_raw right_camera:=/pgrstereo/right left_camera:=/pgrstereo/left --approximate=0.005

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --approximate=0.01
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.005

# Clipboard:
rosrun camera_calibration cameracalibrator.py --size 3x4 --square 0.052 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01
rosrun camera_calibration cameracalibrator.py --size 3x4 --square 0.052 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.1
rosrun camera_calibration cameracalibrator.py --size 3x4 --square 0.0505 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01

# Clear Clipboard:
rosrun camera_calibration cameracalibrator.py --size 5x4 --square 0.031 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --approximate=0.01


# scp /home/turtlebot/ros/src/hast/cam_info/StereoCalRaw20151229.yaml benjamin@jupiter.local:/Users/benjamin/git/hast/ros/turtlebot/cam_info/
# scp /home/turtlebot/ros/src/hast/cam_info/StereoCalRaw20151223.yaml benjamin@jupiter.local:/Users/benjamin/git/hast/ros/turtlebot/cam_info/
# scp turtlebot@192.168.1.4:/home/turtlebot/ros/src/hast/cam_info/StereoCalRaw20150612.yaml /Users/benjamin/Documents/ros_data/hast/src/
cp /tmp/calibrationdata.tar.gz /home/turtlebot/ros/src/hast/cam_info/Archive/calibrationdata_20151222.tar.gz
git push

#('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')

# --------------------------------
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0340 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01
# --------------------------------
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.1215 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01

# -------------------------------- Mono Left
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.340  image:=/right/camera/image_raw camera:=/right/camera/
# -------------------------------- Mono Right
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.340  image:=/left/camera/image_raw camera:=/left/camera/
# -------------------------------- Mono Drone (bottom)
# Clipboard 
rosrun camera_calibration cameracalibrator.py --size 4x3 --square 0.0525 image:=/ardrone/bottom/image_raw camera:=/ardrone/bottom

# Gatorboard:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 image:=/ardrone/bottom/image_raw camera:=/ardrone/bottom

# --square 0.108 is the size of the squares in meters (the tutorial uses 108 mm boxes), my board is 15/16" squares (15/16" = 23.8125 mm ~ .024m)  NOTE: Checkerboard size refers to the number of internal corner, as described in the OpenCV documentation (i.e. the 8x6 checkerboard contains 9x7 squares)
# --approximate=0.01 This allows 0.01s time difference between image pairs
# --no-service-check : This suppresses the service check for the driver
# Use these commands after roslaunch camera1394_stereo.launch



# rosrun
# camera_calibration
# cameracalibrator.py
# --size 8x6
# --square 0.108
# --approximate=0.01
# right:=/stereo/right/image_raw
# left:=/stereo/left/image_raw
# right_camera:=/stereo/right
# left_camera:=/stereo/left


# copy to mac from turtlebot
scp /home/$USER/ros/src/hast/cam_info/kobukistereo20161118.yaml benjamin@jupiter.local:/Users/benjamin/git/hast/ros/turtlebot/cam_info/


# -------------------------------- Stereo Blackflies
# Gatorboard:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/right/image_raw left:=/left/image_raw right_camera:=/right/ left_camera:=/left/ --approximate=0.01
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.111 right:=/right/image_raw left:=/left/image_raw right_camera:=/right/ left_camera:=/left/ --approximate=0.005
# Clipboard:
rosrun camera_calibration cameracalibrator.py --size 3x4 --square 0.052 right:=/right/image_raw left:=/left/image_raw right_camera:=/right/ left_camera:=/left/ --approximate=0.01
rosrun camera_calibration cameracalibrator.py --size 3x4 --square 0.0505 right:=/right/image_raw left:=/left/image_raw right_camera:=/right /left_camera:=/left/ --approximate=0.01


# -------------------------------- Stereo chameleons
# Clipboard:
rosrun camera_calibration cameracalibrator.py --size 5x4 --square 0.052 right:=/right/camera/image_raw left:=/left/camera/image_raw right_camera:=/right/camera left_camera:=/left/camera --approximate=0.01
