# Hast Package - first time setup
	mkdir ~/ros/data
	mkdir ~/ros/src/metahast && cd ~/ros/src/metahast
## go to all git servers and add new ssh key
	git init && git remote add bb git@bitbucket.org:ags_robotics/metahast.git
	git pull bb master 
	mkdir -p ~/.ros/camera_info
	. ~/ros/src/metahast/hast/cam_info/copyyamls.sh 
	catkin_remake
	echo 'roscd metahast && cd ..' >> ~/.bashrc

# Launching an experiment
## 1) start the uav process:
	roslaunch hast ardrone.launch

## 2) launch the UGV nodes:
	roslaunch hast kobuki_base_n.launch

## 3) launch the recorder and KF files with dates
	roslaunch hast hast_n.launch date:=20180418 trial:=001
### 3a) flags:
	saveimages:=true 	:: saves images
	showcircles:=true 	:: shows left and right images with "found" circles
	<arg name="sim" 		default="false" />
	<arg name="april_debug" default="false" />
	<arg name="trial" 		default="000" />
	<arg name="date" 		default="20160930" />
	<arg name="user" 		default="$(env USER)" />
	<arg name="saveimages" 	default="false" />
	<arg name="showcircles" default="false" />
	<arg name="saveraw" 	default="false" />
	<arg name="trigger_saveraw"	default="false" />  <!-- rosrun hast triggerSaveRaw -->
	<arg name="tb_base" 	default="kobuki" />
	<arg name="down_image"  default="image_rect" />
	<arg name="tf_x_offset" default="0.025" />

## 4) launch the experiment
	roslaunch hast experiment.launch action:=true step:=true
### 4a) flags:
	abcde:=true 	:: UAV flys through waypoints A-E
	<arg name="arc" 		default="false" />
	<arg name="pull" 		default="false" />
	<arg name="picket" 		default="false" />
	<arg name="abcde" 		default="false" />
	<arg name="step" 		default="false" />
	<arg name="ugvfore" 	default="false" />
	<arg name="ugvauto" 	default="false" />
	<arg name="iflift" 		default="false" />
	<arg name="aprillog" 	default="false" />
	<arg name="ugvcomplex" 	default="false" />
	<arg name="action" 		default="false" />

## 5) TRENDNET_CREATE::turtlebot_create



