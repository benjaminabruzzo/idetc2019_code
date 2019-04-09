
# Launching simulated experiment
## 1) launch the simulator, world, and robot descriptions
	roslaunch hast_gazebo spawn_robots.launch </br>

	To load a block in the world:
	block:=true

	To load specific configurations from actual experiments:
	vicon_date:=yyyymmdd
	vicon_run:=###

## 2) launch the recorder and KF files with dates
	(roscd hast && . scripts/makedirs 20180415 1 5)
	roslaunch hast_gazebo hast.launch date:=20180415 run:=001
### 2a) flags:
	saveimages:=true 	:: saves images
	showcircles:=true 	:: shows left and right images with "found" circles

## 3) launch the experiment
	roslaunch hast experiment.launch 
### 3a) flags:
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
	<arg name="wait" 		default="15" />

