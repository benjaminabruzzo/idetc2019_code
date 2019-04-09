// Programming tools
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>

//Vision tools
// #include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <camera1394/SetCameraRegisters.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <ardrone_autonomy/Navdata.h>
	#include <std_msgs/Empty.h>
	#include <geometry_msgs/Pose2D.h>
	#include <geometry_msgs/PoseStamped.h>
	#include <geometry_msgs/PoseWithCovariance.h>
	#include <geometry_msgs/PointStamped.h>
	#include <geometry_msgs/Twist.h>
	#include <nav_msgs/Odometry.h>
	#include <nav_msgs/Path.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>
	#include <hast/flag.h>
	#include <hast/matrix3x3.h>
	#include <hast/pixels.h>
	#include <hast/posewithheader.h>
	#include <hast/uavstate.h>
	#include <hast/ugvstate.h>
	#include <hast/vector3.h>
	#include <hast/vector4.h>

	// Services
	#include <hast/null.h>
	#include <hast/flipflop.h>
	#include <hast/slamswitch.h>
	#include <hast/stereoodomcs.h>
	#include <hast/uavcontrolstate.h>
	#include <hast/uavpicket.h>
	#include <hast/uavnavstate.h>
	#include <hast/ugvautopilot.h>
	#include <hast/ugvdrive.h>
	#include <hast/ugvgoal.h>

	//pcl library
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>

