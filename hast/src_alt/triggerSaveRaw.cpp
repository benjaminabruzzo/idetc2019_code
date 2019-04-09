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
	#include <geometry_msgs/Twist.h>
	#include <nav_msgs/Odometry.h>
	#include <tf/transform_listener.h>
	#include <hast/flag.h>
	// #include <hast/markers.h>
	#include <hast/matrix3x3.h>
	// #include <hast/navdata.h>
	#include <hast/pixels.h>
	#include <hast/uavstate.h>
	#include <hast/ugvstate.h>
	#include <hast/vector3.h>
	#include <hast/vector4.h>

	// Services
	#include <hast/flipflop.h>
	#include <hast/stereoodomcs.h>
	#include <hast/uavcontrolstate.h>
	#include <hast/uavnavstate.h>
	#include <hast/ugvautopilot.h>
	#include <hast/ugvdrive.h>


	hast::flag Kill_msg;
	std_msgs::Empty Null_msg;
	geometry_msgs::Twist DroneCmd_dr_msg;

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "triggerRaw");
		ros::NodeHandle n;
			/*-----  Publishers and Subscribers */
			ros::Publisher SaveTrigger_pub 	= n.advertise<std_msgs::Empty>("/pgrstereo/saveTrigger", 1000);

		while (ros::ok())
		{
			ROS_INFO("Ready to capture image.. \n");
			std::cin.ignore();

			SaveTrigger_pub.publish(Null_msg);
			ros::spinOnce();

			ROS_INFO("Resetting camera switch...");

			ros::Duration(1).sleep(); // sleep for 'x' second(s).
			
		}
		ROS_INFO("...done\n");
		ros::shutdown();

		return 0;
	}
