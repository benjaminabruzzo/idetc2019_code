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
		ros::init(argc, argv, "HastKill");
		ros::NodeHandle n;
			/*-----  Publishers and Subscribers */
			ros::Publisher HastKill		= n.advertise<hast::flag>("/hast/shutdown", 1000);
			ros::Publisher DroneLand	= n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
			ros::Publisher DroneCmd_dr_pub 	= n.advertise<geometry_msgs::Twist>	("/hast/uav/cmd_vel", 1000);

			DroneCmd_dr_msg.linear.x = 0.0;
			DroneCmd_dr_msg.linear.y = 0.0;
			DroneCmd_dr_msg.linear.z = 0.0;
			DroneCmd_dr_msg.angular.z = 0.0;
			DroneCmd_dr_msg.angular.x = 0.0;
			DroneCmd_dr_msg.angular.y = 0.0;
		while (ros::ok())
		{
			ROS_INFO("Ready to kill Hast Nodes.\n");
			Kill_msg.flag = true;
			std::cin.ignore();
			ROS_INFO("Killing Hast Nodes...");
			ROS_INFO("Stopping Drone...");
			ROS_INFO("Landing Drone...");
			for(int b = 1; b < 15; ++b)
			{
				HastKill.publish(Kill_msg);
				DroneCmd_dr_pub.publish(DroneCmd_dr_msg);
				DroneLand.publish(Null_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}

			ROS_INFO("Resetting Hast Killswitch...");
			for(int b = 1; b < 15; ++b)
			{
				Kill_msg.flag = false;
				HastKill.publish(Kill_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}
			ROS_INFO("...done\n");
			ros::shutdown();
		}

		return 0;
	}
