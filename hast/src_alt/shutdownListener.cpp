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

class shutdownListener
{
	private:
		ros::Subscriber HastShutDown_sub;
		hast::flag Kill_msg;
		std_msgs::Empty Null_msg;
		geometry_msgs::Twist DroneCmd_dr_msg;

	public:
		ros::NodeHandle n;

	shutdownListener()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &shutdownListener::nodeShutDown, this);
		ROS_INFO("Shutdown Listener Constructed");
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{ros::shutdown();}
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugvDrive");
	shutdownListener tC;
	ros::spin();
	return 0;
}
