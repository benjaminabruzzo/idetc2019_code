#include "genheaders.hpp"

hast::flag Kill_msg;
std_msgs::Empty Null_msg;
geometry_msgs::Twist DroneCmd_dr_msg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HastKill");
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
