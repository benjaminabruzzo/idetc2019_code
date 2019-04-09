#include "genheaders.hpp"

// <param name="goal_x" 		value="$(arg goal_x)"/>
// <param name="goal_y" 		value="$(arg goal_y)"/>
// <param name="goal_theta" 	value="$(arg goal_theta)"/>
// <param name="goal_topic" 	value="$(arg goal_topic)"/>

geometry_msgs::Pose2D goalPose_msg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "testPub");
	ros::NodeHandle n;

	std::string s_goal_topic;

	// hastSLAM testSLAM;

	/*-----  Publishers and Subscribers */
	ros::param::get("~goal_topic",s_goal_topic); // frame for uav detecting april tags
	ros::param::get("~goal_x",goalPose_msg.x );
	ros::param::get("~goal_y",goalPose_msg.y );
	ros::param::get("~goal_theta",goalPose_msg.theta);

	ros::Publisher posePublisher	= n.advertise<geometry_msgs::Pose2D>(s_goal_topic.c_str(), 10);

	// goalPose_msg.x = 3.0;
	// goalPose_msg.y = 0.0;
	// goalPose_msg.theta = 0.01; // radians?

	ROS_INFO("Ready to test publisher.\n");

	std::cin.ignore();
	ROS_INFO("ugv goal : [%6.4f %6.4f : %6.4f]", goalPose_msg.x, goalPose_msg.y, goalPose_msg.theta );
	posePublisher.publish(goalPose_msg);
	ros::spinOnce();
	ros::Duration(0.1).sleep(); // sleep for 'x' second(s).

	return 0;
}
