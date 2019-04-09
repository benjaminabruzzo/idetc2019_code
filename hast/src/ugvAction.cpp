#include "genheaders.hpp"

// Move library
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
		

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class moveBaseActionClass
{
	private:		
		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;

		MoveBaseClient ac;

		ros::Subscriber ugvGoal_sub, HastShutDown_sub;
		std::string s_ugv_goaltopic, s_map, s_kill_topic;

	public:
		moveBaseActionClass(std::string s_move_base_namespace) : ac(s_move_base_namespace, true)
		{
			s_map = "/map";
			ros::param::get("~map", s_map);
			ros::param::get("~ugv_goaltopic", s_ugv_goaltopic);
			ros::param::get("~kill_topic", s_kill_topic);

			ugvGoal_sub 		= n.subscribe(s_ugv_goaltopic, 	1, &moveBaseActionClass::setGoal, 		this);
			HastShutDown_sub 	= n.subscribe(s_kill_topic,		1, &moveBaseActionClass::nodeShutDown, 	this);

			ROS_INFO("ugvAction:: Waiting for action server to start.");
			ac.waitForServer();
			ROS_INFO("ugvAction:: Action server started, waiting for goal.");
		}

		void setGoal(const geometry_msgs::Pose2D::ConstPtr& goalPose)
		{
		    move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = s_map;
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = goalPose-> x;
			goal.target_pose.pose.position.y = goalPose-> y;
			goal.target_pose.pose.position.z = 0.0;

			goal.target_pose.pose.orientation.x = 0.0;
			goal.target_pose.pose.orientation.y = 0.0;
			goal.target_pose.pose.orientation.z = sin((goalPose-> theta)/2);
			goal.target_pose.pose.orientation.w = cos((goalPose-> theta)/2);

			// ROS_INFO("Sending goal: Frame:map, Position(3.304, 0.635, 0.000), Orientation(0.000, 0.000, 0.986, -0.166) = Angle: -2.809");
			ROS_INFO("Sending goal: Frame:%s, Position(%6.3f, %6.3f, 0.000), Orientation(0.000, 0.000, %6.3f, %6.3f) = Angle: %6.3f",
				s_map.c_str(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
				goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w, goalPose-> theta);

			ac.sendGoal(goal);
		}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		// ROS_WARN("ckfRecorder: nodeShutDown");
		if (ShutDown->flag)
		{
			// aprilAlloc();
			// ugv.openAllocFile()
			ROS_INFO("ckfRecorder: Shutdown requested..");
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "hastUGVAction");

	std::string s_move_base_namespace;
	ros::param::get("~s_move_base_namespace", s_move_base_namespace);
	moveBaseActionClass actionClient(s_move_base_namespace);
	ros::spin();
	return 0;
}


// http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClient