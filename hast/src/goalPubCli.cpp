#include "genheaders.hpp"

// Move library
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class goalPublisher
{
	private:

	public:
		ros::NodeHandle n;

		ros::Subscriber HastShutDown_sub;
		hast::flag Kill_msg;

		ros::ServiceClient GoalTagLocation_cli; // client to request goal tag location
			hast::ugvgoal goal_call;
		ros::Publisher UGVposePublisher;
			geometry_msgs::Pose2D UGVgoalPose_msg; //pose message for uav goal location
			geometry_msgs::PoseStamped UGVgoalPoseStamped_msg; //pose message for uav goal location
			int goal_id;


// rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

	goalPublisher()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &goalPublisher::nodeShutDown, this);
		UGVposePublisher	= n.advertise<geometry_msgs::Pose2D>("/hast/ugv/goal", 10);
		GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>("/hast/service/ugv/goal", true);

		ros::param::get("~april_goal_id", goal_id);


		ROS_INFO("goalPublisher created.");
		spinsleep(2);
		experimentAction();
	}

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{ros::shutdown();}
	}

	void experimentAction()
	{
		ROS_INFO("goal_call.request.tagID = %i;", goal_id);
		goal_call.request.tagID = goal_id;
		ROS_INFO("GoalTagLocation_cli.call(goal_call);");
		GoalTagLocation_cli.call(goal_call);
		spinsleep(8);

		if (goal_call.response.goalInMap) 
		{
			ROS_INFO("uav found goal tag, setting location as goal for ugv \n x: %6.8f \n y: %6.8f \n theta: %6.8f", 
				goal_call.response.goalP.x, goal_call.response.goalP.y, goal_call.response.goalYaw);
			UGVgoalPose_msg.x = goal_call.response.goalP.x;
			UGVgoalPose_msg.y = goal_call.response.goalP.y;
			UGVgoalPose_msg.theta = goal_call.response.goalYaw; // set the ugv goal yaw to match tag yaw, for some reason
			UGVposePublisher.publish(UGVgoalPose_msg);
			// ROS_INFO("// UGVposePublisher.publish(UGVgoalPose_msg); (suppressed)");
		}
	}

	void spinsleep(int dwell_seconds)
	{
		for(int b = 0; b < dwell_seconds; ++b)
		{
			ros::spinOnce();
			ros::Duration(1).sleep(); // sleep for 'x' second(s).
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goalPublisher");
	goalPublisher gP;
	ros::spin();
	return 0;
}


