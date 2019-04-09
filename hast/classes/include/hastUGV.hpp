#ifndef HAST_UGV_H
#define HAST_UGV_H

#include "genheaders.hpp"
#include "ckfClass.hpp"

class hastUGV
{
	private:
		// Constants which might be useful in the future
		double Pi;
		int L2Norm;
	public:
		std::FILE * mFile;
		std::string s_filename;

		// ROS comms
		ros::Publisher state_pub; // ugv publisher
		ros::Subscriber WheelOdom_sub; //, cmdVel_sub;
		// std::string s_ugvCmd_topic;

		hast::ugvstate state_msg;
		uint stateMsg_id; // pose msg counter

		// time and Counters
		double velStamp, velStamplast;
		uint velCounter, estCounter;

		cv::Mat EstPosition_gl, EstPosition_ugv, correction_gl;
		cv::Mat MeasuredVel_lo, MeasuredVel_gl;
		double EstYaw_gl, cosyaw, sinyaw, yawCorrection;
		cv::Mat Rgl2lo, Rgl2lo4x4;
		cv::Mat Hlo2gl, Hgl2lo;

		// create complementary filter for ugv
		ckfClass ckf;

		/*-----  Wheel Odometry */
		std::string s_wheel_odom_topic, s_base_footprint_topic;
		uint wheelcount;

		ros::Time wheelTimeStamp;
		double wheelTime, lastWheelTime, wheel_dt;

		double wheel_yaw_rate, wheelyaw_q, wheeldeltayaw_q;
		cv::Mat wheel_twist_linear, wheel_twist_angular;
		
		double vel_dt_aggregator;
		bool ckf_sim;


	// Functions
		hastUGV();
		void openmFile();
		void posePublisher(double timestamp);
		cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in);
		cv::Mat invertH(cv::Mat H);
		void WheelOdometry(const nav_msgs::Odometry::ConstPtr& osub);

};

#endif