#ifndef HAST_UAV_H
#define HAST_UAV_H

#include "genheaders.hpp"
#include "ckfClass.hpp"
#include "utils.hpp"
#include "fileops.hpp"


class hastUAV
{
	private:
		// ROS comms
		// ros::NodeHandle uav_n;

		/* -------- Clock Times */
		ros::Time navStamp;
		uint navHeadSeq;
		double navTS, navTSpublished, navdt; //, KFdt;

		double Pi;
		int L2Norm;

	public:
		//ROS comms
		ros::Publisher state_pub, pose_pub; // uav publisher
		double yaw_drift_rate, yaw_drift;
		ros::Subscriber cmdVel_sub, navData_sub;
		hast::uavstate state_msg; // uav publisher
		uint stateMsg_id, pose_msg_seq; // pose msg counter
		uint navDataCounter, estCounter;


		cv::Mat EstPosition_lo, EstPositionHom_lo;
		cv::Mat EstPosition_gl, EstPositionHom_g;
		cv::Mat EstPosition_ugv, correction_ugv; 

		cv::Mat MeasuredVel_lo, MeasuredVel_gl;
		cv::Mat MeasuredAcc_lo, MeasuredAcc_gl;
		double EstYaw_gl, EstYawBias, EstYaw_ugv, yawCorrection;
		double cosyaw, sinyaw;

		// tf variables
		tf::TransformBroadcaster TF_gl_pub;
		tf::Transform TF_gl;
		tf::Matrix3x3 R_TF_gl;
		tf::Quaternion Q_TF_gl;
		std::string s_TF_gl_parent, s_TF_gl_child;

		// cmd_vel variables
		uint cmdcount;
		cv::Mat cmdTwist;
		double velStamp;

		// Inertial Measurements
		uint flightState;
		double echoAlt, echoAltLast, deltaAlt;

		// compass drift rate...
		double compassYaw, compassYawLast, compassYawDelta;
		bool liftoffSwitch;
		double compassYawFirst; // used to set the first value to zero, and all later ref to that
		std::vector<double> compassReadings;
		uint compassCounter;


		cv::Mat Rgl2lo, Rgl2lo4x4;
		cv::Mat Hlo2gl, Hgl2lo;
		cv::Mat H_cam2uav; 
		// H_cam2uav = makeRz(-(90)*Pi/180)*makeRx(-(90-Wedge)*Pi/180);

		// create complementary filter for uav
		ckfClass ckf;

		fileops mfile;

	
		bool ckf_sim;
		bool observedByStereo;
		bool positionFromTags;

		std::FILE * mFile;
		std::string s_filename;

	//Localization functions

		hastUAV(); // void for construction of class
		void openmFile();
		void readCmdVel(const geometry_msgs::Twist::ConstPtr& vsub);
		void inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata);
		bool uavSwitchTags(hast::flipflop::Request &req, hast::flipflop::Response &res);
		void tfPublisher(ros::Time pubstamp);
		void posePublisher(double pubtime);
		void poseHeaderPublisher(ros::Time pubstamp);
};

#endif