#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
#include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"
// these will probably be deleted 
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

namespace enc = sensor_msgs::image_encodings;

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";


pcl::PointCloud<pcl::PointXYZ>::Ptr aprilCloudXYZ_ptr (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr clearCloudXYZ_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr goalCloudXYZ_ptr (new pcl::PointCloud<pcl::PointXYZ>);
class ckfRecorder
{
	private:
		// ROS_DEBUG flag
			bool debug_flag;

		/*---------  File Recorder ------------- */
			fileops mfile;
			std::string s_aprilMfile, s_april_prealloc, s_oneckfMfile;
			std::string s_trial, s_dotm, s_root, s_date, s_user;
		
			std::string s_markerid, s_uav_baseframe;
			std::FILE * aprilMfile;
			std::FILE * aprilAllocfile;
			std::FILE * oneckfMfile;
			double Pi;
			int L2Norm;


		/*---------  Marker Definintions ------------- */
		// std::vector<aprilDKFclass> april;
			// apriltags_ros::AprilTagDetectionArray tagArray; //marker array as detected by image

			std::vector<apriltagclass> april;
				tf::TransformListener listener;
				int printlength;
				char markerChannel[20];
				cv::Vec<double,3> markerObs;
				// uint TotalMarkers;// = 12; // I am using #00 through #11 == 12 with zero-indexing
			std::vector<apriltags_ros::AprilTagDetection> tagArray; //marker array as detected by image
				ros::Time tagStamp;
				double tagTSpublished;
				uint tagArraySize;
				int tag_max_id;
				// std::string tag_max_id;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
				ros::Subscriber stereoPose_sub, HastShutDown_sub, TagDetections_sub;
				bool april_debug;

				ros::Subscriber slamState_sub;
					std::string s_slamState_topic;
				// ros::Publisher markerSacn_pub; // marker publisher?

				ros::Publisher aprilCloud2_pub, clearCloud2_pub, goalCloud2_pub; 
					sensor_msgs::PointCloud2 aprilCloud2_msg, clearCloud2_msg, goalCloud2_msg;
					pcl::PointXYZ aprilCenter_XYZ; // pcl point for center of april tag
			/*----- Client Services */
			ros::ServiceServer StateService_ser;
			ros::ServiceServer GoalTagLocation_ser;
			ros::ServiceServer positionFromTags_ser; // service to switch uav position from observing to deriving position from april tags
				int lastgoal;
				// geometry_msgs::PointStamped goalPoint;
			ros::ServiceClient clearCostmaps_cli; // client to request goal tag location
				hast::null nullcall;

		/*----- image transport Channels */
			image_transport::ImageTransport it;

		/* COMBINED CKF *************																	******************  COMBINED CKF */
			hastUAV uav; // create uav from class
			hastUGV ugv; // create ugv from class
			ckfClass oneCKF; // create ckf for uav and ugv

		/*----- Stereo Variables */
			// bool stereoObs_odomBit;
			// bool stereoObs_odomMODE;
			double stereoObs_stamp, stereoObs_yaw_ugv2uav;
			cv::Mat stereoObs_P, stereoObs_P_global, stereoObs_PCov;
			double stereoObs_yaw_cov;
			uint stereoObs_counter;
			cv::Mat Bluepix, Greenpix, Redpix; // pixel vectors [rX rY lX lY]

			bool ckf_sim;

			std::string s_Pose_sub, s_ugvn_state, s_obstacle_cloud_topic, s_goal_cloud_topic;
			std::string s_tagDetections;
	public:

		int framecount; // increment counter for dropping frames
		int dropnframes; // constant to drop n frames
		int debugcounter; //dummy counter for debugging
		int downimagecountup;

	ckfRecorder()
	: it(n)
	{
		/*--------- Math Constants ------------- */
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function
		downimagecountup = 0; // number of images to ignore before starting slam
		// downimagecountup = 10; // number of images to ignore before starting slam
		debug_flag = false;

		/*---------  File Recorder Initilizer ------------- */
		if(ros::param::get("~user",s_user)){} else {s_user = "";}
		if(ros::param::get("~date",s_date)){} else {s_date = "";}
		if(ros::param::get("~trial",s_trial)){} else {s_trial = "";}

		if (n.getParam("/hast/kf/sim", ckf_sim)) {} else {ckf_sim = false;}
		uav.ckf_sim = ckf_sim; ugv.ckf_sim = ckf_sim;

		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		ros::Duration(0.5).sleep();

		uav.s_filename = s_root + s_date + "/" + s_trial + "/uavRecorder_" + s_trial + s_dotm;
		uav.openmFile();

		ugv.s_filename = s_root + s_date + "/" + s_trial + "/ugvRecorder_" + s_trial + s_dotm;
		ugv.openmFile();

		s_aprilMfile = s_root + s_date + "/" + s_trial + "/april_"  + s_trial + s_dotm;
		ROS_INFO("ckfRecorder: %s", s_aprilMfile.c_str());
		aprilMfile = std::fopen (s_aprilMfile.c_str(), "w");
		fprintf (aprilMfile, "%% %s\n", s_aprilMfile.c_str());
		fprintf (aprilMfile, "%%clc; \n%%clear all;\n%%close all;\n\n");

		s_oneckfMfile = s_root + s_date + "/" + s_trial + "/ckfRecorder_"  + s_trial + s_dotm;
		ROS_INFO("ckfRecorder: %s", s_oneckfMfile.c_str());
		oneckfMfile = std::fopen (s_oneckfMfile.c_str(), "w");
		fprintf (oneckfMfile, "%% %s\n", s_oneckfMfile.c_str());
		fprintf (oneckfMfile, "%%clc; \n%%clear all;\n%%close all;\n\n");


		ROS_INFO("ckfRecorder: .m files opened");

		/*--------- Initialize ROS Communication & Variables ------------- */
		ros::param::get("~uav_baseframe",s_uav_baseframe); // frame for uav detecting april tags
		ros::param::get("~ckf_TF_parent",uav.s_TF_gl_parent);
		ros::param::get("~ckf_TF_child",uav.s_TF_gl_child);
		ros::param::get("~odom_topic", ugv.s_wheel_odom_topic);
		ros::param::get("~footprint_topic",ugv.s_base_footprint_topic);
			ROS_INFO("ckfRecorder::hastUGV: %s", ugv.s_wheel_odom_topic.c_str());
		
		ros::param::get("~uav_imu_yawdriftrate",uav.yaw_drift_rate);
			ROS_INFO("ckfRecorder::yaw_drift_rate: %4.4f", uav.yaw_drift_rate);
		
		ros::param::get("~tag_max_id", tag_max_id);
			ROS_INFO("ckfRecorder::tag_max_id: %i", tag_max_id);
		
		/*-----  Publishers and Subscribers */
		// if(ros::param::get("~uav_slam_state",s_slamState_topic)){} else {s_slamState_topic = "/hast/slamstate";}
		if(ros::param::get("~uav_slam_state",s_slamState_topic)){} else {s_slamState_topic = "/hast/slamstate";}

		if(ros::param::get("~tag_detection_topic",s_tagDetections)){} else {s_tagDetections = "tag_detections";}
		if(ros::param::get("~Pose_sub",s_Pose_sub)){} else {s_Pose_sub = "/BEEP/stereo/pose";}
		if(n.getParam("/hast/april/debug", april_debug)){} else {april_debug = false;}
		
		HastShutDown_sub 	= n.subscribe("/hast/shutdown",	1, &ckfRecorder::nodeShutDown, this);
		stereoPose_sub 		= n.subscribe(s_Pose_sub, 1, &ckfRecorder::stereoRead , this);
		TagDetections_sub 	= n.subscribe(s_tagDetections, 1, &ckfRecorder::tagDetections , this);
				
		if(ros::param::get("~obstacle_cloud_topic",s_obstacle_cloud_topic)){} else {s_obstacle_cloud_topic = "/hast/april/PointCloud2";}
		if(ros::param::get("~goal_cloud_topic",s_goal_cloud_topic)){} else {s_goal_cloud_topic = "/hast/goal/PointCloud2";}

		aprilCloud2_pub = n.advertise<sensor_msgs::PointCloud2>(s_obstacle_cloud_topic, 1);
		goalCloud2_pub = n.advertise<sensor_msgs::PointCloud2>(s_goal_cloud_topic, 1);

		uav.navData_sub = n.subscribe("/ardrone/navdata",  5, &hastUAV::inertialUpdate, &uav);
		uav.cmdVel_sub  = n.subscribe("/ardrone/cmd_vel",  5, &hastUAV::readCmdVel, &uav);
		uav.state_pub   = n.advertise<hast::uavstate>("/hast/uav/state", 1);
		uav.pose_pub    = n.advertise<hast::posewithheader>("/hast/uav/pose", 1);

		ugv.WheelOdom_sub 	= n.subscribe(ugv.s_wheel_odom_topic,	5, &hastUGV::WheelOdometry, &ugv);	


		if(ros::param::get("~ugvn_state",s_ugvn_state)){} else {s_ugvn_state = "/BEEP/ugv/state";}	
		ugv.state_pub = n.advertise<hast::ugvstate>(s_ugvn_state, 1);
		ugv.posePublisher(ros::Time::now().toSec());

		/*-----  Services and Clients */
		positionFromTags_ser = n.advertiseService("/hast/service/uav/posFromTags", &hastUAV::uavSwitchTags , &uav);
		StateService_ser  	= n.advertiseService("/hast/service/uav/state", &ckfRecorder::serveState , this);
		GoalTagLocation_ser = n.advertiseService("/hast/service/ugv/goal", &ckfRecorder::serveGoalLocation , this);
			lastgoal = 1000;
		// clearCostmaps_cli = n.serviceClient<hast::null>("/move_base/clear_costmaps", true);

		ROS_INFO("ckfRecorder::publishers and subscribers constructed");

		/*----- dual-purpose stereo message variables */
		// stereoObs_odomMODE = true;
		// stereoObs_odomBit = true;
		stereoObs_stamp = 0;
		stereoObs_counter = 0;
		stereoObs_yaw_ugv2uav = 0;
		stereoObs_P = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
		dropnframes = 0;
		debugcounter = 0;

		// oneCKF init
			oneCKF.counter = 0;
			oneCKF.Mech = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.Aiding = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.PosteriorEst = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
			oneCKF.Fk = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.PosteriorCov = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
			oneCKF.Qw = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
			oneCKF.H = (cv::Mat_<double>(4, 5) <<
											1,0,0,0,0,
											0,1,0,0,0,
											0,0,1,0,0, 
											0,0,0,1,1); // !!! 4x5 measurement matrix
			oneCKF.Rk = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);// !!! 4x4 stereo covariance matrix
			oneCKF.I = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);

		april.reserve( tag_max_id );
		for(uint i = 0; i != tag_max_id; i++) //init april tags
		{
			apriltagclass obj;
			// aprilDKFclass obj;

				obj.id = i;
				if (i==7){obj.isgoal = true;}else{obj.isgoal=false;}

			april.push_back(obj);

			s_markerid = "tag_" + patch::to_string(i);
			fprintf(aprilMfile, "%% --------------- %s: --------------- %% \n", s_markerid.c_str());
		}
	}

	void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)
	{//ROS_INFO("void stereoRead(const hast::uavstate::ConstPtr& stereo_state_msg)");
		// New stereo observation
		// stereoObs_odomBit = stereo_state_msg -> odomBIT;
		// uav.state_msg.odomBIT = stereo_state_msg -> odomBIT;
		// stereoObs_odomMODE = stereo_state_msg -> odomMODE;
		stereoObs_stamp = stereo_state_msg -> stamp;
		stereoObs_P = (cv::Mat_<double>(4, 1) << stereo_state_msg->P.x, stereo_state_msg->P.y, stereo_state_msg->P.z, 1.0);
		stereoObs_PCov = (cv::Mat_<double>(3, 3) <<
						 stereo_state_msg->PCov.row0.x, stereo_state_msg->PCov.row0.y, stereo_state_msg->PCov.row0.z,
						 stereo_state_msg->PCov.row1.x, stereo_state_msg->PCov.row1.y, stereo_state_msg->PCov.row1.z,
						 stereo_state_msg->PCov.row2.x, stereo_state_msg->PCov.row2.y, stereo_state_msg->PCov.row2.z);

		stereoObs_yaw_ugv2uav = stereo_state_msg -> yaw; // uav heading angle in degrees in ugv frame
		stereoObs_yaw_cov = stereo_state_msg -> yaw_cov; // variance of heading angle (in radians)

		stereoObs_P_global = ugv.Hlo2gl * stereoObs_P;

		Redpix = (cv::Mat_<double>(4, 1) << stereo_state_msg->red.xr, stereo_state_msg->red.yr, stereo_state_msg->red.xl, stereo_state_msg->red.yl);
		Bluepix = (cv::Mat_<double>(4, 1) << stereo_state_msg->blue.xr, stereo_state_msg->blue.yr, stereo_state_msg->blue.xl, stereo_state_msg->blue.yl);
		Greenpix = (cv::Mat_<double>(4, 1) << stereo_state_msg->green.xr, stereo_state_msg->green.yr, stereo_state_msg->green.xl, stereo_state_msg->green.yl);

		fprintf (ugv.mFile, "ugvRecorder.stereo.time(%d,1)  = % -6.14f;\n", ++stereoObs_counter, stereoObs_stamp);
		fprintf (ugv.mFile, "ugvRecorder.stereo.yaw_ugv(%d,1)  = % -6.14f;\n", stereoObs_counter, stereoObs_yaw_ugv2uav);
		fprintf (ugv.mFile, "ugvRecorder.stereo.yaw_cov(%d,1)  = % -6.14f;\n", stereoObs_counter, stereoObs_yaw_cov);
		fprintf (ugv.mFile, "ugvRecorder.stereo.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", stereoObs_counter, stereoObs_P_global.at<double>(0,0), stereoObs_P_global.at<double>(1,0), stereoObs_P_global.at<double>(2,0));
		fprintf (ugv.mFile, "ugvRecorder.stereo.Position_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", stereoObs_counter, stereoObs_P.at<double>(0,0), stereoObs_P.at<double>(1,0), stereoObs_P.at<double>(2,0));

		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n", stereoObs_counter, stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2));
		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n", stereoObs_counter, stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2));
		fprintf (ugv.mFile, "ugvRecorder.stereo.PCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f];\n\n", stereoObs_counter, stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2));

		update_oneCKF(stereoObs_stamp);
		// uav.posePublisher(stereoObs_stamp);
		// ugv.posePublisher(stereoObs_stamp); // only use this if the base is being simulated.  check if the base is powered on.
	}


	void update_oneCKF(double stereotime)
	{//ROS_INFO("void update_oneCKF(double stereotime) open");
		double dt = stereotime - oneCKF.lastCKFtime;
		bool Qdk_flag;
		if (dt > 10)
			{dt = 0.05;} // Ignore first dt or any long duration gaps in ckf updates
			else {oneCKF.lastCKFtime = stereotime;} // reset 'last' time
			

		// convert uav and ugv poses to ugv-centric
		// estimate position of UAV in estimated UGV frame?
		uav.EstPosition_ugv = ugv.Hgl2lo * (cv::Mat_<double>(4, 1) << 
										uav.EstPosition_gl.at<double>(0,0), 
										uav.EstPosition_gl.at<double>(1,0), 
										uav.EstPosition_gl.at<double>(2,0),
										1); //homogeneous coordinates

		uav.EstYaw_ugv = uav.EstYaw_gl - ugv.EstYaw_gl;

		oneCKF.Mech = (cv::Mat_<double>(5, 1) <<
										uav.EstPosition_ugv.at<double>(0, 0),
										uav.EstPosition_ugv.at<double>(1, 0),
										uav.EstPosition_ugv.at<double>(2, 0),
										uav.EstYaw_ugv,
										0); // bias is not mechanized? No, it is. This would work if you used compass yaw above here and the bias estimate here
										// H has the sum of the bias and the yaw value coupled, which is how it gets estimated

		oneCKF.Aiding = (cv::Mat_<double>(5, 1) <<
										stereoObs_P.at<double>(0, 0),
										stereoObs_P.at<double>(1, 0),
										stereoObs_P.at<double>(2, 0),
										stereoObs_yaw_ugv2uav,
										0); // double bias_from_stereo = 0; //stereo cannot measure bias

		oneCKF.Rk = (cv::Mat_<double>(4, 4) <<
										stereoObs_PCov.at<double>(0, 0), stereoObs_PCov.at<double>(0, 1), stereoObs_PCov.at<double>(0, 2), 0,
										stereoObs_PCov.at<double>(1, 0), stereoObs_PCov.at<double>(1, 1), stereoObs_PCov.at<double>(1, 2), 0,
										stereoObs_PCov.at<double>(2, 0), stereoObs_PCov.at<double>(2, 1), stereoObs_PCov.at<double>(2, 2), 0,
										0, 0, 0, stereoObs_yaw_cov);// !!! 4x4 stereo covariance matrix

		// uavckf.Qdk should be rotated into ugv frame
		// oneCKF.Qdk = ugv.Rgl2lo4x4*uav.ckf.Qdk*ugv.Rgl2lo4x4.t() + ugv.ckf.Qdk;	


		oneCKF.Qdk = uav.ckf.Qdk + ugv.ckf.Qdk;	
		// how are these getting added?  They're different sizes? 
		// Actually, these are the same size. Who cares if the fifth state of the ugv Q matrix is always zero?

		// % Increment KF
		oneCKF.incrementKF();

		// This prevents divide by zero error ... 
		Qdk_flag = true;
		if ((oneCKF.Qdk.at<double>(0,0) < 0.00001) ||
			(oneCKF.Qdk.at<double>(1,1) < 0.00001) ||
			(oneCKF.Qdk.at<double>(2,2) < 0.00001) ||
			(oneCKF.Qdk.at<double>(3,3) < 0.00001))
			{ // if Qdk is too small, then neither vehicle moved ... It also helps at the very beginning of an experiment
				Qdk_flag = false;
				// oneCKF.Qdk.at<double>(0,0) =2;
				// oneCKF.Qdk.at<double>(1,1) =2;
				// oneCKF.Qdk.at<double>(2,2) =2;
				// oneCKF.Qdk.at<double>(3,3) =2;
			}
		// The estimated values are the previous esitmates plus the error adjustement
		// take the Qd fraction of the update and apply that to each vehicle?
		// ugv
		if(Qdk_flag)
		{
			ugv.yawCorrection = oneCKF.PosteriorEst.at<double>(3,0) * (ugv.ckf.Qdk.at<double>(3,3)/oneCKF.Qdk.at<double>(3,3));
		} else {
			ugv.yawCorrection = 0;
		}

		ugv.EstYaw_gl += ugv.yawCorrection;

		ugv.cosyaw = cos(Pi * ugv.EstYaw_gl / 180);
		ugv.sinyaw = sin(Pi * ugv.EstYaw_gl / 180);
		ugv.Rgl2lo = (cv::Mat_<double>(3, 3) <<
										 ugv.cosyaw, ugv.sinyaw, 0,
										-ugv.sinyaw, ugv.cosyaw, 0,
															0,0,1);
		ugv.Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
									   ugv.cosyaw, ugv.sinyaw, 0, 0,
									  -ugv.sinyaw, ugv.cosyaw, 0, 0,
									   0, 0, 1, 0,
									   0, 0, 0, 1);
		if(Qdk_flag)
		{
			ugv.correction_gl = ugv.Rgl2lo.t() * (cv::Mat_<double>(3, 1) <<
				oneCKF.PosteriorEst.at<double>(0,0)* (ugv.ckf.Qdk.at<double>(0,0)/oneCKF.Qdk.at<double>(0,0)),
				oneCKF.PosteriorEst.at<double>(1,0)* (ugv.ckf.Qdk.at<double>(1,1)/oneCKF.Qdk.at<double>(1,1)),
				oneCKF.PosteriorEst.at<double>(2,0)* (ugv.ckf.Qdk.at<double>(2,2)/oneCKF.Qdk.at<double>(2,2)));
		} else {
			ugv.correction_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		}
		
		ugv.EstPosition_gl += ugv.correction_gl;
		ugv.Hgl2lo = wrapH(ugv.Rgl2lo, ugv.EstPosition_gl);
		ugv.Hlo2gl = invertH(ugv.Hgl2lo);

		// uav
		if(Qdk_flag)
		{
			uav.correction_ugv = (cv::Mat_<double>(4, 1) <<
				oneCKF.PosteriorEst.at<double>(0,0)* (uav.ckf.Qdk.at<double>(0,0)/oneCKF.Qdk.at<double>(0,0)),
				oneCKF.PosteriorEst.at<double>(1,0)* (uav.ckf.Qdk.at<double>(1,1)/oneCKF.Qdk.at<double>(1,1)),
				oneCKF.PosteriorEst.at<double>(2,0)* (uav.ckf.Qdk.at<double>(2,2)/oneCKF.Qdk.at<double>(2,2)),
				0);
		} else {
			uav.correction_ugv = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
		}
		uav.EstPosition_ugv += uav.correction_ugv;
		uav.EstPositionHom_g = ugv.Hlo2gl * uav.EstPosition_ugv;
		uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << 
										uav.EstPositionHom_g.at<double>(0,0), 
										uav.EstPositionHom_g.at<double>(1,0), 
										uav.EstPositionHom_g.at<double>(2,0));
		if(Qdk_flag)
		{
			uav.yawCorrection = oneCKF.PosteriorEst.at<double>(3,0) * (uav.ckf.Qdk.at<double>(3,3)/oneCKF.Qdk.at<double>(3,3));
		} else {
			uav.yawCorrection = 0;
		}
		uav.EstYaw_gl += ugv.yawCorrection;
		
		// all of the bias goes to the uav * (uav.ckf.Qdk.at<double>(4,4)/oneCKF.Qdk.at<double>(4,4));
		uav.EstYawBias += oneCKF.PosteriorEst.at<double>(4,0);

		double cosyaw = cos(Pi * uav.EstYaw_gl / 180);
		double sinyaw = sin(Pi * uav.EstYaw_gl / 180);
		uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
								   cosyaw, sinyaw, 0,
								   -sinyaw, cosyaw, 0,
								   0, 0, 1);
		uav.Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
								   cosyaw, sinyaw, 0, 0,
								  -sinyaw, cosyaw, 0, 0,
								   0, 0, 1, 0,
								   0, 0, 0, 1);

		uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
		uav.Hlo2gl = invertH(uav.Hgl2lo);

		uav.observedByStereo = true;
		++oneCKF.counter;
		// write out data
			fprintf (oneckfMfile, "ckfRecorder.time(%u,:) = % -6.14f;\n", oneCKF.counter, stereotime);
			fprintf (oneckfMfile, "ckfRecorder.zk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.zk.at<double>(0,0), oneCKF.zk.at<double>(1,0), oneCKF.zk.at<double>(2,0), oneCKF.zk.at<double>(3,0), oneCKF.zk.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.yk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.yk.at<double>(0,0), oneCKF.yk.at<double>(1,0), oneCKF.yk.at<double>(2,0), oneCKF.yk.at<double>(3,0));
			fprintf (oneckfMfile, "ckfRecorder.Mech(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Mech.at<double>(0,0), oneCKF.Mech.at<double>(1,0), oneCKF.Mech.at<double>(2,0), oneCKF.Mech.at<double>(3,0), oneCKF.Mech.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.Kkyk(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Kkyk.at<double>(0,0), oneCKF.Kkyk.at<double>(1,0), oneCKF.Kkyk.at<double>(2,0), oneCKF.Kkyk.at<double>(3,0), oneCKF.Kkyk.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.Aiding(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.Aiding.at<double>(0,0), oneCKF.Aiding.at<double>(1,0), oneCKF.Aiding.at<double>(2,0), oneCKF.Aiding.at<double>(3,0),oneCKF.Aiding.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.PriorEst(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.PriorEst.at<double>(0,0), oneCKF.PriorEst.at<double>(1,0), oneCKF.PriorEst.at<double>(2,0), oneCKF.PriorEst.at<double>(3,0), oneCKF.PriorEst.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorEst(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorEst.at<double>(0,0), oneCKF.PosteriorEst.at<double>(1,0), oneCKF.PosteriorEst.at<double>(2,0), oneCKF.PosteriorEst.at<double>(3,0), oneCKF.PosteriorEst.at<double>(4,0));
			fprintf (oneckfMfile, "ckfRecorder.ugv.correction_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, ugv.correction_gl.at<double>(0,0), ugv.correction_gl.at<double>(1,0), ugv.correction_gl.at<double>(2,0));
			fprintf (oneckfMfile, "ckfRecorder.ugv.yawCorrection(%u,:) = % -6.14f ;\n", oneCKF.counter, ugv.yawCorrection);
			fprintf (oneckfMfile, "ckfRecorder.uav.correction_ugv(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", oneCKF.counter, uav.correction_ugv.at<double>(0,0), uav.correction_ugv.at<double>(1,0), uav.correction_ugv.at<double>(2,0), uav.correction_ugv.at<double>(3,0));
			fprintf (oneckfMfile, "ckfRecorder.uav.yawCorrection(%u,:) = % -6.14f ;\n", oneCKF.counter, uav.yawCorrection);

			fprintf (oneckfMfile, "ckfRecorder.PriorCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(0, 0), oneCKF.PriorCov.at<double>(0, 1), oneCKF.PriorCov.at<double>(0, 2), oneCKF.PriorCov.at<double>(0, 3), oneCKF.PriorCov.at<double>(0, 4));
			fprintf (oneckfMfile, "ckfRecorder.PriorCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(1, 0), oneCKF.PriorCov.at<double>(1, 1), oneCKF.PriorCov.at<double>(1, 2), oneCKF.PriorCov.at<double>(1, 3), oneCKF.PriorCov.at<double>(1, 4));
			fprintf (oneckfMfile, "ckfRecorder.PriorCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(2, 0), oneCKF.PriorCov.at<double>(2, 1), oneCKF.PriorCov.at<double>(2, 2), oneCKF.PriorCov.at<double>(2, 3), oneCKF.PriorCov.at<double>(2, 4));
			fprintf (oneckfMfile, "ckfRecorder.PriorCov(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(3, 0), oneCKF.PriorCov.at<double>(3, 1), oneCKF.PriorCov.at<double>(3, 2), oneCKF.PriorCov.at<double>(3, 3), oneCKF.PriorCov.at<double>(3, 4));
			fprintf (oneckfMfile, "ckfRecorder.PriorCov(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PriorCov.at<double>(4, 0), oneCKF.PriorCov.at<double>(4, 1), oneCKF.PriorCov.at<double>(4, 2), oneCKF.PriorCov.at<double>(4, 3), oneCKF.PriorCov.at<double>(4, 4));

			fprintf (oneckfMfile, "ckfRecorder.Rk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(0, 0), oneCKF.Rk.at<double>(0, 1), oneCKF.Rk.at<double>(0, 2), oneCKF.Rk.at<double>(0, 3));
			fprintf (oneckfMfile, "ckfRecorder.Rk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(1, 0), oneCKF.Rk.at<double>(1, 1), oneCKF.Rk.at<double>(1, 2), oneCKF.Rk.at<double>(1, 3));
			fprintf (oneckfMfile, "ckfRecorder.Rk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(2, 0), oneCKF.Rk.at<double>(2, 1), oneCKF.Rk.at<double>(2, 2), oneCKF.Rk.at<double>(2, 3));
			fprintf (oneckfMfile, "ckfRecorder.Rk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Rk.at<double>(3, 0), oneCKF.Rk.at<double>(3, 1), oneCKF.Rk.at<double>(3, 2), oneCKF.Rk.at<double>(3, 3));

			fprintf (oneckfMfile, "ckfRecorder.Sk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(0, 0), oneCKF.Sk.at<double>(0, 1), oneCKF.Sk.at<double>(0, 2), oneCKF.Sk.at<double>(0, 3));
			fprintf (oneckfMfile, "ckfRecorder.Sk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(1, 0), oneCKF.Sk.at<double>(1, 1), oneCKF.Sk.at<double>(1, 2), oneCKF.Sk.at<double>(1, 3));
			fprintf (oneckfMfile, "ckfRecorder.Sk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(2, 0), oneCKF.Sk.at<double>(2, 1), oneCKF.Sk.at<double>(2, 2), oneCKF.Sk.at<double>(2, 3));
			fprintf (oneckfMfile, "ckfRecorder.Sk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Sk.at<double>(3, 0), oneCKF.Sk.at<double>(3, 1), oneCKF.Sk.at<double>(3, 2), oneCKF.Sk.at<double>(3, 3));

			fprintf (oneckfMfile, "ckfRecorder.Skinv(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(0, 0), oneCKF.Skinv.at<double>(0, 1), oneCKF.Skinv.at<double>(0, 2), oneCKF.Skinv.at<double>(0, 3));
			fprintf (oneckfMfile, "ckfRecorder.Skinv(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(1, 0), oneCKF.Skinv.at<double>(1, 1), oneCKF.Skinv.at<double>(1, 2), oneCKF.Skinv.at<double>(1, 3));
			fprintf (oneckfMfile, "ckfRecorder.Skinv(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(2, 0), oneCKF.Skinv.at<double>(2, 1), oneCKF.Skinv.at<double>(2, 2), oneCKF.Skinv.at<double>(2, 3));
			fprintf (oneckfMfile, "ckfRecorder.Skinv(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Skinv.at<double>(3, 0), oneCKF.Skinv.at<double>(3, 1), oneCKF.Skinv.at<double>(3, 2), oneCKF.Skinv.at<double>(3, 3));

			fprintf (oneckfMfile, "ckfRecorder.Kk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(0, 0), oneCKF.Kk.at<double>(0, 1), oneCKF.Kk.at<double>(0, 2), oneCKF.Kk.at<double>(0, 3));
			fprintf (oneckfMfile, "ckfRecorder.Kk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(1, 0), oneCKF.Kk.at<double>(1, 1), oneCKF.Kk.at<double>(1, 2), oneCKF.Kk.at<double>(1, 3));
			fprintf (oneckfMfile, "ckfRecorder.Kk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(2, 0), oneCKF.Kk.at<double>(2, 1), oneCKF.Kk.at<double>(2, 2), oneCKF.Kk.at<double>(2, 3));
			fprintf (oneckfMfile, "ckfRecorder.Kk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(3, 0), oneCKF.Kk.at<double>(3, 1), oneCKF.Kk.at<double>(3, 2), oneCKF.Kk.at<double>(3, 3));
			fprintf (oneckfMfile, "ckfRecorder.Kk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Kk.at<double>(4, 0), oneCKF.Kk.at<double>(4, 1), oneCKF.Kk.at<double>(4, 2), oneCKF.Kk.at<double>(4, 3));

			fprintf (oneckfMfile, "ckfRecorder.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(0, 0), oneCKF.Qdk.at<double>(0, 1), oneCKF.Qdk.at<double>(0, 2), oneCKF.Qdk.at<double>(0, 3), oneCKF.Qdk.at<double>(0, 4));
			fprintf (oneckfMfile, "ckfRecorder.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(1, 0), oneCKF.Qdk.at<double>(1, 1), oneCKF.Qdk.at<double>(1, 2), oneCKF.Qdk.at<double>(1, 3), oneCKF.Qdk.at<double>(1, 4));
			fprintf (oneckfMfile, "ckfRecorder.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(2, 0), oneCKF.Qdk.at<double>(2, 1), oneCKF.Qdk.at<double>(2, 2), oneCKF.Qdk.at<double>(2, 3), oneCKF.Qdk.at<double>(2, 4));
			fprintf (oneckfMfile, "ckfRecorder.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(3, 0), oneCKF.Qdk.at<double>(3, 1), oneCKF.Qdk.at<double>(3, 2), oneCKF.Qdk.at<double>(3, 3), oneCKF.Qdk.at<double>(3, 4));
			fprintf (oneckfMfile, "ckfRecorder.Qdk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.Qdk.at<double>(4, 0), oneCKF.Qdk.at<double>(4, 1), oneCKF.Qdk.at<double>(4, 2), oneCKF.Qdk.at<double>(4, 3), oneCKF.Qdk.at<double>(4, 4));

			fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(0, 0), oneCKF.PosteriorCov.at<double>(0, 1), oneCKF.PosteriorCov.at<double>(0, 2), oneCKF.PosteriorCov.at<double>(0, 3), oneCKF.PosteriorCov.at<double>(0, 4));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(1, 0), oneCKF.PosteriorCov.at<double>(1, 1), oneCKF.PosteriorCov.at<double>(1, 2), oneCKF.PosteriorCov.at<double>(1, 3), oneCKF.PosteriorCov.at<double>(1, 4));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(2, 0), oneCKF.PosteriorCov.at<double>(2, 1), oneCKF.PosteriorCov.at<double>(2, 2), oneCKF.PosteriorCov.at<double>(2, 3), oneCKF.PosteriorCov.at<double>(2, 4));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(3, 0), oneCKF.PosteriorCov.at<double>(3, 1), oneCKF.PosteriorCov.at<double>(3, 2), oneCKF.PosteriorCov.at<double>(3, 3), oneCKF.PosteriorCov.at<double>(3, 4));
			fprintf (oneckfMfile, "ckfRecorder.PosteriorCov(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, oneCKF.PosteriorCov.at<double>(4, 0), oneCKF.PosteriorCov.at<double>(4, 1), oneCKF.PosteriorCov.at<double>(4, 2), oneCKF.PosteriorCov.at<double>(4, 3), oneCKF.PosteriorCov.at<double>(4, 4));

			fprintf (oneckfMfile, "ckfRecorder.uav.red.left.xy(%u,:)    = [% -6.14f % -6.14f];\n", oneCKF.counter, Redpix.at<double>(0,2), Redpix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.red.right.xy(%u,:)   = [% -6.14f % -6.14f];\n", oneCKF.counter, Redpix.at<double>(0,0), Redpix.at<double>(0,1));
			fprintf (oneckfMfile, "ckfRecorder.uav.blue.left.xy(%u,:)   = [% -6.14f % -6.14f];\n", oneCKF.counter, Bluepix.at<double>(0,2), Bluepix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.blue.right.xy(%u,:)  = [% -6.14f % -6.14f];\n", oneCKF.counter, Bluepix.at<double>(0,0), Bluepix.at<double>(0,1));
			fprintf (oneckfMfile, "ckfRecorder.uav.green.left.xy(%u,:)  = [% -6.14f % -6.14f];\n", oneCKF.counter, Greenpix.at<double>(0,2), Greenpix.at<double>(0,3));
			fprintf (oneckfMfile, "ckfRecorder.uav.green.right.xy(%u,:) = [% -6.14f % -6.14f];\n", oneCKF.counter, Greenpix.at<double>(0,0), Greenpix.at<double>(0,1));

			fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(0, 0), uav.ckf.Qdk.at<double>(0, 1), uav.ckf.Qdk.at<double>(0, 2), uav.ckf.Qdk.at<double>(0, 3), uav.ckf.Qdk.at<double>(0, 4));
			fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(1, 0), uav.ckf.Qdk.at<double>(1, 1), uav.ckf.Qdk.at<double>(1, 2), uav.ckf.Qdk.at<double>(1, 3), uav.ckf.Qdk.at<double>(1, 4));
			fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(2, 0), uav.ckf.Qdk.at<double>(2, 1), uav.ckf.Qdk.at<double>(2, 2), uav.ckf.Qdk.at<double>(2, 3), uav.ckf.Qdk.at<double>(2, 4));
			fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(3, 0), uav.ckf.Qdk.at<double>(3, 1), uav.ckf.Qdk.at<double>(3, 2), uav.ckf.Qdk.at<double>(3, 3), uav.ckf.Qdk.at<double>(3, 4));
			fprintf (oneckfMfile, "ckfRecorder.uav.Qdk(5,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, uav.ckf.Qdk.at<double>(4, 0), uav.ckf.Qdk.at<double>(4, 1), uav.ckf.Qdk.at<double>(4, 2), uav.ckf.Qdk.at<double>(4, 3), uav.ckf.Qdk.at<double>(4, 4));

			fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(0, 0), ugv.ckf.Qdk.at<double>(0, 1), ugv.ckf.Qdk.at<double>(0, 2), ugv.ckf.Qdk.at<double>(0, 3));
			fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(1, 0), ugv.ckf.Qdk.at<double>(1, 1), ugv.ckf.Qdk.at<double>(1, 2), ugv.ckf.Qdk.at<double>(1, 3));
			fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(2, 0), ugv.ckf.Qdk.at<double>(2, 1), ugv.ckf.Qdk.at<double>(2, 2), ugv.ckf.Qdk.at<double>(2, 3));
			fprintf (oneckfMfile, "ckfRecorder.ugv.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", oneCKF.counter, ugv.ckf.Qdk.at<double>(3, 0), ugv.ckf.Qdk.at<double>(3, 1), ugv.ckf.Qdk.at<double>(3, 2), ugv.ckf.Qdk.at<double>(3, 3));

			fprintf (oneckfMfile, "ckfRecorder.uav.Yaw(%d,1)  = % -6.14f;\n", oneCKF.counter, uav.EstYaw_gl);
			fprintf (oneckfMfile, "ckfRecorder.uav.YawBias(%d,1)  = % -6.14f;\n", oneCKF.counter, uav.EstYawBias);
			fprintf (oneckfMfile, "ckfRecorder.uav.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n\n", oneCKF.counter, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));

		//reset Qd
		oneCKF.Qdk  = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		uav.ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		ugv.ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
		// ROS_INFO("void update_oneCKF(double stereotime) close");
	}

	

/**** Landmark functions ****/
	bool serveGoalLocation(hast::ugvgoal::Request &req, hast::ugvgoal::Response &res)
	{
		// first clear the costmaps
		// clearCostmaps_cli.call(nullcall);

		if (april[req.tagID].measCount > 0)
		{
			ROS_INFO("UGV is requesting an april tag location for goal action");
			
			if (lastgoal != 1000)
			{ // then there has been a different goal already, unset it
				april[lastgoal].isgoal = false;
			} // else this is the first time, either way, set the requested tag as the goal
			lastgoal = req.tagID;
			april[req.tagID].isgoal = true;

			res.goalInMap = true;	// response to verify goal location is in the map
			res.goalP.x = april[req.tagID].EstPosition_gl.at<double>(0,0);	// position of target in map
			res.goalP.y = april[req.tagID].EstPosition_gl.at<double>(1,0);	// position of target in map
			res.goalP.z = april[req.tagID].EstPosition_gl.at<double>(2,0);	// position of target in map

			res.goalPoint.header.frame_id = uav.s_TF_gl_parent; // ugv1/hast/odom because the CKF operates relative to ugv1 starting location
			res.goalPoint.point.x = april[req.tagID].EstPosition_gl.at<double>(0,0);	// position of target in map
			res.goalPoint.point.y = april[req.tagID].EstPosition_gl.at<double>(1,0);	// position of target in map
			res.goalPoint.point.z = april[req.tagID].EstPosition_gl.at<double>(2,0);	// position of target in map

			// res.goalYaw = april[req.tagID].EstYaw_gl;	// yaw of target in map
			// res.goalYaw = 3.1415;	// yaw of target in map
			res.goalYaw = 1.522;	// yaw of target in map
			
		} else {
			ROS_INFO("requested April tag has not been observed");
			res.goalInMap = false;	// response to verify goal location is in the map
			res.goalP.x = 0;	// position of target in map
			res.goalP.y = 0;	// position of target in map
			res.goalP.z = 0;	// position of target in map
			res.goalYaw = 0;	// yaw of target in map
		}
		return true; // always reply true
	}

	void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags) 
	{
		if (oneCKF.counter >= 10 ) // only detect tags once the UAV estimate has settled
		{
		// run april update every time there is a new tag detection:
		tagArray = tags->detections;
		tagArraySize = tagArray.size();

		int id;
		// fprintf(aprilMfile,"\n");
		for(int k = 0; k != tagArraySize; k++) 
		{
			// set marker IDs
			id = tagArray[k].id;
			printlength = sprintf(markerChannel, "/april_marker_%u", id);
			s_markerid = "tag_" + patch::to_string(id);

			// marker time stamp
			tagStamp = tagArray[k].pose.header.stamp;
			tagTSpublished = tagStamp.toSec();

			// Straight vanilla transforms
			april[id].MeasPosition_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z, 1);

			// Games using TF
			double qx,qy,qz,qw, qroll, qpitch;
			tf::StampedTransform StampedXform_uav2cam, StampedXform_april2map;

			tf::Quaternion q_cam2tag(tagArray[k].pose.pose.orientation.x,  tagArray[k].pose.pose.orientation.y, tagArray[k].pose.pose.orientation.z, tagArray[k].pose.pose.orientation.w);
			tf::Vector3 t_cam2tag(tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z);
			tf::Transform Xform_cam2tag(q_cam2tag, t_cam2tag);
			tf::Matrix3x3(Xform_cam2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_cam_rad);
			april[id].MeasYaw_cam = toDegrees(april[id].MeasYaw_cam_rad);

			try {listener.lookupTransform("/hast/uav/ardrone_base_link", "/hast/uav/base_bottomcam", ros::Time(0), StampedXform_uav2cam);}
				catch (tf::TransformException ex) {ROS_INFO("ckfRecorder: /hast/uav/base_bottomcam is missing");} 
		
			try {listener.lookupTransform("/map", markerChannel, ros::Time(0), StampedXform_april2map);}
				catch (tf::TransformException ex) {ROS_INFO("ckfRecorder: /map to %s is missing", markerChannel);} 

			tf::Quaternion q_uav2cam(StampedXform_uav2cam.getRotation().x(), StampedXform_uav2cam.getRotation().y(), StampedXform_uav2cam.getRotation().z(), StampedXform_uav2cam.getRotation().w()); 
			tf::Vector3 t_uav2cam(StampedXform_uav2cam.getOrigin().x(), StampedXform_uav2cam.getOrigin().y(), StampedXform_uav2cam.getOrigin().z());
			tf::Transform Xform_uav2cam(q_uav2cam, t_uav2cam);
			tf::Transform Xform_uav2tag = Xform_uav2cam*Xform_cam2tag;

			tf::Vector3 t_map2tag(StampedXform_april2map.getOrigin().x(), StampedXform_april2map.getOrigin().y(), StampedXform_april2map.getOrigin().z());

			qx = Xform_uav2tag.getRotation().x(); 
			qy = Xform_uav2tag.getRotation().y(); 
			qz = Xform_uav2tag.getRotation().z(); 
			qw = Xform_uav2tag.getRotation().w(); 
			tf::Matrix3x3(Xform_uav2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_uav_rad);

			// if (~uav.positionFromTags)
			// {
				april[id].MeasYaw_uav = toDegrees(april[id].MeasYaw_uav_rad);
				april[id].MeasYaw_gl = wrapDegrees(uav.EstYaw_gl + april[id].MeasYaw_uav); //degrees
				april[id].MeasPosition_uav = (cv::Mat_<double>(4, 1) << Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z(), 1);
				april[id].MeasPosition_gl = uav.Hlo2gl * april[id].MeasPosition_uav;
				april[id].MeasPosition_tf = (cv::Mat_<double>(3, 1) << t_map2tag.getX(), t_map2tag.getY(), t_map2tag.getZ());

				if (uav.observedByStereo)
				{
					april[id].measCount++;
					fprintf(aprilMfile, "%% --------------- msg %s: --------------- %% \n", s_markerid.c_str());
					// fprintf(aprilMfile,"april.%s.id(%u,:) = %i;\n", s_markerid.c_str(), april[id].measCount, id);
					fprintf(aprilMfile,"april.%s.time(%u,:) = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, tagTSpublished);
					fprintf(aprilMfile,"april.%s.MeasPosition_cam(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", s_markerid.c_str(), april[id].measCount, 
										april[id].MeasPosition_cam.at<double>(0,0), april[id].MeasPosition_cam.at<double>(1,0), april[id].MeasPosition_cam.at<double>(2,0));
					fprintf(aprilMfile,"april.%s.MeasPosition_uav(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", s_markerid.c_str(), april[id].measCount, 
										april[id].MeasPosition_uav.at<double>(0,0), april[id].MeasPosition_uav.at<double>(1,0), april[id].MeasPosition_uav.at<double>(2,0));
					fprintf(aprilMfile,"april.%s.MeasPosition_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", s_markerid.c_str(), april[id].measCount, 
										april[id].MeasPosition_gl.at<double>(0,0), april[id].MeasPosition_gl.at<double>(1,0), april[id].MeasPosition_gl.at<double>(2,0));
					fprintf(aprilMfile,"april.%s.quaternion_xform(%u,:) = [% -6.14f % -6.14f % -6.14f % -6.14f];\n", s_markerid.c_str(), april[id].measCount, qx, qy, qz, qw);
					fprintf(aprilMfile,"april.%s.MeasYaw_cam(%u,:)     = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_cam);
					fprintf(aprilMfile,"april.%s.MeasYaw_uav(%u,:)     = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_uav);
					fprintf(aprilMfile,"april.%s.MeasYaw_gl(%u,:)      = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_gl);
					fprintf(aprilMfile,"april.%s.MeasYaw_uav_rad(%u,:) = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_uav_rad);
					fprintf(aprilMfile,"april.%s.MeasYaw_cam_rad(%u,:) = % -6.14f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_cam_rad);
					fprintf(aprilMfile,"april.%s.MeasPosition_tf(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", s_markerid.c_str(), april[id].measCount, 
										t_map2tag.getX(), t_map2tag.getY(), t_map2tag.getZ());

				}

				// track which tags are in the FOV (this needs to be zeroed at some point...)
				april[id].Rk_gl = uav.Rgl2lo4x4 * april[id].Rk_uav * uav.Rgl2lo4x4.t();

		} //end for



	} // end if (oneCKF.counter >= 10 ) // only detect tags once the UAV estimate has settled
	} // end void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags) 




	bool serveState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res)
	{res.state = uav.flightState; return true;}


	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{
		// ROS_WARN("ckfRecorder: nodeShutDown");
		if (ShutDown->flag)
		{
			// ugv.openAllocFile()
			ROS_INFO("ckfRecorder: Shutdown requested..");
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ckfRecorder");
	ckfRecorder cfkRec;
	ros::spin();
	return 0;
}

