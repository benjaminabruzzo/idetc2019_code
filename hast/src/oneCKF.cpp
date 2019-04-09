#include "genheaders.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
#include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"
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
			hastSLAM slam;

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

		slam.s_slamMfile = s_root + s_date + "/" + s_trial + "/slam_"  + s_trial + s_dotm;

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

		// Marker vars
		// TotalMarkers = 10; // I am using #00 through #11 == 12 with zero-indexing, without using marker _01 or _04, for some reason...
		// TotalMarkers = 5; // use only the first five to make testing easier
		slam.initSLAM(tag_max_id);
		// init robot pose
			slam.augmentedState.push_back(uav.EstPosition_gl.at<double>(0,0)); //ugv x
			slam.augmentedState.push_back(uav.EstPosition_gl.at<double>(1,0)); //ugv y
			slam.augmentedState.push_back(uav.EstPosition_gl.at<double>(2,0)); //ugv z
			slam.augmentedState.push_back(uav.EstYaw_gl); //ugv yaw
			slam.augmentedStateMeas.push_back(uav.EstPosition_gl.at<double>(0,0)); //ugv x
			slam.augmentedStateMeas.push_back(uav.EstPosition_gl.at<double>(1,0)); //ugv y
			slam.augmentedStateMeas.push_back(uav.EstPosition_gl.at<double>(2,0)); //ugv z
			slam.augmentedStateMeas.push_back(uav.EstYaw_gl); //ugv yaw

			// ugv..copyTo(slam.augmentedCovariance);


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
		slam.NumberOfTagsinFOV = 0;

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
				// writeCvMatDoubles(aprilMfile, uav.Hlo2gl, "april." + s_markerid + ".uavH", slam.slamCount);

				// track which tags are in the FOV (this needs to be zeroed at some point...)
				april[id].Rk_gl = uav.Rgl2lo4x4 * april[id].Rk_uav * uav.Rgl2lo4x4.t();
				slam.TagsInFOV[slam.NumberOfTagsinFOV] = april[id].id;
				//update the number count of tags in FOV
				slam.NumberOfTagsinFOV++;
			// } else if (uav.positionFromTags){

			// }

		} //end for

		// ROS_INFO("Number of tags in FOV %u", slam.NumberOfTagsinFOV);
		if ((slam.NumberOfTagsinFOV > 0) && uav.observedByStereo)
		{ // only run slam if there is a new measurement of tags & the UAVv has been observed at least once
			// MarkerSLAM(tagTSpublished);
			// update the map every time there is at least one tag in the FOV
			// publishAprilObstacles();
		}

		/* 
			tagArray[k].id
			tagArray[k].size
			tagArray[k].pose
			tagArray[k].pose.header
			tagArray[k].pose.header.seq
			tagArray[k].pose.header.stamp
			tagArray[k].pose.pose
			tagArray[k].pose.pose.position
			tagArray[k].pose.pose.position.x
			tagArray[k].pose.pose.position.y
			tagArray[k].pose.pose.position.z
			tagArray[k].pose.pose.orientation.x
			tagArray[k].pose.pose.orientation.y
			tagArray[k].pose.pose.orientation.z
			tagArray[k].pose.pose.orientation.w
		*/
	} // end if (oneCKF.counter >= 10 ) // only detect tags once the UAV estimate has settled
	} // end void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags) 


	void publishAprilObstacles()
	{
		// use these two tags to create a wall in the cost map
		int tag_a = 2;
		int tag_b = 3;
		double num_of_obstacles = 10;
		if ((april[tag_a].measCount > (1+downimagecountup)) && (april[tag_b].measCount > (1+downimagecountup)))
		{
			pcl::PointXYZ tag_XYZ; // pcl point for center of obstacle
			double a = (april[tag_a].EstPosition_gl.at<double>(0,0) - april[tag_b].EstPosition_gl.at<double>(0,0))/num_of_obstacles;
			double b = (april[tag_a].EstPosition_gl.at<double>(1,0) - april[tag_b].EstPosition_gl.at<double>(1,0))/num_of_obstacles;
			// fprintf(slam.slamMfile,"slam.wall.ab(%u,:) = [% -6.14f, % -6.14f];\n", slam.slamCount, a, b);
			double d, e;
			for(int k = 1; k != num_of_obstacles; k++) 
			{
				d = april[tag_b].EstPosition_gl.at<double>(0,0) + k*a;
				e = april[tag_b].EstPosition_gl.at<double>(1,0) + k*b;
				// fprintf(slam.slamMfile,"slam.wall.dek(%u,:) = [% -6.14f, % -6.14f, %i];\n", slam.slamCount, d, e, k);

				// 	point in cloud
				tag_XYZ.x = d;
				tag_XYZ.y = e;
				tag_XYZ.z = 0;
				// tag_XYZ.z = april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(2,0) ;
				// aprilCloudXYZ_ptr->points.push_back(tag_XYZ);
				tag_XYZ.z = -0.1;
				// clearCloudXYZ_ptr->points.push_back(tag_XYZ);

			}
		}

		pcl::toROSMsg(*aprilCloudXYZ_ptr, aprilCloud2_msg);
		aprilCloud2_msg.header.frame_id = "/map";
		aprilCloud2_pub.publish(aprilCloud2_msg);

		pcl::toROSMsg(*goalCloudXYZ_ptr, goalCloud2_msg);
		goalCloud2_msg.header.frame_id = "/map";
		goalCloud2_pub.publish(goalCloud2_msg);

		// pcl::toROSMsg(*clearCloudXYZ_ptr, clearCloud2_msg);
		// clearCloud2_msg.header.frame_id = "/map";
		// clearCloud2_pub.publish(clearCloud2_msg);

	}

	void MarkerSLAM(double scantime)
	{//ROS_INFO("void MarkerSLAM(double scantime)");
		// First, check which markers are being observed for the first time and which are updating measurements
		int updatingTags = 0; // number of tags that are updating measurements
		std::vector<int> updatingIDs;
		int newTags = 0; // number of first time observations
		uint ID; // id of april tag under consideration
		
		for(uint i = 0; i != slam.NumberOfTagsinFOV; i++) 
		{
			ID = april[slam.TagsInFOV[i]].id;
			// ROS_INFO("MarkerSLAM::april[slam.TagsInFOV[%i]].measCount = %u", slam.TagsInFOV[i], april[slam.TagsInFOV[i]].measCount);
			if (april[slam.TagsInFOV[i]].measCount == (1+downimagecountup)) // first, check to see if this is the first estimate of the marker:
			{
				// ROS_INFO("MarkerSLAM::slam.firstTimeForID[%i] = %i;", newTags, ID);
				slam.firstTimeForID[newTags]=ID;
				newTags++;
				// ROS_INFO("MarkerSLAM::newTags [%u]", newTags);
			} else {
				if (april[slam.TagsInFOV[i]].measCount < (1+downimagecountup))
				{
					// Do nothing
				} else { // measuremnt >= 11, start updating
					slam.updateForID[updatingTags] = ID;
					updatingIDs.push_back(ID);
					updatingTags++;
					// ROS_INFO("MarkerSLAM::updatingTags [%u]", updatingTags);
				}
			}
		} // end for(uint i = 0; i != slam.NumberOfTagsinFOV; i++) 

		// fprintf(aprilMfile, "%% ~~~~~~~~~~~~~~~~~~~~ SLAM Update ~~~~~~~~~~~~~~~~~~~~ %% \n\n");

		/* ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~*/ 
		// now that I know how many markers are new and which are repeat measurements, ...
		// I can update the measurement matrices, but only if there are any second+ measurements (otherwise skip this section)

		if (updatingTags>0) //&& (1)) // 1 vs 0 just lets be turn on and off this block
		{// now that we know there is an update for at least one april tag:
			slam.slamCount++;
			fprintf(slam.slamMfile,"\nslam.time(%u,:) = % -6.14f;\n", slam.slamCount, scantime);
			fprintf(slam.slamMfile,"slam.newTags(%u,:) = %i;\n", slam.slamCount, newTags);
			fprintf(slam.slamMfile,"slam.updatingTags(%u,:) = %i;\n", slam.slamCount, updatingTags);

			ROS_INFO("MarkerSLAM::updatingTags if (%i>0) ", updatingTags);
			cv::Mat SLAM_H; // measurement matrix based on observed april tags
			// H should only have j*4 rows (j = number of tags observed, each with four states)
			// H must have as many columns as 4*(total number of markers in dictionary) plus 4 for the UAV
			// I think H only needs as many columns as previously observed tags...
			SLAM_H = cv::Mat::zeros(updatingTags*4, (1+slam.numberOfLandmarksOrdered)*4, CV_64F);  //1+ accounts for UAv
			ROS_INFO("SLAM_H = cv::Mat::zeros(updatingTags*4, (1+slam.numberOfLandmarksOrdered)*4, CV_64F);");
			ROS_INFO("SLAM_H = cv::Mat::zeros(%i, %i, CV_64F);", updatingTags*4, (1+slam.numberOfLandmarksOrdered)*4);
			//slam.numberOfLandmarksOrdered does not need to be decremented because of the UAV states
			cv::Mat SLAM_Rk; // measurement covariance matrix for each marker
			SLAM_Rk = cv::Mat::zeros(updatingTags*4, updatingTags*4, CV_64F);
			ROS_INFO("SLAM_Rk = cv::Mat::zeros(updatingTags*4, updatingTags*4, CV_64F);");
			ROS_INFO("SLAM_Rk = cv::Mat::zeros(%i, %i, CV_64F);", updatingTags*4, updatingTags*4);
				// Rk_slam = [
				//     Rk_uav Z;...
				//     Z Rk_uav]

			// the measured states of the uav is the currently estimated of the uav
				slam.augmentedStateMeas[0] = uav.EstPosition_gl.at<double>(0,0);
				slam.augmentedStateMeas[1] = uav.EstPosition_gl.at<double>(1,0);
				slam.augmentedStateMeas[2] = uav.EstPosition_gl.at<double>(2,0);
				slam.augmentedStateMeas[3] = uav.EstYaw_gl;
				cv::Mat aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); 
				writeCvMatDoubles(slam.slamMfile, aux_C, "slam.aux_C_0", slam.slamCount);
				// cellCvMatDoubles(slam.slamMfile, aux_C, "slam.aux_C_0", slam.slamCount);
				cellCvMatDoubles(slam.slamMfile, slam.augmentedCovariance, "slam.augmentedCovariance_0", slam.slamCount);
				
				// ROS_INFO("aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); ");
				// std::cout << aux_C <<std::endl;
				cv::Mat uav_C = uav.ckf.Qdk.colRange(0,4).rowRange(0,4);
				writeCvMatDoubles(slam.slamMfile, uav_C, "slam.uav.C_0", slam.slamCount);
				// cellCvMatDoubles(slam.slamMfile, uav_C, "slam.uav_C_0", slam.slamCount);
				// ROS_INFO("uav_C = uav.ckf.Qdk.colRange(0,4).rowRange(0,4);");
				// std::cout << uav_C <<std::endl;
				// uav_C.copyTo(aux_C); // this should copy Rk to the correct sub-block
						// cv::Mat temp_C = aux_C.inv() + uav_C.inv();
						// cv::Mat temp_Cinv = temp_C.inv();
						// temp_Cinv.copyTo(aux_C);
				aux_C += uav_C;
				// ROS_INFO("aux_C += uav_C;");
				// std::cout << aux_C <<std::endl;
				aux_C.release();
				uav_C.release();
				// temp_C.release();
				// temp_Cinv.release();


			// Now that H and Rk are defined, we can populate them
			ROS_INFO("for(uint i = 0; i != updatingTags; i++)");
			for(uint i = 0; i != updatingTags; i++)
			{
				ROS_INFO("i = %i", i);
				int SLAM_Hrows = SLAM_H.rows;
				int SLAM_Hcols = SLAM_H.cols;
				ROS_INFO("[SLAM_H.rows SLAM_H.cols] = [%i %i]", SLAM_Hrows, SLAM_Hcols );
				// the first four columns of H are always block diagonals of I:
				cv::Mat aux_H_UAV = SLAM_H.colRange(0,4).rowRange(i*4,(i+1)*4);
				ROS_INFO("aux_H_UAV = SLAM_H.colRange(0,4).rowRange(i*4,(i+1)*4);");
				ROS_INFO("aux_H_UAV = SLAM_H.colRange(0,4).rowRange(%u, %u);", i*4,(i+1)*4);
				slam.I4.copyTo(aux_H_UAV); // this should write data to SLAM_H				
				aux_H_UAV.release();//release pointer

				// ROS_INFO("i = %i", i);
				uint ID = april[slam.updateForID[i]].id; // the numeric ID of the observed tag
				uint Ord = april[slam.updateForID[i]].order; // serial order of observation
				ROS_INFO("uint ID = april[slam.updateForID[%i]].id = %u;", i, ID);
				ROS_INFO("uint Ord = april[slam.updateForID[%i]].order = %u;", i, Ord);

				// the negative eye(4) needs to be shifted to the right by the eye(4) of the UAV plus the correct position in the dictionary
				cv::Mat aux_H_TAG = SLAM_H.colRange(4*Ord,4*(Ord+1)).rowRange(4*((i+1)-1), ((i+1)*4)); 
				ROS_INFO("cv::Mat aux_H_TAG = SLAM_H.colRange(4*Ord,4*(Ord+1)).rowRange(4*((i+1)-1), ((i+1)*4)); ");
				ROS_INFO("cv::Mat aux_H_TAG = SLAM_H.colRange(%u,%u).rowRange(%u, %u); ", 4*Ord, 4*(Ord+1), 4*((i+1)-1), ((i+1)*4));
				slam.nI4.copyTo(aux_H_TAG); // this should write data to SLAM_H
				aux_H_TAG.release(); //release pointer
				
				// Rk is constant for all tag measurements, but must be rotated to the global/map frame eventually
				// ROS_INFO("cv::Mat aux_Rk = SLAM_Rk.colRange(%i,%i).rowRange(%i,%i);", i*4, 4*(i+1), i*4, 4*(i+1));				
				int SLAM_Rkrows = SLAM_Rk.rows;
				int SLAM_Rkcols = SLAM_Rk.cols;
				ROS_INFO("[SLAM_Rk.rows SLAM_Rk.cols] = [%i %i]", SLAM_Rkrows, SLAM_Rkcols );
				ROS_WARN("aux_Rk = SLAM_Rk.colRange((i)*4,(i+1)*4).rowRange((i)*4,(i+1)*4); ");
				ROS_WARN("aux_Rk = SLAM_Rk.colRange(%u,%u).rowRange(%u,%u); ", (i-1)*4,i*4, (i-1)*4,i*4);
				cv::Mat aux_Rk = SLAM_Rk.colRange((i)*4,(i+1)*4).rowRange((i)*4,(i+1)*4);
				// ROS_WARN("aux_Rk = SLAM_Rk.colRange((Ord-1)*4,Ord*4).rowRange((Ord-1)*4,Ord*4); ");
				// ROS_WARN("aux_Rk = SLAM_Rk.colRange(%u,%u).rowRange(%u,%u); ", (Ord-1)*4,Ord*4, (Ord-1)*4,Ord*4);
				// cv::Mat aux_Rk = SLAM_Rk.colRange((Ord-1)*4,Ord*4).rowRange((Ord-1)*4,Ord*4);
				// this definition of Rk using the order causes issues when only a higeher order target is in view ********************************************
				// cv::Mat aux_Rk = SLAM_Rk.colRange(Ord*4,4*(Ord+1)).rowRange(Ord*4,4*(Ord+1)); 
				april[slam.updateForID[i]].Rk_gl.copyTo(aux_Rk); // this should copy Rk to the correct sub-block
					writeCvMatDoubles(slam.slamMfile, aux_Rk, "slam.tag_" + patch::to_string(ID) + ".Rk", slam.slamCount);
					aux_Rk.release();

				slam.augmentedStateMeas[4*Ord+0] = april[ID].MeasPosition_gl.at<double>(0,0);
				slam.augmentedStateMeas[4*Ord+1] = april[ID].MeasPosition_gl.at<double>(1,0);
				slam.augmentedStateMeas[4*Ord+2] = april[ID].MeasPosition_gl.at<double>(2,0);
				slam.augmentedStateMeas[4*Ord+3] = april[ID].MeasYaw_gl;
				
			} //end for(uint i = 0; i != updatingTags; i++) 


			//now Rk and H are defined, and P was grown automatically when the markers were measured the first time (see below)
			// increment slam estimate of re-observed markers and UAV
			slam.HPHt = SLAM_H * slam.augmentedCovariance * SLAM_H.t();
			slam.Sk = slam.HPHt + SLAM_Rk;
			slam.PHt = slam.augmentedCovariance*SLAM_H.t(); // PHtr = P_slam*H' 
			slam.Kk = slam.PHt * slam.Sk.inv(); // Kalman Gain, // K = PHtr * Sk
			slam.vk = SLAM_H * (cv::Mat(slam.augmentedStateMeas) - cv::Mat(slam.augmentedState));
			slam.correction = slam.Kk * slam.vk;

			// last thing to do, update the covariance matrix
			// Ck = (I-K H) Ck

			cv::Mat KH = slam.Kk * SLAM_H;
			int KHrows = KH.rows;
			int KHcols = KH.cols;
			ROS_INFO("KH [rows x cols] = [%i %i]", KHrows, KHcols);

			int Crows = slam.augmentedCovariance.rows;
			int Ccols = slam.augmentedCovariance.cols;
			ROS_INFO("slam.Ck [rows x cols] = [%i %i]", Crows, Ccols);

			cv::Mat KHI = cv::Mat::eye(KH.rows, KH.cols, CV_64F); //64f == double
			slam.augmentedCovariance = (KHI - KH) * slam.augmentedCovariance;

			int rows = slam.correction.rows;
			for(uint rown = 0; rown != rows; rown++) 
			{
				slam.augmentedState[rown] += slam.correction.at<double>(rown,0);
			}
			// end slam increment 

			fprintf(slam.slamMfile,"slam.uav.est.p.global(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", slam.slamCount, slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2]);
			fprintf(slam.slamMfile,"slam.uav.est.yaw.global(%u,:) = % -6.14f;\n", slam.slamCount, slam.augmentedState[3]);
			cv::Mat uav_Ck = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); 
				writeCvMatDoubles(slam.slamMfile, uav_Ck, "slam.uav.Ck", slam.slamCount);
				uav_Ck.release();
			cellCvMatDoubles(slam.slamMfile, SLAM_H, "slam.H", slam.slamCount);
			cellCvMatDoubles(slam.slamMfile, SLAM_Rk, "slam.Rk", slam.slamCount);
			cellCvMatDoubles(slam.slamMfile, slam.Kk, "slam.Kk", slam.slamCount);
			cellCvMatDoubles(slam.slamMfile, slam.vk, "slam.vk", slam.slamCount);
			cellCvMatDoubles(slam.slamMfile, slam.correction, "slam.correction", slam.slamCount);
			cellCvMatDoubles(slam.slamMfile, slam.augmentedCovariance, "slam.augmentedCovariance", slam.slamCount);
			cellVecDoubles(slam.slamMfile, slam.augmentedStateMeas, "slam.augmentedStateMeas", slam.slamCount);
			cellVecDoubles(slam.slamMfile, slam.augmentedState, "slam.augmentedState_k", slam.slamCount);

			fprintf(aprilMfile, "%% --------------- slam update --------------- %% \n");
			aprilCloudXYZ_ptr->points.clear();
			goalCloudXYZ_ptr->points.clear();
			pcl::PointXYZ tag_XYZ, point_XYZ; // pcl point for center of april tag
			// pcl::PointXYZ clear_XYZ; // pcl point for center of april tag

			for(uint n = 0; n != slam.numberOfLandmarksOrdered; n++) 
			{//update poses of all observed landmarks
				// ROS_INFO("n = %u", n);
				// ROS_INFO("april[%i].EstPosition_gl = slam.augmentedState[%i], slam.augmentedState[%i], slam.augmentedState[%i];", slam.landmarksOrdered[n], 4*(n+1)+0, 4*(n+1)+1, 4*(n+1)+2);
				// ROS_INFO("april[%i].EstPosition_gl = [%6.4f  %6.4f  %6.4f]", slam.landmarksOrdered[n], slam.augmentedState[4*(n+1)+0], slam.augmentedState[4*(n+1)+1], slam.augmentedState[4*(n+1)+2]);
				april[slam.landmarksOrdered[n]].EstPosition_gl = (cv::Mat_<double>(3, 1) << slam.augmentedState[4*(n+1)+0], slam.augmentedState[4*(n+1)+1], slam.augmentedState[4*(n+1)+2]);
				april[slam.landmarksOrdered[n]].EstYaw_gl = slam.augmentedState[4*(n+1)+3];
				std::string s_tagid = "tag_" + patch::to_string(april[slam.landmarksOrdered[n]].id);

				// 	point in cloud
				tag_XYZ.x = april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(0,0) ;
				tag_XYZ.y = april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(1,0) ;
				tag_XYZ.z = 0;

				point_XYZ.x = april[slam.landmarksOrdered[n]].MeasPosition_tf.at<double>(0,0) ;
				point_XYZ.y = april[slam.landmarksOrdered[n]].MeasPosition_tf.at<double>(1,0) ;
				point_XYZ.z = 0;

				if (april[slam.landmarksOrdered[n]].isgoal)
				{ // only push landmarks that are not a UGV goal location
					goalCloudXYZ_ptr->points.push_back(point_XYZ);
				} else {
					aprilCloudXYZ_ptr->points.push_back(point_XYZ);
				}

				fprintf(aprilMfile,"april.%s.EstPosition_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", s_tagid.c_str(), slam.slamCount, 
								april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(0,0), 
								april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(1,0), 
								april[slam.landmarksOrdered[n]].EstPosition_gl.at<double>(2,0));
				fprintf(aprilMfile,"april.%s.EstYaw_gl(%u,:) = % -6.14f;\n", s_tagid.c_str(), slam.slamCount, april[slam.landmarksOrdered[n]].EstYaw_gl);
				fprintf(slam.slamMfile,"slam.%s.est.p.global(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", s_tagid.c_str(), slam.slamCount, 
					slam.augmentedState[4*(n+1)+0], 
					slam.augmentedState[4*(n+1)+1], 
					slam.augmentedState[4*(n+1)+2]);
				fprintf(slam.slamMfile,"slam.%s.est.yaw.global(%u,:) = % -6.14f;\n", s_tagid.c_str(), slam.slamCount, slam.augmentedState[4*(n+1)+3]);
				// ROS_INFO("cv::Mat april_Ck = slam.augmentedCovariance.colRange(4*(n+1),4*(n+2)).rowRange(4*(n+1),4*(n+2)); ");
				// ROS_INFO("cv::Mat april_Ck = slam.augmentedCovariance.colRange(%i,%i).rowRange(%i,%i); ", 4*(n+1), 4*(n+2), 4*(n+1), 4*(n+2));
				cv::Mat april_Ck = slam.augmentedCovariance.colRange(4*(n+1),4*(n+2)).rowRange(4*(n+1),4*(n+2)); 
					writeCvMatDoubles(slam.slamMfile, april_Ck, "slam." + s_tagid + ".Ck", slam.slamCount);
					april_Ck.release();

			}

			// uav.EstPosition_gl.at<double>(0,0) += slam.correction.at<double>(0,0);
			// uav.EstPosition_gl.at<double>(1,0) += slam.correction.at<double>(1,0);
			// uav.EstPosition_gl.at<double>(2,0) += slam.correction.at<double>(2,0);
			// uav.EstYaw_gl += slam.correction.at<double>(3,0);
		} //end if (updatingTags>0)

		// If there are new tags, the whole system must be augmented:
		if (newTags>0)
		{
			if (slam.slamCount == 0)
			{
				// then set current uav pose as augmented state #1
				slam.augmentedState[0] = uav.EstPosition_gl.at<double>(0,0);
				slam.augmentedState[1] = uav.EstPosition_gl.at<double>(1,0);
				slam.augmentedState[2] = uav.EstPosition_gl.at<double>(2,0);
				slam.augmentedState[3] = uav.EstYaw_gl;
				cv::Mat aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); 
				cv::Mat uav_C = uav.ckf.Qdk.colRange(0,4).rowRange(0,4);

				uav_C.copyTo(aux_C); // this should copy Rk to the correct sub-block
				aux_C.release();
				uav_C.release();
			}
			cv::Mat newLandmarkPos, obsRk;
			double newLandmarkYaw;
			int newLandmarkID;
			// ROS_INFO("for(uint i = 0; i != newTags; i++) ");
			for(uint i = 0; i != newTags; i++) 
			{
				newLandmarkID = april[slam.firstTimeForID[i]].id;
				newLandmarkPos = april[slam.firstTimeForID[i]].getMeasPosition_gl();  // rotate this to global eventually
				newLandmarkYaw = april[slam.firstTimeForID[i]].MeasYaw_gl;
				obsRk = april[slam.firstTimeForID[i]].Rk_gl;
				slam.augmentSLAM(newLandmarkID, newLandmarkPos, newLandmarkYaw, obsRk);
				april[slam.firstTimeForID[i]].order = slam.numberOfLandmarksOrdered; // should not start at zero
				// fprintf(slam.slamMfile,"slam.order(%u+1) = %u;\n", slam.numberOfLandmarksOrdered, slam.numberOfLandmarksOrdered);
			} // end for(uint i = 0; i != newTags; i++) 
		} //end if (newTags>0)

		cellVecUints(slam.slamMfile,slam.TagsInFOV,"%%slam.TagsInFOV", slam.slamCount);
		cellVecInts(slam.slamMfile,slam.firstTimeForID,"%%slam.firstTimeForID", slam.slamCount);
		cellVecInts(slam.slamMfile,slam.updateForID,"%%slam.updateForID", slam.slamCount);
		cellVecInts(slam.slamMfile,slam.landmarksOrdered,"%%slam.landmarksOrdered", slam.slamCount);
		fprintf(slam.slamMfile,"%%slam.newTags(%u,:) = %i;\n", slam.slamCount, newTags);
		fprintf(slam.slamMfile,"%%slam.updatingTags(%u,:) = %i;\n", slam.slamCount, updatingTags);

	}	

	void aprilAlloc()
	{//ROS_INFO("void aprilAlloc()");
		s_april_prealloc = s_root + s_date + "/" + s_trial + "/april_prealloc_"  + s_trial + s_dotm;
		ROS_INFO("ckfRecorder: %s", s_april_prealloc.c_str());
		aprilAllocfile = std::fopen (s_april_prealloc.c_str(), "w");
		fprintf (aprilAllocfile, "%% %s\n", s_april_prealloc.c_str());
		fprintf (aprilAllocfile, "%%clc; \n%%clear all;\n%%close all;\n\n");

		// Loop through all markers
		std::string s_markerid; // string for april makrer id
		for(uint i = 0; i != tag_max_id; i++)
		{
			/*----- Generate Channel for TF */
			printlength = sprintf(markerChannel, "/april_marker_%u", i);
			s_markerid = "tag_" + patch::to_string(april[i].id);

			if (april[i].measCount>=1)
			{
				fprintf(aprilAllocfile,"april.%s.time = zeros(%d,1);\n", s_markerid.c_str(), april[i].measCount);
				fprintf(aprilAllocfile,"april.%s.Position_gl = zeros(%d,3);\n", s_markerid.c_str(), april[i].measCount);
				fprintf(aprilAllocfile,"april.%s.MeasPosition_uav = zeros(%d,3);\n", s_markerid.c_str(), april[i].measCount);
				fprintf(aprilAllocfile,"april.%s.EstPosition_gl = zeros(%d,3);\n", s_markerid.c_str(), april[i].measCount);
				fprintf(aprilAllocfile,"april.%s.quaternion_xform = zeros(%d,4);\n", s_markerid.c_str(), april[i].measCount);
				fprintf(aprilAllocfile, "\n");// fprintf(aprilAllocfile, "%% --------------- End Group --------------- %% \n\n");
			}
		}
	}

	bool serveState(hast::uavnavstate::Request &req, hast::uavnavstate::Response &res)
	{res.state = uav.flightState; return true;}


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
/**** Bookkeeping functions ****/
	
	void writeVecInt(std::FILE * filename, std::vector<int> vectorOfInts, std::string vectorName, int count)
	{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
		int vectorSize = vectorOfInts.size();

		fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
		for(uint i = 0; i != vectorSize; i++) 
		{
			fprintf (filename, "%i ", vectorOfInts[i]);
		}
		fprintf (filename, "];\n");
	}

	void writeVecDoubles(std::FILE * filename, std::vector<double> vectorOfDoubles, std::string vectorName, int count)
	{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
		int vectorSize = vectorOfDoubles.size();

		fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
		// fprintf (filename, "%i ", vectorSize);
		// ROS_INFO("augmentSLAM:: %s .size() = %i ", vectorName.c_str(), count);
		for(uint i = 0; i != vectorSize; i++) 
		{
			fprintf (filename," %6.8f", vectorOfDoubles[i]);
		}
		fprintf (filename, "];\n");
	}

	void cellVecDoubles(std::FILE * filename, std::vector<double> vectorOfDoubles, std::string vectorName, int count)
	{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
		int vectorSize = vectorOfDoubles.size();

		fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
		for(uint i = 0; i != vectorSize; i++) 
		{
			fprintf (filename," %6.8f", vectorOfDoubles[i]);
		}
		fprintf (filename, "];\n");
	}

	void cellVecInts(std::FILE * filename, std::vector<int> vectorOfInts, std::string vectorName, int count)
	{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
		int vectorSize = vectorOfInts.size();

		fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
		for(uint i = 0; i != vectorSize; i++) 
		{
			fprintf (filename," %i", vectorOfInts[i]);
		}
		fprintf (filename, "];\n");
	}

	void cellVecUints(std::FILE * filename, std::vector<uint> vectorOfUints, std::string vectorName, int count)
	{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
		int vectorSize = vectorOfUints.size();

		fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
		for(uint i = 0; i != vectorSize; i++) 
		{
			fprintf (filename," %u", vectorOfUints[i]);
		}
		fprintf (filename, "];\n");
	}

	void writeCvMatDoubles(std::FILE * filename, cv::Mat matrixOfDoubles, std::string matrixName, int count)
	{
		int rows = matrixOfDoubles.rows;
		int cols = matrixOfDoubles.cols;

		for(uint rown = 0; rown != rows; rown++) 
		{
			fprintf (filename, "%s(%i,:, %i) = [", matrixName.c_str(), rown+1, count);
			for(uint coln = 0; coln != cols; coln++) 
			{
				fprintf (filename, " %6.8f", matrixOfDoubles.at<double>(rown,coln));
			}
			fprintf (filename, "];\n");
		}
	}

	void cellCvMatDoubles(std::FILE * filename, cv::Mat matrixOfDoubles, std::string matrixName, int count)
	{
		int rows = matrixOfDoubles.rows;
		int cols = matrixOfDoubles.cols;


		fprintf (filename, "%s{%i,1} = [", matrixName.c_str(), count);
		for(uint rown = 0; rown != rows; rown++) 
		{
			
			for(uint coln = 0; coln != cols; coln++) 
			{
				fprintf (filename, " %6.8f", matrixOfDoubles.at<double>(rown,coln));
			}
			fprintf (filename, "; ");
		}
		fprintf (filename, "];\n");
	}
	
	cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
	{// H = [R -R*t(1:3,1); 0 0 0 1];
		cv::Mat H, nRt, t;
		t = (cv::Mat_<double>(3, 1) << t_in.at<double>(0,0),t_in.at<double>(1,0),t_in.at<double>(2,0));
		nRt = -R_in * t;
		H = (cv::Mat_<double>(4, 4) <<
			R_in.at<double>(0,0), R_in.at<double>(0,1), R_in.at<double>(0,2), nRt.at<double>(0,0),
			R_in.at<double>(1,0), R_in.at<double>(1,1), R_in.at<double>(1,2), nRt.at<double>(1,0),
			R_in.at<double>(2,0), R_in.at<double>(2,1), R_in.at<double>(2,2), nRt.at<double>(2,0),
			0,0,0,1);
		return H;
	}

	cv::Mat invertH(cv::Mat H)
	{// Hinv = [R' -R'*t; 0 0 0 1];
		cv::Mat Hinv, RT, R, t, nRTt;
		t = (cv::Mat_<double>(3, 1) << H.at<double>(0,3),H.at<double>(1,3),H.at<double>(2,3));
		R = (cv::Mat_<double>(3, 3) << 	H.at<double>(0,0), H.at<double>(0,1), H.at<double>(0,2),
																		H.at<double>(1,0), H.at<double>(1,1), H.at<double>(1,2),
																		H.at<double>(2,0), H.at<double>(2,1), H.at<double>(2,2));
		RT = R.t();
		nRTt = -RT*t;
		Hinv = (cv::Mat_<double>(4, 4) <<
			RT.at<double>(0,0), RT.at<double>(0,1), RT.at<double>(0,2), nRTt.at<double>(0,0),
			RT.at<double>(1,0), RT.at<double>(1,1), RT.at<double>(1,2), nRTt.at<double>(1,0),
			RT.at<double>(2,0), RT.at<double>(2,1), RT.at<double>(2,2), nRTt.at<double>(2,0),
			0,0,0,1);
		return Hinv;
	}

	double wrapDegrees(double angle)
	{ // Force angle to stay between +- 180 degrees
		double toplimit = 180;
		double botlimit = -180;
		while(angle > toplimit){angle -= (toplimit - botlimit);}
		while(angle < botlimit){angle += (toplimit - botlimit);}
		return angle;
	}

	double wrapRadians(double angle)
	{ // Force angle to stay between +- 180 degrees
		double toplimit = 1.57079632679;
		double botlimit = -1.57079632679;
		while(angle > toplimit){angle -= (toplimit - botlimit);}
		while(angle < botlimit){angle += (toplimit - botlimit);}
		return angle;
	}

	double toDegrees(double radians)
	{return radians*180/Pi;}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ckfRecorder");
	ckfRecorder cfkRec;
	ros::spin();
	return 0;
}

