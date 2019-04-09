#include "genheaders.hpp"
namespace enc = sensor_msgs::image_encodings;

// static const char strings
	static const char lWindow[] = "Left Image";
	static const char rWindow[] = "Right Image";

	static const char Redstr[] = "Red";
	static const char Bluestr[] = "Blue";
	static const char Greenstr[] = "Green";

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(5) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloudCAM_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloudUGV_ptr (new pcl::PointCloud<pcl::PointXYZ>);

class stereoObs
{
private:
	/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
		/*----- image transport Channels */
		image_transport::ImageTransport it;
		image_transport::Subscriber LeftImage_sub, LeftRaw_sub;
		image_transport::Subscriber RightImage_sub, RightRaw_sub;
		std::string s_left_rect_topic, s_right_rect_topic;
		ros::Subscriber SaveTrigger_sub;
			std_msgs::Empty Null_msg;
		bool showcircles, showYUV;
		bool gazebo_sim;


		/*----- Client Services */
		ros::ServiceServer OdomSwitch_ser;

		std::string s_Pose_pub, s_stereoCloudCAM_pub, s_stereoCloudUGV_pub, s_OdomSwitch_ser;
		/*----- Pose Publishers */
		ros::Publisher Pose_pub, Pixels_pub; //, Odom_pub;
			hast::uavstate Pose_msg;
			// hast::markers Pixels_msg;
		tf::TransformBroadcaster uav_TF_pub;
		tf::Transform uav_TF;
		tf::Matrix3x3 uavR_TF;
		tf::Quaternion uavQ_TF;
		std::string s_uav_TF_parent, s_uav_TF_child;

		ros::Subscriber NavData_sub, HastShutDown_sub;
			bool PoseOdomBitFlag;
			// bool odomMODE;

		ros::Publisher stereoCloudCAM_pub, stereoCloudUGV_pub; 
			sensor_msgs::PointCloud2 stereoCloudCAM_msg, stereoCloudUGV_msg;

	/*--------- Constants ------------- */
		int L2Norm;// Frobenius norm for CV norm() function
		double FocalLength, Baseline, Pi;
		cv::Scalar RGBRed, RGBBlue, RGBGreen;

	/*---------  File Recorder ------------- */
		std::string s_filename, s_prealloc, s_trial, s_dotm, s_root, s_handle, s_date, s_user, s_ugv_n;
		std::FILE * pFile;
		std::FILE * allocFile;
		int goodloops, badloops, guidcount, odomcount;

	/*--------- Time, Counters, and Flags Containers ------------- */
		bool stereoflag, newLeft, newRight, SaveImages;
		double tnow;
		ros::Time nowStamp;
		int calledCount;
		// uint IsOncount;

	/*---------Imagery Variables ------------- */
		/*--------- Constant containers ------------- */
		int Gkx, Gky; /* Window size for mxn blurring window*/
		double GsigmaX, GsigmaY; /* Gaussian Standard Deviation in X and Y*/
		double Cx, Cy;
		int image_width, image_height;

		/*--------- Image Containers ------------- */
		ros::Time leftStamp_raw, rightStamp_raw;
		ros::Time leftStamp, rightStamp;
		cv::Mat left_image, right_image, left_imageg, right_imageg;
		cv::Mat rCircle, lCircle;
		cv::Mat lContours, rContours;

		std::string s_LeftOriginal, s_RightOriginal;
		std::string s_lCircle, s_rCircle;
		std::string s_lContour, s_rContour;

		cv::Mat lYUV, lYUVg, rYUV, rYUVg;
		cv::Mat lY, lU, lV;
		cv::Mat rY, rU, rV;
		std::string s_imString;

	/*--- KeyPoint Containers --- */
		int roi_halfwindow;
		std::vector<cv::KeyPoint> rRedKeypoint, lRedKeypoint;
		std::vector<cv::KeyPoint> rBlueKeypoint, lBlueKeypoint;
		std::vector<cv::KeyPoint> rGreenKeypoint, lGreenKeypoint;

		/*--- Imagery Containers --- */
			cv::Vec<double,4> Bluepix, Greenpix, Redpix; // pixel vectors [rX rY lX lY]
			cv::Vec<double,4> RawBluepix, RawGreenpix, RawRedpix, RawPixels; // pixel vectors [rX rY lX lY]
			cv::Vec<double,2> RedRadii, BlueRadii, GreenRadii, RawRadii; // Target radii, [right left]
			int Red_u_low, Red_u_high, Red_v_low, Red_v_high;
			int Blue_u_low, Blue_u_high, Blue_v_low, Blue_v_high;
			int Green_u_low, Green_u_high, Green_v_low, Green_v_high;

			int EdgeThreshold;
			cv::Mat closingElement; // = getStructuringElement(2,cv::Size( 7, 7 ),cv::Point(-1,-1));

			std::vector<std::vector<cv::Point> > leftGreenContours, leftBlueContours, leftRedContours;
			std::vector<std::vector<cv::Point> > rightGreenContours, rightBlueContours, rightRedContours;
			std::vector<std::vector<cv::Point> > lGreenPolygons, lBluePolygons, lRedPolygons;
			std::vector<std::vector<cv::Point> > rGreenPolygons, rBluePolygons, rRedPolygons;

			/*--- Green HSV --- */
			cv::Mat GreenLEDmin, GreenLEDmax;
			cv::Mat leftGreenLEDbitwiseOR, rightGreenLEDbitwiseOR;
			cv::Mat leftGreenLEDdilated, rightGreenLEDdilated;
			cv::Mat leftGreenLEDeroded, rightGreenLEDeroded;
			cv::Mat leftGreenLEDmorph, rightGreenLEDmorph;
			cv::Mat leftGreenLEDclosed, rightGreenLEDclosed;
			std::string s_GreenLEDmorph;

			/*--- Blue HSV --- */
			cv::Mat BlueLEDmin, BlueLEDmax;
			cv::Mat leftBlueLEDbitwiseOR, rightBlueLEDbitwiseOR;
			cv::Mat leftBlueLEDdilated, rightBlueLEDdilated;
			cv::Mat leftBlueLEDeroded, rightBlueLEDeroded;
			cv::Mat leftBlueLEDmorph, rightBlueLEDmorph;
			cv::Mat leftBlueLEDclosed, rightBlueLEDclosed;
			std::string s_BlueLEDmorph;

			/*--- Red HSV --- */
			cv::Mat RedLEDmin, RedLEDmax;
			cv::Mat leftRedLEDbitwiseOR, rightRedLEDbitwiseOR;
			cv::Mat leftRedLEDdilated, rightRedLEDdilated;
			cv::Mat leftRedLEDeroded, rightRedLEDeroded;
			cv::Mat leftRedLEDmorph, rightRedLEDmorph;
			cv::Mat leftRedLEDclosed, rightRedLEDclosed;
			std::string s_RedLEDmorph;

		// boolean tags for which images to save
			bool save_originals, save_circles, save_rect;
			bool save_raw, trigger_save_raw;
			std::string s_left_raw_topic, s_right_raw_topic, s_raw_trigger_topic;
			int leftraw_count, rightraw_count;
			int leftrect_count, rightrect_count;
			cv::Mat LeftRawIamge, RightRawIamge;
			double RawSaveTime;

	/*--------- 3D Position Containers ------------- */
		pcl::PointXYZ camCloudPt, ugvCloudPt; // pcl point for center of obstacle

		cv::Matx<double,4,1> BlueP_cam, GreenP_cam, RedP_cam, UAV_p_cam;

		cv::Matx<double,4,1> soloBlueP_cam, soloGreenP_cam, soloRedP_cam;
		cv::Matx<double,4,1> soloBlueP_ugv, soloGreenP_ugv, soloRedP_ugv;
		int soloBlueCount, soloGreenCount, soloRedCount;
		double soloTime;

		cv::Matx<double,3,3> Redscale, Bluescale, Greenscale;

		cv::Matx<double,3,3> BlueJ_cam, RedJ_cam, GreenJ_cam;
		cv::Matx<double,3,3> BlueJJt_cam, RedJJt_cam, GreenJJt_cam;

		cv::Matx<double,4,1> BlueP_ugv, GreenP_ugv, RedP_ugv, UAV_p_ugv;
		cv::Matx<double,3,3> BlueJJt_ugv, RedJJt_ugv, GreenJJt_ugv, UAV_p_cov;

	/*--------- Camera to ugv Transform Matrices ------------- */
		double Wedge;
		// double dRx, dRz;
		double camZOffset, camXOffset, camYOffset, RightOffset; //Blue wedge angle, cosine/sine of the wedge angle X/Z offset of camera axis
		cv::Matx<double,4,4> cam2ugv;
		cv::Matx<double,3,3> cam2ugv33, Rz;

		// Matrices for the cross product orientation uncertainty
		cv::Matx<double,4,1> GmB, RmB, BmR, RmG, znum;
		bool good_axes;
		uint bad_axes_count;
		cv::Matx<double,4,1> x_axis, y_axis, z_axis; //Calculated UAV axes from Circle centers
		cv::Matx<double,3,3> z_JJt, y_JJt, x_JJt;
		double yaw_ugv, UAV_yaw_var;

		// camera nodelet parameters
		double left_shutter_speed, right_shutter_speed;
		double left_wb_red, right_wb_red;
		double left_wb_blue, right_wb_blue;
		double left_gain, right_gain;

		std::string s_cam_cloud_frameID, s_ugv_cloud_frameID;

public:
	stereoObs()
	: it(n)
	{
		
		if(ros::param::get("~ugv_n",s_ugv_n)){} else {s_ugv_n = "kobuki";}
		if(ros::param::get("~user",s_user)){} else {s_user = "";}
		if(ros::param::get("~date",s_date)){} else {s_date = "";}
		if(ros::param::get("~trial",s_trial)){} else {s_trial = "";}

		if(ros::param::get("~gazebo_sim",gazebo_sim)){} else {gazebo_sim = false;}
		if(ros::param::get("~saverect",save_rect)){} else {save_rect = false;}
		if(ros::param::get("~showcircles",showcircles)){} else {showcircles = false;}
		if(ros::param::get("~showYUV",showYUV)){} else {showYUV = false;}
		
		ROS_INFO("stereoobs: Constructing stereoObs() ..");
		/*--------- Initialize ROS Communication ------------- */
		/*-----  Publishers and Subscribers */
		// LeftImage_sub    = it.subscribe("/sync/left/image_rect_color",   1,  &stereoObs::leftCvBridge, this);
		// RightImage_sub   = it.subscribe("/sync/right/image_rect_color",  1,  &stereoObs::rightCvBridge, this);

		if(ros::param::get("~saveraw",save_raw)){} else {save_raw = false;}
		if(ros::param::get("~trigger_saveraw",trigger_save_raw)){} else {trigger_save_raw = false;}
		if (save_raw)
		{		
			if(ros::param::get("~raw_trigger_topic",s_raw_trigger_topic)){} else {s_raw_trigger_topic = "/pgrstereo/saveTrigger";}
			if(ros::param::get("~left_raw_topic",s_left_raw_topic)){} else {s_left_raw_topic = "/pgrstereo/left/image_raw";}
			if(ros::param::get("~right_raw_topic",s_right_raw_topic)){} else {s_right_raw_topic = "/pgrstereo/right/image_raw";}
			LeftRaw_sub		 = it.subscribe(s_left_raw_topic,   1,  &stereoObs::leftCvBridge_raw, this);
			RightRaw_sub	 = it.subscribe(s_right_raw_topic,  1,  &stereoObs::rightCvBridge_raw, this);
			SaveTrigger_sub	 = n.subscribe(s_raw_trigger_topic,  1,  &stereoObs::SaveRaw_wait, this);
		}

		if(ros::param::get("~left_rect_topic",s_left_rect_topic)){} else {s_left_rect_topic = "/pgrstereo/left/image_rect_color";}
		if(ros::param::get("~right_rect_topic",s_right_rect_topic)){} else {s_right_rect_topic = "/pgrstereo/right/image_rect_color";}


		LeftImage_sub    = it.subscribe(s_left_rect_topic,   1,  &stereoObs::leftCvBridge, this);
		RightImage_sub   = it.subscribe(s_right_rect_topic,  1,  &stereoObs::rightCvBridge, this);
		// LeftImage_sub    = it.subscribe("/left/camera/image_rect_color",   1,  &stereoObs::leftCvBridge, this);
		// RightImage_sub   = it.subscribe("/right/camera/image_rect_color",  1,  &stereoObs::rightCvBridge, this);
		// NavData_sub      = n.subscribe("/ardrone/navdata",           10, &stereoObs::stereoCalc , this);
		HastShutDown_sub = n.subscribe("/hast/shutdown",           10, &stereoObs::nodeShutDown, this);
		// Pixels_pub       = n.advertise<hast::markers>("/hast/stereo/pixels", 30);

	// ~~~~~~~~~~~~~~~~~~ These need to be soft addressed

		if(ros::param::get("~Pose_pub",s_Pose_pub)){} else {s_Pose_pub = "/hast/stereo/pose";}
		if(ros::param::get("~OdomSwitch_ser" ,s_OdomSwitch_ser)){} else {s_OdomSwitch_ser = "/hast/service/stereo/OdomSwitch";}

		if(ros::param::get("~cam_cloud_frameID",s_cam_cloud_frameID)){} else {s_cam_cloud_frameID = "";}
		if(ros::param::get("~ugv_cloud_frameID",s_ugv_cloud_frameID)){} else {s_ugv_cloud_frameID = "";}
		if(ros::param::get("~stereoCloudCAM_pub",s_stereoCloudCAM_pub)){} else {s_stereoCloudCAM_pub = "/hast/stereo/markerCloudCAM";}
		if(ros::param::get("~stereoCloudUGV_pub",s_stereoCloudUGV_pub)){} else {s_stereoCloudUGV_pub = "/hast/stereo/markerCloudUGV";}


		Pose_pub         = n.advertise<hast::uavstate>(s_Pose_pub, 30); Pose_msg.id = 0;
		stereoCloudCAM_pub = n.advertise<sensor_msgs::PointCloud2>(s_stereoCloudCAM_pub, 1);
		stereoCloudUGV_pub = n.advertise<sensor_msgs::PointCloud2>(s_stereoCloudUGV_pub, 1);

	// ~~~~~~~~~~~~~~~~~~ These need to be soft addressed

		/*---------  File Recorder Initilizer ------------- */
		s_handle = s_ugv_n + "Stereo";
		s_root = "/home/" + s_user + "/ros/data/";
		s_dotm = ".m";
		s_filename = s_root + s_date + "/" + s_trial + "/" + s_handle + "_" + s_trial + s_dotm;
		ROS_INFO("stereoobs: %s", s_filename.c_str());
		pFile = std::fopen (s_filename.c_str(),"w");


		/*--------- Math Constants ------------- */
		L2Norm = 4; // Frobenius norm for CV norm() function
		Pi  = atan(1)*4; // 3.14159...
		RGBRed = cv::Scalar(0,255,255);
		RGBBlue = cv::Scalar(255,255,0);
		RGBGreen = cv::Scalar(255,0,255);

		EdgeThreshold = 100;
		// closingElement = getStructuringElement(2,cv::Size( 5, 5 ),cv::Point(-1,-1));
		closingElement = getStructuringElement(2,cv::Size( 7, 7 ),cv::Point(-1,-1));
		// closingElement = getStructuringElement(2,cv::Size( 9, 9 ),cv::Point(-1,-1));
		// closingElement = getStructuringElement(2,cv::Size( 11, 11 ),cv::Point(-1,-1));
		

		Gkx = 5; /* Window size for mxn blurring window*/
		Gky = 5; /* Window size for mxn blurring window*/
		GsigmaX = 2.5; /* Gaussian Standard Deviation in X and Y*/
		GsigmaY = 2.5; /* Gaussian Standard Deviation in X and Y*/

		if(ros::param::get("~roi_halfwindow",roi_halfwindow)){} else {roi_halfwindow = 75;}

		if(ros::param::get("~red_u_high",Red_u_high)){} else {Red_u_high = 255;}
		if(ros::param::get("~red_v_high",Red_v_high)){} else {Red_v_high = 100;}
		if(ros::param::get("~red_u_low" ,Red_u_low)){} else {Red_u_low = 140;}
		if(ros::param::get("~red_v_low" ,Red_v_low)){} else {Red_v_low = 0;}

		if(ros::param::get("~blue_u_high",Blue_u_high)){} else {Blue_u_high = 110;}
		if(ros::param::get("~blue_v_high",Blue_v_high)){} else {Blue_v_high = 255;}
		if(ros::param::get("~blue_u_low" ,Blue_u_low)){} else {Blue_u_low = 0;}
		if(ros::param::get("~blue_v_low" ,Blue_v_low)){} else {Blue_v_low = 125;}

		if(ros::param::get("~green_u_high",Green_u_high)){} else {Green_u_high = 110;}
		if(ros::param::get("~green_v_high",Green_v_high)){} else {Green_v_high = 100;}
		if(ros::param::get("~green_u_low" ,Green_u_low)){} else {Green_u_low = 0;}
		if(ros::param::get("~green_v_low" ,Green_v_low)){} else {Green_v_low = 10;}
		

		fprintf (pFile,"%s.thresholds.red.u = [%i %i];\n", s_handle.c_str(), Red_u_high, Red_u_low);
		fprintf (pFile,"%s.thresholds.red.v = [%i %i];\n", s_handle.c_str(), Red_v_high, Red_v_low);
		fprintf (pFile,"%s.thresholds.blue.u = [%i %i];\n", s_handle.c_str(), Blue_u_high, Blue_u_low);
		fprintf (pFile,"%s.thresholds.blue.v = [%i %i];\n", s_handle.c_str(), Blue_v_high, Blue_v_low);
		fprintf (pFile,"%s.thresholds.green.u = [%i %i];\n", s_handle.c_str(), Green_u_high, Green_u_low);
		fprintf (pFile,"%s.thresholds.green.v = [%i %i];\n", s_handle.c_str(), Green_v_high, Green_v_low);


		/*--------- Time, Counters, and Flags Containers ------------- */
		goodloops = 0;
		badloops = 0;
		guidcount = 0;
		odomcount = 0;
		calledCount = 0;
		bad_axes_count = 0;
		leftraw_count = 0;
		rightraw_count = 0;
		soloRedCount = 0;
		soloBlueCount = 0;
		soloGreenCount = 0;
		soloTime = 0;

		// odomMODE = true;
		PoseOdomBitFlag = true;
		stereoflag = false;
		newLeft = false;
		newRight = false;
		if(n.getParam("/hast/stereo/SaveImages", SaveImages)){} else {SaveImages = false;}
		if (SaveImages){ROS_WARN("Saving Images");}

		tnow = ros::Time::now().toSec();

		/*--------- Initialize 3d position containers -------------  , */ 
		RawRedpix[0] = 0; RawRedpix[1] = 0; RawRedpix[2] = 0; RawRedpix[3] = 0;
		RawBluepix[0] = 0; RawBluepix[1] = 0; RawBluepix[2] = 0; RawBluepix[3] = 0;
		RawGreenpix[0] = 0; RawGreenpix[1] = 0; RawGreenpix[2] = 0; RawGreenpix[3] = 0;

		RedP_cam  = cv::Matx41d(0, 0, 0, 1);
		BlueP_cam = cv::Matx41d(0, 0, 0, 1);
		GreenP_cam  = cv::Matx41d(0, 0, 0, 1);
		UAV_p_cam  = cv::Matx41d(0, 0, 0, 1);

		RedJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		BlueJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		GreenJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);

		Redscale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // scale for JJt calculation, Red is the best
		// Bluescale  = cv::Matx33d(9.068,0,0, 0,8.7718,0, 0,0,1.7534); // ratio of Blue to Red [xl xr y] = [9.068 8.7718 1.7534]
		// Greenscale  = cv::Matx33d(31.8011,0,0, 0,54.8634,0, 0,0,2.4954); // ratio of Green to Red [xl xr y] = [31.8011 54.8634 2.4954]
		Bluescale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // ratio of Blue to Red [xl xr y] = [9.068 8.7718 1.7534]
		Greenscale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // ratio of Green to Red [xl xr y] = [31.8011 54.8634 2.4954]

		BlueJJt_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		GreenJJt_cam = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		RedJJt_cam   = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);


		RedP_ugv   = cv::Matx41d(0, 0, 0, 1);
		BlueP_ugv  = cv::Matx41d(0, 0, 0, 1);
		GreenP_ugv = cv::Matx41d(0, 0, 0, 1);

		UAV_p_ugv = cv::Matx41d(0, 0, 0, 1);
		UAV_p_cov = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		BlueJJt_ugv  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		GreenJJt_ugv = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
		RedJJt_ugv   = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);

		soloRedP_ugv   = cv::Matx41d(0, 0, 0, 1);
		soloBlueP_ugv  = cv::Matx41d(0, 0, 0, 1);
		soloGreenP_ugv = cv::Matx41d(0, 0, 0, 1);
		soloRedP_cam   = cv::Matx41d(0, 0, 0, 1);
		soloBlueP_cam  = cv::Matx41d(0, 0, 0, 1);
		soloGreenP_cam  = cv::Matx41d(0, 0, 0, 1);


		/*--------- uav TF topics ------------- */
		// if(n.getParam("/hast/stereo/SaveImages", SaveImages)){} else {SaveImages = false;}
		ros::param::get("~uav_TF_parent",s_uav_TF_parent);
		ros::param::get("~uav_TF_child",s_uav_TF_child);

		/*--------- Camera to ugv Transform Matrix ------------- */
		/*--------- Imagery Constants ------------- */
		// if(n.getParam("/hast/stereo/FocalLength", FocalLength)){} else {FocalLength = 700;}
		// if(n.getParam("/hast/stereo/Baseline", Baseline)){} else {Baseline = 0.155;}
		// if(n.getParam("/hast/stereo/Cx", Cx)){} else {Cx = 400;}
		// if(n.getParam("/hast/stereo/Cy", Cy)){} else {Cy = 235;}
		// if(n.getParam("/hast/stereo/RightOffset", RightOffset)){} else {RightOffset = -100;}
		// if(n.getParam("/hast/stereo/image_width", image_width)){} else {image_width = 640;}  // [px]
		// if(n.getParam("/hast/stereo/image_height", image_height)){} else {image_height = 480;}  // [px]

		if(ros::param::get("~FocalLength", FocalLength)){} else {FocalLength = 700;}
		if(ros::param::get("~Baseline", Baseline)){} else {Baseline = 0.155;}
		if(ros::param::get("~RightOffset", RightOffset)){} else {RightOffset = -100;}
		if(ros::param::get("~camXOffset", camXOffset)){} else {camXOffset = 0.00;}
		if(ros::param::get("~camYOffset", camYOffset)){} else {camYOffset = 0.00;}
		if(ros::param::get("~camZOffset", camZOffset)){} else {camZOffset = 0.221;}
		if(ros::param::get("~Cx", Cx)){} else {Cx = 400;}
		if(ros::param::get("~Cy", Cy)){} else {Cy = 235;}
		if(ros::param::get("~image_width", image_width)){} else {image_width = 640;}  // [px]
		if(ros::param::get("~image_height", image_height)){} else {image_height = 480;}  // [px]

		if(n.getParam("/pgrstereo/left/left_nodelet/shutter_speed", left_shutter_speed)){} else {left_shutter_speed = 0;}
		if(n.getParam("/pgrstereo/left/left_nodelet/white_balance_red", left_wb_red)){} else {left_wb_red = 0;}
		if(n.getParam("/pgrstereo/left/left_nodelet/white_balance_blue", left_wb_blue)){} else {left_wb_blue = 0;}
		if(n.getParam("/pgrstereo/left/left_nodelet/gain", left_gain)){} else {left_gain = 0;}
		if(n.getParam("/pgrstereo/right/right_nodelet/shutter_speed", right_shutter_speed)){} else {right_shutter_speed = 0;}
		if(n.getParam("/pgrstereo/right/right_nodelet/white_balance_red", right_wb_red)){} else {right_wb_red = 0;}
		if(n.getParam("/pgrstereo/right/right_nodelet/white_balance_blue", right_wb_blue)){} else {right_wb_blue = 0;}
		if(n.getParam("/pgrstereo/right/right_nodelet/gain", right_gain)){} else {right_gain = 0;}


		fprintf (pFile,"%s.pgryaml.left_shutter_speed = % -6.8f;\n", s_handle.c_str(), left_shutter_speed);
		fprintf (pFile,"%s.pgryaml.left_wb_red = % -6.8f;\n", s_handle.c_str(), left_wb_red);
		fprintf (pFile,"%s.pgryaml.left_wb_blue = % -6.8f;\n", s_handle.c_str(), left_wb_blue);
		fprintf (pFile,"%s.pgryaml.left_gain = % -6.8f;\n", s_handle.c_str(), left_gain);

		fprintf (pFile,"%s.pgryaml.right_shutter_speed = % -6.8f;\n", s_handle.c_str(), right_shutter_speed);
		fprintf (pFile,"%s.pgryaml.right_wb_red = % -6.8f;\n", s_handle.c_str(), right_wb_red);
		fprintf (pFile,"%s.pgryaml.right_wb_blue = % -6.8f;\n", s_handle.c_str(), right_wb_blue);
		fprintf (pFile,"%s.pgryaml.right_gain = % -6.8f;\n", s_handle.c_str(), right_gain);


		// camXOffset = 0.216143033313071;
		// camYOffset = 0.029736772947058;
		// camZOffset = 0.323833236722482;

		// calculated from 20180323 004
		// camXOffset = 0.0502;
		// camYOffset = 0.0222;
		// camZOffset = 0.2110;
		
		if (!gazebo_sim)
		{
			// calculated from 20180619 008
			camXOffset = 0.18;
			camYOffset = 0.006;
			camZOffset = 0.3;

		}



		fprintf (pFile,"%s.pgryaml.camXOffset = % -6.8f;\n", s_handle.c_str(), camXOffset);
		fprintf (pFile,"%s.pgryaml.camYOffset = % -6.8f;\n", s_handle.c_str(), camYOffset);
		fprintf (pFile,"%s.pgryaml.camZOffset = % -6.8f;\n", s_handle.c_str(), camZOffset);

		fprintf (pFile,"%s.pgryaml.Cy = [% -6.8f];\n", s_handle.c_str(), Cy);
		fprintf (pFile,"%s.pgryaml.Cx = [% -6.8f];\n", s_handle.c_str(), Cx);
		fprintf (pFile,"%s.pgryaml.Baseline = % -6.8f;\n", s_handle.c_str(), Baseline);
		fprintf (pFile,"%s.pgryaml.FocalLength = % -6.8f;\n", s_handle.c_str(), FocalLength);
		fprintf (pFile,"%s.pgryaml.RightOffset = [% -6.8f];\n", s_handle.c_str(), RightOffset);
		fprintf (pFile,"%s.pgryaml.image_width = %i;\n", s_handle.c_str(), image_width);
		fprintf (pFile,"%s.pgryaml.image_height = %i;\n", s_handle.c_str(), image_height);
		fprintf (pFile,"%s.pgryaml.roi_halfwindow = %i;\n", s_handle.c_str(), roi_halfwindow);


	// cam2ugv33 = makeRz(-(90-dRx)*Pi/180)*makeRx(-(90-Wedge)*Pi/180);
		Wedge = 20; // tilt of cameras above the ugv body horizon
		fprintf (pFile,"%s.pgryaml.Wedge = %6.4f;\n", s_handle.c_str(), Wedge);
		cam2ugv33 = makeRz(-(90)*Pi/180)*makeRx(-(90-Wedge)*Pi/180);
		
		// cam2ugv33 = cv::Matx33d(
		// 						0.028162988892403, -0.336011273469401,  0.941436811558863,
		// 						0.999409040836297, -0.009103804086312, -0.033146490701618,
		// 						0.019708250843766,  0.941813965097476,  0.335556314194875);

		cam2ugv = cv::Matx44d(
								cam2ugv33(0,0), cam2ugv33(0,1), cam2ugv33(0,2), camXOffset,
								cam2ugv33(1,0), cam2ugv33(1,1), cam2ugv33(1,2), camYOffset,
								cam2ugv33(2,0), cam2ugv33(2,1), cam2ugv33(2,2), camZOffset,
								0,   0,  0, 1);
   
		// based on work from wednesday 20180314
		// 0.028162988892403  -0.336011273469401   0.941436811558863   0.216143033313071
		// 0.999409040836297  -0.009103804086312  -0.033146490701618   0.029736772947058
		// 0.019708250843766   0.941813965097476   0.335556314194875   0.323833236722482
		//                 0                   0                   0   1.000000000000000
		
		// cw  = cos(Wedge*Pi/180.0);
		// sw  = sin(Wedge*Pi/180.0);
		// cam2ugv33 = cv::Matx33d(
		//             0, -cw,-sw,
		//             1,   0,  0,
		//             0, -sw, cw);
		// ROS_INFO("stereoobs: **************************************************");
		// ROS_INFO("stereoobs: **************************************************");
		// ROS_INFO("stereoobs: cw = cos(% -6.8f * % -6.8f / 180) = % -6.8f" ,Wedge, Pi, cw);
		// ROS_INFO("stereoobs: sw = sin(% -6.8f * % -6.8f / 180) = % -6.8f" ,Wedge, Pi, sw);
		// ROS_INFO("stereoobs: -------cam2ugv-------------------------------------");
		// ROS_INFO("stereoobs: [00 01 02 03] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ", cam2ugv(0,0), cam2ugv(0,1), cam2ugv(0,2), cam2ugv(0,3));
		// ROS_INFO("stereoobs: [10 11 12 13] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ", cam2ugv(1,0), cam2ugv(1,1), cam2ugv(1,2), cam2ugv(1,3));
		// ROS_INFO("stereoobs: [20 21 22 23] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ", cam2ugv(2,0), cam2ugv(2,1), cam2ugv(2,2), cam2ugv(2,3));
		// ROS_INFO("stereoobs: [30 31 32 33] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ", cam2ugv(3,0), cam2ugv(3,1), cam2ugv(3,2), cam2ugv(3,3));
		// ROS_INFO("stereoobs: **************************************************");
		// ROS_INFO("stereoobs: **************************************************");
		// ROS_INFO("stereoobs: -------Baseline = [% -6.8f]--------------------------", Baseline);
		// ROS_INFO("stereoobs: -------Projection Matrix--------------------------");
		// ROS_INFO("stereoobs: [00 01 02 03] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ", FocalLength,     0.0,  Cx, 0.0);
		// ROS_INFO("stereoobs: [10 11 12 13] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ",     0.0, FocalLength,  Cy, 0.0);
		// ROS_INFO("stereoobs: [20 21 22 23] = [% -6.8f % -6.8f % -6.8f % -6.8f]  ",     0.0,     0.0, 1.0, 0.0);
		// ROS_INFO("stereoobs: **************************************************");
		// ROS_INFO("stereoobs: **************************************************");
		ros::Duration(0.05).sleep(); // sleep for 'x' second(s).

		fprintf (pFile,"%s.cam2ugv(1,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), cam2ugv(0,0), cam2ugv(0,1), cam2ugv(0,2), cam2ugv(0,3));
		fprintf (pFile,"%s.cam2ugv(2,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), cam2ugv(1,0), cam2ugv(1,1), cam2ugv(1,2), cam2ugv(1,3));
		fprintf (pFile,"%s.cam2ugv(3,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), cam2ugv(2,0), cam2ugv(2,1), cam2ugv(2,2), cam2ugv(2,3));
		fprintf (pFile,"%s.cam2ugv(4,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), cam2ugv(3,0), cam2ugv(3,1), cam2ugv(3,2), cam2ugv(3,3));

		fprintf (pFile,"%s.Rz(1,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), Rz(0,0), Rz(0,1), Rz(0,2));
		fprintf (pFile,"%s.Rz(2,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), Rz(1,0), Rz(1,1), Rz(1,2));
		fprintf (pFile,"%s.Rz(3,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), Rz(2,0), Rz(2,1), Rz(2,2));

		fprintf (pFile,"%s.Projection.Left(1,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), FocalLength,     0.0,  Cx, 0.0);
		fprintf (pFile,"%s.Projection.Left(2,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(),     0.0, FocalLength,  Cy, 0.0);
		fprintf (pFile,"%s.Projection.Left(3,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(),     0.0,     0.0, 1.0, 0.0);
		fprintf (pFile,"%s.Projection.Right(1,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), FocalLength,     0.0,  Cx, RightOffset);
		fprintf (pFile,"%s.Projection.Right(2,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(),     0.0, FocalLength,  Cy, 0.0);
		fprintf (pFile,"%s.Projection.Right(3,:) = [% -6.8f, % -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(),     0.0,     0.0, 1.0, 0.0);

		fprintf (pFile,"%s.Baseline = % -6.8f;\n", s_handle.c_str(), Baseline);
		fprintf (pFile,"%s.FocalLength = % -6.8f;\n", s_handle.c_str(), FocalLength);
		fprintf (pFile,"%s.Cx = [% -6.8f];\n", s_handle.c_str(), Cx);
		fprintf (pFile,"%s.Cy = [% -6.8f];\n", s_handle.c_str(), Cy);

		
		ROS_INFO("stereoobs: Watching..");
	}//close constructor
	~stereoObs()
	{
		cv::destroyWindow(rWindow);
		cv::destroyWindow(lWindow);
	}//close destructor

/* ################## --------- ROS Communication Functions --------- ################## */

	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{ // node cleanup for end of experiment
		if(ShutDown->flag)
			{
				ROS_INFO("stereoobs: Stereo Obs Shutting Down...");
				ros::shutdown();
			}
	}

	void leftCvBridge_raw(const sensor_msgs::ImageConstPtr& msg)
	{

		leftStamp_raw = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			LeftRawIamge = cv_ptr->image;
			// showImage(LeftRawIamge, "left");
			if(!trigger_save_raw)
			{
				s_imString = s_root + s_date + "/" + s_trial + "/original/left_raw_" + patch::to_string(++leftraw_count) + ".png"; 
				imwrite(s_imString.c_str(), cv_ptr->image);
				fprintf (pFile,"%s.leftraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), leftraw_count, ros::Time::now().toSec()); // this one needs to be here for plotting purposes
			}
		}//end try
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}//end catch
	} // end void Cbl

	void rightCvBridge_raw(const sensor_msgs::ImageConstPtr& msg)
	{
		rightStamp_raw = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			RightRawIamge = cv_ptr->image;
			// showImage(LeftRawIamge, "right");
			if(!trigger_save_raw)
			{
				s_imString = s_root + s_date + "/" + s_trial + "/original/right_raw_" + patch::to_string(++rightraw_count) + ".png"; 
				imwrite(s_imString.c_str(), cv_ptr->image);
				fprintf (pFile,"%s.rightraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), rightraw_count, ros::Time::now().toSec()); // this one needs to be here for plotting purposes
			}
		}//end try
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}//end catch
	} // end void CBr

	void SaveRaw_wait(const std_msgs::EmptyConstPtr& msg)
	{ // Null_msg;
		ROS_INFO("stereoobs: Saving Raw images %i", leftraw_count+1);

		RawSaveTime = ros::Time::now().toSec();
		s_imString = s_root + s_date + "/" + s_trial + "/original/left_raw_" + patch::to_string(++leftraw_count) + ".png"; 
		imwrite(s_imString.c_str(), LeftRawIamge);
		// showImage(LeftRawIamge, "left");
		fprintf (pFile,"%s.leftraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), leftraw_count, RawSaveTime); // this one needs to be here for plotting purposes
		s_imString = s_root + s_date + "/" + s_trial + "/original/right_raw_" + patch::to_string(++rightraw_count) + ".png"; 
		imwrite(s_imString.c_str(), RightRawIamge);
		// showImage(RightRawIamge, "right");
		fprintf (pFile,"%s.rightraw.time(%d,1) = %8.6f;\n", s_handle.c_str(), rightraw_count, RawSaveTime); // this one needs to be here for plotting purposes		
	}

	void leftCvBridge(const sensor_msgs::ImageConstPtr& msg)
	{
		leftStamp = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			if(save_rect)
			{
				s_imString = s_root + s_date + "/" + s_trial + "/original/left_rect_" + patch::to_string(++leftrect_count) + ".png"; 
				imwrite(s_imString.c_str(), cv_ptr->image);
			}
		}
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}//end catch
		left_image=cv_ptr->image;
		newLeft = true;
		// ROS_WARN("stereoobs: newLeft = true;");
		// showImage(left_image, "left_image");
		// calculate(true);  //always run in observation mode
		calculate(); // run with both modes
	} // end void Cbl

	void rightCvBridge(const sensor_msgs::ImageConstPtr& msg)
	{
		rightStamp = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			if(save_rect)
			{
				s_imString = s_root + s_date + "/" + s_trial + "/original/right_rect_" + patch::to_string(++rightrect_count) + ".png"; 
				imwrite(s_imString.c_str(), cv_ptr->image);
			}
		}
		catch (cv_bridge::Exception& e)
		{ROS_ERROR("cv_bridge exception: %s", e.what());return;}
		right_image=cv_ptr->image;
		newRight = true;
		// ROS_INFO("stereoobs: newRight = true;");
		// showImage(right_image, "right_image");
		calculate(); // run with both modes
	} // end void CBr

/* ################## --------- Triangulation Functions --------- ################## */
	void calculate()
	{//begin grab

		double image_delay = abs(rightStamp.toSec() - leftStamp.toSec());
		// ROS_INFO("stereoObs::image_delay = %6.24f", image_delay);

		if(newLeft && newRight)
		{
			stereoflag = true;
			++calledCount;

			// uint numloops = 10;
			// double tstart = ros::Time::now().toSec();
			// ROS_INFO("Start %u x loops", numloops);
			// for (unsigned int i=0; i<numloops; ++i)
			// {

				fprintf (pFile,"\n%s.calltime(%d,1) = %8.6f;\n", s_handle.c_str(), calledCount, tnow);
				// s_imString = s_root + s_date + "/" + s_trial + "/original/left_image.png"; imwrite(s_imString.c_str(), left_image);
				// s_imString = s_root + s_date + "/" + s_trial + "/original/right_image.png"; imwrite(s_imString.c_str(), right_image);

				/*---------images used for drawing detected Circles------------- */
				left_image.copyTo(lCircle);
				right_image.copyTo(rCircle);
				left_image.copyTo(lContours);
				right_image.copyTo(rContours);

				/*---------YUV Circle Detection ------------- */
				cv::GaussianBlur(left_image, left_imageg, cv::Size(Gkx, Gky), GsigmaX, GsigmaY); // blur must be an odd window size
				cv::GaussianBlur(right_image, right_imageg, cv::Size(Gkx, Gky), GsigmaX, GsigmaY); // blur must be an odd window size

				/*---------convert images to YUV ------------- */
				cv::cvtColor(left_imageg, lYUVg, CV_BGR2YUV);
				cv::cvtColor(right_imageg, rYUVg, CV_BGR2YUV);

				std::vector<cv::Mat> lchannels(3);
				std::vector<cv::Mat> rchannels(3);

				cv::split(lYUVg, lchannels);
				cv::split(rYUVg, rchannels);

				lY = lchannels[0];
				lU = lchannels[1];
				lV = lchannels[2];

				rY = rchannels[0];
				rU = rchannels[1];
				rV = rchannels[2];

				isRed();
				isBlue();
				isGreen();


				soloTime = ros::Time::now().toSec();
				if (RedRadii[0] != 0 && RedRadii[1] != 0)
				{
					soloRedCount += 1;
					soloRedP_cam = calcVec(Redpix); 
					soloRedP_ugv = cam2ugv * soloGreenP_cam;

					fprintf (pFile,"%s.Red.solo.Time(%d,1) = %8.6f;\n", s_handle.c_str(), soloRedCount, soloTime);
					fprintf (pFile,"%s.Red.solo.P_cam(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloRedCount, soloRedP_cam(0,0), soloRedP_cam(1,0), soloRedP_cam(2,0));
					fprintf (pFile,"%s.Red.solo.P_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloRedCount, soloRedP_ugv(0,0), soloRedP_ugv(1,0), soloRedP_ugv(2,0));
				}
				if (BlueRadii[0] != 0 && BlueRadii[1] != 0)
				{
					soloBlueCount += 1;
					soloBlueP_cam = calcVec(Bluepix); 
					soloBlueP_ugv = cam2ugv * soloBlueP_cam;

					fprintf (pFile,"%s.Blue.solo.Time(%d,1) = %8.6f;\n", s_handle.c_str(), soloBlueCount, soloTime);
					fprintf (pFile,"%s.Blue.solo.P_cam(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloBlueCount, soloBlueP_cam(0,0), soloBlueP_cam(1,0), soloBlueP_cam(2,0));
					fprintf (pFile,"%s.Blue.solo.P_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloBlueCount, soloBlueP_ugv(0,0), soloBlueP_ugv(1,0), soloBlueP_ugv(2,0));
				}
				if (GreenRadii[0] != 0 && GreenRadii[1] != 0)
				{
					soloGreenCount += 1;
					soloGreenP_cam = calcVec(Greenpix); 
					soloGreenP_ugv = cam2ugv * soloGreenP_cam;

					fprintf (pFile,"%s.Green.solo.Time(%d,1) = %8.6f;\n", s_handle.c_str(), soloGreenCount, soloTime);
					fprintf (pFile,"%s.Green.solo.P_cam(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloGreenCount, soloGreenP_cam(0,0), soloGreenP_cam(1,0), soloGreenP_cam(2,0));
					fprintf (pFile,"%s.Green.solo.P_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), soloGreenCount, soloGreenP_ugv(0,0), soloGreenP_ugv(1,0), soloGreenP_ugv(2,0));
				}





				// try to determine how long it takes for these routines to run: this block runs at about 1kHz
				// uint numloops = 1000;
				// double tstart = ros::Time::now().toSec();
				// ROS_INFO("Start %u x loops", numloops);
				// for (unsigned int i=0; i<numloops; ++i)
				// {
					// fprintf (pFile,"%% Test RawRedpix = [%6.2f %6.2f %6.2f %6.2f ];\n", RawRedpix[0], RawRedpix[1], RawRedpix[2], RawRedpix[3]);
					// fprintf (pFile,"%% Test RawBluepix = [%6.2f %6.2f %6.2f %6.2f ];\n", RawBluepix[0], RawBluepix[1], RawBluepix[2], RawBluepix[3]);
					// fprintf (pFile,"%% Test RawGreenpix = [%6.2f %6.2f %6.2f %6.2f ];\n", RawGreenpix[0], RawGreenpix[1], RawGreenpix[2], RawGreenpix[3]);

			// if (((RawRedpix[0] == 0) && (RawRedpix[1] == 0)) || ((RawRedpix[2] == 0) && (RawRedpix[3] == 0))) // then there is no pre-existing pixel to center for the roi for one of the images
			// 	{ isRed(); } else { roiIsRed(RawRedpix);}
			// if (((RawBluepix[0] == 0) && (RawBluepix[1] == 0)) || ((RawBluepix[2] == 0) && (RawBluepix[3] == 0))) // then there is no pre-existing pixel to center for the roi for one of the images
			// 	{ isBlue(); } else { roiIsBlue(RawBluepix); }
			// if (((RawGreenpix[0] == 0) && (RawGreenpix[1] == 0)) || ((RawGreenpix[2] == 0) && (RawGreenpix[3] == 0))) // then there is no pre-existing pixel to center for the roi for one of the images
			// 	{ isGreen();} else { roiIsGreen(RawGreenpix);}

				// }//for (unsigned int i=0; i<numloops; i++)
				// double tend = ros::Time::now().toSec();
				// ROS_INFO("Finished %u x loops in %6.14f seconds", numloops, tend - tstart);

					
				// fprintf (pFile,"%s.Red.left.count.radii(%d)  = %u;\n", s_handle.c_str(), calledCount,   int(lRedKeypoint.size()));
				// fprintf (pFile,"%s.Red.right.count.radii(%d) = %u;\n", s_handle.c_str(), calledCount,   int(rRedKeypoint.size()));
				// fprintf (pFile,"%s.Blue.left.count.radii(%d)  = %u;\n", s_handle.c_str(), calledCount,  int(lBlueKeypoint.size()));
				// fprintf (pFile,"%s.Blue.right.count.radii(%d) = %u;\n", s_handle.c_str(), calledCount,  int(rBlueKeypoint.size()));
				// fprintf (pFile,"%s.Green.left.count.radii(%d)  = %u;\n", s_handle.c_str(), calledCount, int(lGreenKeypoint.size()));
				// fprintf (pFile,"%s.Green.right.count.radii(%d) = %u;\n", s_handle.c_str(), calledCount, int(rGreenKeypoint.size()));


				// fprintf (pFile,"%s.Red.keypoints.i(%d,:) = %u;\n", s_handle.c_str(), calledCount, kpl.size());

					if (showcircles)
					{
						showImage(lCircle, "lCircle");
						showImage(rCircle, "rCircle");
					}

					if (showYUV)
					{
						showImage(lU, "lU");
						showImage(rU, "rU");
						showImage(lV, "lV");
						showImage(rV, "rV");
					}



				// showImage(leftRedLEDclosed, "leftRedLEDclosed");
				// showImage(rightRedLEDclosed, "rightRedLEDclosed");
				// showImage(leftBlueLEDclosed, "leftBlueLEDclosed_");
				// showImage(rightBlueLEDclosed, "rightBlueLEDclosed_");
				// showImage(leftGreenLEDclosed, "leftGreenLEDclosed_");
				// showImage(rightGreenLEDclosed, "rightGreenLEDclosed_");


				if (SaveImages)
				{
					s_imString = s_root + s_date + "/" + s_trial + "/original/left_image_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), left_image);
					s_imString = s_root + s_date + "/" + s_trial + "/original/right_image_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), right_image);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/left_image_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), lCircle);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/right_image_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rCircle);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/lU_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), lU);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/lV_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), lV);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/rU_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rU);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/rV_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rV);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/leftRedLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftRedLEDclosed);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/rightRedLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightRedLEDclosed);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/leftBlueLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftBlueLEDclosed);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/rightBlueLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightBlueLEDclosed);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/leftGreenLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftGreenLEDclosed);
					s_imString = s_root + s_date + "/" + s_trial + "/circles/rightGreenLEDclosed_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightGreenLEDclosed);
				}


				/*--------- Calculate uav Pose ------------- */
				nowStamp = ros::Time::now();
				tnow = nowStamp.toSec();
				if (stereoflag)
				{
					++goodloops;
						fprintf (pFile,"\n%s.time(%d,1) = %8.6f;\n", s_handle.c_str(), goodloops, tnow); // this one needs to be here for plotting purposes

					// uint numloops = 1000;
					// double tstart = ros::Time::now().toSec();
					// ROS_INFO("Start %u x loops", numloops);
					// for (unsigned int i=0; i<numloops; ++i)
					// {

						calcXYZ();
						crossOrientation();

					// }//for (unsigned int i=0; i<numloops; i++)
					// double tend = ros::Time::now().toSec();
					// ROS_INFO("Finished %u x loops in %6.14f seconds", numloops, tend - tstart);

					if (good_axes)
					{			


						/*--------- Compose Stereo Publication ------------- */
						Pose_msg.P.x = UAV_p_ugv(0,0);
						Pose_msg.P.y = UAV_p_ugv(1,0);
						Pose_msg.P.z = UAV_p_ugv(2,0);
						
						Pose_msg.PCov.row0.x = UAV_p_cov(0,0);
						Pose_msg.PCov.row0.y = UAV_p_cov(0,1);
						Pose_msg.PCov.row0.z = UAV_p_cov(0,2);
						Pose_msg.PCov.row1.x = UAV_p_cov(1,0);
						Pose_msg.PCov.row1.y = UAV_p_cov(1,1);
						Pose_msg.PCov.row1.z = UAV_p_cov(1,2);
						Pose_msg.PCov.row2.x = UAV_p_cov(2,0);
						Pose_msg.PCov.row2.y = UAV_p_cov(2,1);
						Pose_msg.PCov.row2.z = UAV_p_cov(2,2);

						Pose_msg.R.row0.x = x_axis(0,0);
						Pose_msg.R.row0.y = x_axis(1,0);
						Pose_msg.R.row0.z = x_axis(2,0);
						Pose_msg.R.row1.x = y_axis(0,0);
						Pose_msg.R.row1.y = y_axis(1,0);
						Pose_msg.R.row1.z = y_axis(2,0);
						Pose_msg.R.row2.x = z_axis(0,0);
						Pose_msg.R.row2.y = z_axis(1,0);
						Pose_msg.R.row2.z = z_axis(2,0);
						Pose_msg.RCov.row0.x = x_JJt(0,0);
						Pose_msg.RCov.row0.y = x_JJt(0,1);
						Pose_msg.RCov.row0.z = x_JJt(0,2);
						Pose_msg.RCov.row1.x = x_JJt(1,0);
						Pose_msg.RCov.row1.y = x_JJt(1,1);
						Pose_msg.RCov.row1.z = x_JJt(1,2);
						Pose_msg.RCov.row2.x = x_JJt(2,0);
						Pose_msg.RCov.row2.y = x_JJt(2,1);
						Pose_msg.RCov.row2.z = x_JJt(2,2);

						Pose_msg.yaw = yaw_ugv; //atan2(x_axis(0, 1), x_axis(0, 0)) * 180 / Pi; //heading angle in degrees in tb frame
						Pose_msg.yaw_cov = UAV_yaw_var;

						Pose_msg.stamp = tnow;

						// ROS_INFO("stereoobs: -------Pose_msg --------------------------");
						// ROS_INFO("stereoobs: UAV_p_ugv  = [% -6.8f % -6.8f % -6.8f]  ", UAV_p_ugv(0,0), UAV_p_ugv(1,0), UAV_p_ugv(2,0));
						// ROS_INFO("stereoobs: [00 01 02] = [% -6.8f % -6.8f % -6.8f]  ", Pose_msg.R.row0.x, Pose_msg.R.row0.y, Pose_msg.R.row0.z);
						// ROS_INFO("stereoobs: [10 11 12] = [% -6.8f % -6.8f % -6.8f]  ", Pose_msg.R.row1.x, Pose_msg.R.row1.y, Pose_msg.R.row1.z);
						// ROS_INFO("stereoobs: [20 21 22] = [% -6.8f % -6.8f % -6.8f]  ", Pose_msg.R.row2.x, Pose_msg.R.row2.y, Pose_msg.R.row2.z);

						uav_TF.setOrigin( tf::Vector3(UAV_p_ugv(0,0), UAV_p_ugv(1,0), UAV_p_ugv(2,0)) );
						uavR_TF.setValue(Pose_msg.R.row0.x, Pose_msg.R.row1.x, Pose_msg.R.row2.x, 
										 Pose_msg.R.row0.y, Pose_msg.R.row1.y, Pose_msg.R.row2.y, 
										 Pose_msg.R.row0.z, Pose_msg.R.row1.z, Pose_msg.R.row2.z);
						// uavR_TF.setValue(1, 0, 0, 
						// 				 0, 1, 0, 
						// 				 0, 0, 1);

						uavR_TF.getRotation(uavQ_TF);
						uav_TF.setRotation(uavQ_TF);

						Pose_msg.red.xr = Redpix[0];
						Pose_msg.red.yr = Redpix[1];
						Pose_msg.red.xl = Redpix[2];
						Pose_msg.red.yl = Redpix[3];

						Pose_msg.blue.xr = Bluepix[0];
						Pose_msg.blue.yr = Bluepix[1];
						Pose_msg.blue.xl = Bluepix[2];
						Pose_msg.blue.yl = Bluepix[3];

						Pose_msg.green.xr = Greenpix[0];
						Pose_msg.green.yr = Greenpix[1];
						Pose_msg.green.xl = Greenpix[2];
						Pose_msg.green.yl = Greenpix[3];

						Pose_msg.id = ++Pose_msg.id;

						// publish message and tf
						uav_TF_pub.sendTransform(tf::StampedTransform(uav_TF, nowStamp, s_uav_TF_parent, s_uav_TF_child));
						Pose_pub.publish(Pose_msg);

						pcl::toROSMsg(*stereoCloudCAM_ptr, stereoCloudCAM_msg);
						stereoCloudCAM_msg.header.frame_id = s_cam_cloud_frameID;
						stereoCloudCAM_pub.publish(stereoCloudCAM_msg);

						pcl::toROSMsg(*stereoCloudUGV_ptr, stereoCloudUGV_msg);
						stereoCloudUGV_msg.header.frame_id = s_ugv_cloud_frameID;
						stereoCloudUGV_pub.publish(stereoCloudUGV_msg);


					}// end if(good_axes)
					else
					{
						// saveBadAxes();
					}
				}// end if (stereoflag)
				else
				{
					++badloops;
					if (SaveImages)
						{
							// saveBadImages();
						}
				}

			// }//for (unsigned int i=0; i<numloops; i++)
			// double tend = ros::Time::now().toSec();
			// ROS_INFO("Finished %u x loops in %6.14f seconds", numloops, tend - tstart);

			newLeft = false;
			newRight = false;
		} // end if(new images)
	} // end bool grab

	void calcXYZ()
	{
		/* Calling this function will update the member variables that
		 * Calculate the location of the target with respect to the focal
		 * point of the left camera (the local coordinate frame origin)
		 * Z = (b f)/ d  = ([in][px])/[px] = [in]
		 * X = Z (xl + xr) / (2 f) = ([in][px]) / ([px]) = [in]
		 * Y = Z (yl + yr) / (2 f) = ([in][px]) / ([px]) = [in]
		 * pix[0 1 2 3] = pix[rx ry lx ly]
		 */

		/* ------ Pixels to 3D ---------*/
			stereoCloudCAM_ptr->points.clear();
			RedP_cam = calcVec(Redpix); //Redscale = RedP_cam(2,0) * RedP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
			BlueP_cam = calcVec(Bluepix); //Bluescale = BlueP_cam(2,0) * BlueP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
			GreenP_cam = calcVec(Greenpix); //Greenscale = GreenP_cam(2,0) * GreenP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
			UAV_p_cam = 0.5 * (BlueP_cam + RedP_cam);
			camCloudPt.x = RedP_cam(0,0); camCloudPt.y = RedP_cam(1,0); camCloudPt.z = RedP_cam(2,0); stereoCloudCAM_ptr->points.push_back(camCloudPt);
			camCloudPt.x = BlueP_cam(0,0); camCloudPt.y = BlueP_cam(1,0); camCloudPt.z = BlueP_cam(2,0); stereoCloudCAM_ptr->points.push_back(camCloudPt);
			camCloudPt.x = GreenP_cam(0,0); camCloudPt.y = GreenP_cam(1,0); camCloudPt.z = GreenP_cam(2,0); stereoCloudCAM_ptr->points.push_back(camCloudPt);

			RedJ_cam = calcJ(Redpix);
			BlueJ_cam = calcJ(Bluepix);
			GreenJ_cam = calcJ(Greenpix);

			/* rotate to ugv frame ---------*/
			stereoCloudUGV_ptr->points.clear();
			RedP_ugv   = cam2ugv * RedP_cam;
			BlueP_ugv  = cam2ugv * BlueP_cam;
			GreenP_ugv = cam2ugv * GreenP_cam;
			UAV_p_ugv  = cam2ugv * UAV_p_cam;
			ugvCloudPt.x = RedP_ugv(0,0); ugvCloudPt.y = RedP_ugv(1,0); ugvCloudPt.z = RedP_ugv(2,0); stereoCloudUGV_ptr->points.push_back(ugvCloudPt);
			ugvCloudPt.x = BlueP_ugv(0,0); ugvCloudPt.y = BlueP_ugv(1,0); ugvCloudPt.z = BlueP_ugv(2,0); stereoCloudUGV_ptr->points.push_back(ugvCloudPt);
			ugvCloudPt.x = GreenP_ugv(0,0); ugvCloudPt.y = GreenP_ugv(1,0); ugvCloudPt.z = GreenP_ugv(2,0); stereoCloudUGV_ptr->points.push_back(ugvCloudPt);
			ugvCloudPt.x = UAV_p_ugv(0,0); ugvCloudPt.y = UAV_p_ugv(1,0); ugvCloudPt.z = UAV_p_ugv(2,0); stereoCloudUGV_ptr->points.push_back(ugvCloudPt);

			RedJJt_cam = RedJ_cam * Redscale * RedJ_cam.t();
			BlueJJt_cam = BlueJ_cam * Bluescale * BlueJ_cam.t();
			GreenJJt_cam = GreenJ_cam * Greenscale * GreenJ_cam.t();

			RedJJt_ugv = cam2ugv33 * RedJ_cam * Redscale * RedJ_cam.t() * cam2ugv33.t();
			BlueJJt_ugv = cam2ugv33 * BlueJ_cam * Bluescale * BlueJ_cam.t() * cam2ugv33.t();
			GreenJJt_ugv = cam2ugv33 * GreenJ_cam * Greenscale * GreenJ_cam.t() * cam2ugv33.t();

			fprintf (pFile,"%s.Red.right.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Redpix[0], Redpix[1]);
			fprintf (pFile,"%s.Red.right.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawRedpix[0], RawRedpix[1]);
			fprintf (pFile,"%s.Red.right.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, RedRadii[0]);
			fprintf (pFile,"%s.Red.left.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Redpix[2], Redpix[3]);
			fprintf (pFile,"%s.Red.left.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawRedpix[2], RawRedpix[3]);
			fprintf (pFile,"%s.Red.left.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, RedRadii[1]);
			fprintf (pFile,"%s.Red.left.disp(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, Redpix[2] - Redpix[0]);
			fprintf (pFile,"%s.Blue.right.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Bluepix[0], Bluepix[1]);
			fprintf (pFile,"%s.Blue.right.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawBluepix[0], RawBluepix[1]);
			fprintf (pFile,"%s.Blue.right.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, BlueRadii[0]);
			fprintf (pFile,"%s.Blue.left.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Bluepix[2], Bluepix[3]);
			fprintf (pFile,"%s.Blue.left.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawBluepix[2], RawBluepix[3]);
			fprintf (pFile,"%s.Blue.left.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, BlueRadii[1]);
			fprintf (pFile,"%s.Blue.left.disp(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, Bluepix[2] - Bluepix[0]);
			fprintf (pFile,"%s.Green.right.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Greenpix[0], Greenpix[1]);
			fprintf (pFile,"%s.Green.right.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawGreenpix[0], RawGreenpix[1]);
			fprintf (pFile,"%s.Green.right.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, GreenRadii[0]);
			fprintf (pFile,"%s.Green.left.xy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, Greenpix[2], Greenpix[3]);
			fprintf (pFile,"%s.Green.left.rawxy(%d,:) = [% -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RawGreenpix[2], RawGreenpix[3]);
			fprintf (pFile,"%s.Green.left.radius(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, GreenRadii[1]);
			fprintf (pFile,"%s.Green.left.disp(%d,:) = % -6.8f;\n", s_handle.c_str(), goodloops, Greenpix[2] - Greenpix[0]);			
			fprintf (pFile,"%s.Red.P_cam(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedP_cam(0,0)  , RedP_cam(1,0)  , RedP_cam(2,0));
			fprintf (pFile,"%s.Blue.P_cam(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueP_cam(0,0) , BlueP_cam(1,0) , BlueP_cam(2,0));
			fprintf (pFile,"%s.Green.P_cam(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenP_cam(0,0), GreenP_cam(1,0), GreenP_cam(2,0));
			fprintf (pFile,"%s.uav.Obs_cam(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_cam(0,0) , UAV_p_cam(1,0) , UAV_p_cam(2,0));
			fprintf (pFile,"%s.Red.P_ugv(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedP_ugv(0,0)   , RedP_ugv(1,0)  , RedP_ugv(2,0));
			fprintf (pFile,"%s.Blue.P_ugv(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueP_ugv(0,0)  , BlueP_ugv(1,0) , BlueP_ugv(2,0));
			fprintf (pFile,"%s.Green.P_ugv(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenP_ugv(0,0) , GreenP_ugv(1,0)  , GreenP_ugv(2,0));
			fprintf (pFile,"%s.uav.P_ugv(%d,:)   = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_ugv(0,0) , UAV_p_ugv(1,0)  , UAV_p_ugv(2,0));

			// ROS_INFO("%s.uav.P_ugv(%d,:) = [% -6.8f, % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_ugv(0,0) , UAV_p_ugv(1,0)  , UAV_p_ugv(2,0));
	}//end void calcXYZ

	void crossOrientation()
	{
		// z_axis = (Pg - Pb) X (Pr - Pb) / || (Pg - Pb) X (Pr - Pb) ||
		// y_axis = (Pb - Pr) / || (Pb - Pr) ||
		// x_axis = y_axis X z_axis

		/* ------- Compute Axes and Yaw----------- */
		BmR = cv::Matx41d(BlueP_ugv(0,0) - RedP_ugv(0,0), BlueP_ugv(1,0) - RedP_ugv(1,0), BlueP_ugv(2,0) - RedP_ugv(2,0), 0);
		GmB = cv::Matx41d(GreenP_ugv(0,0) - BlueP_ugv(0,0), GreenP_ugv(1,0) - BlueP_ugv(1,0), GreenP_ugv(2,0) - BlueP_ugv(2,0), 0);
		RmG = cv::Matx41d(RedP_ugv(0,0) - GreenP_ugv(0,0), RedP_ugv(1,0) - GreenP_ugv(1,0), RedP_ugv(2,0) - GreenP_ugv(2,0), 0);
		RmB = cv::Matx41d(RedP_ugv(0,0) - BlueP_ugv(0,0), RedP_ugv(1,0) - BlueP_ugv(1,0), RedP_ugv(2,0) - BlueP_ugv(2,0), 0);

		znum = Cross(GmB, RmB);
		double normz = norm(znum, L2Norm);
		double normy = norm(BmR, L2Norm);
		z_axis = (1.0 / normz) * znum;
		y_axis = (1.0 / normy) * BmR;
		x_axis = Cross(y_axis, z_axis);

		if (z_axis(0, 2)<0.8)
			{
				good_axes=false; //up is not up
				++bad_axes_count;
				fprintf (pFile,"%s.badAxisCount(%d,1) = %u;\n", s_handle.c_str(), bad_axes_count, bad_axes_count);
				ROS_WARN("Bad Axis (%d) z_axis(0,2) = %6.4f", bad_axes_count, z_axis(0, 2));

			} else {
				good_axes=true; // reset flag
			}

		yaw_ugv = atan2(x_axis(0, 1), x_axis(0, 0)) * 180 / Pi; //heading angle in degrees in ugv frame

		/* ------- Propegate Covariances ----------- */
		cv::Matx<double,3,3> z_jacobianOfNorm, y_jacobianOfNorm;
		cv::Matx<double,3,3> GandB_JJt = GreenJJt_ugv + BlueJJt_ugv;
		cv::Matx<double,3,3> RandB_JJt = RedJJt_ugv + BlueJJt_ugv;
		UAV_p_cov = 0.5 * RandB_JJt * 0.5; // scaled both Red and Blue vectors by 1/2

		z_JJt = wedgevector(BmR) * GandB_JJt * wedgevector(BmR).t() + wedgevector(GmB) * RandB_JJt * wedgevector(GmB).t();
		z_jacobianOfNorm = jacobianOfNorm(znum);
		z_JJt = z_jacobianOfNorm * z_JJt * z_jacobianOfNorm.t();

		y_jacobianOfNorm = jacobianOfNorm(BmR);
		y_JJt = y_jacobianOfNorm * RandB_JJt * y_jacobianOfNorm.t();

		x_JJt = wedgevector(-z_axis) * y_JJt * wedgevector(-z_axis).t() + wedgevector(y_axis) * z_JJt * wedgevector(y_axis).t();
		cv::Matx<double,2,2> x_JJt2x2 = cv::Matx22d(
			x_JJt(0,0), x_JJt(0,1),
			x_JJt(1,0), x_JJt(1,1));

		cv::Matx<double,1,2> x_jacobianOfAtan2 = jacobianOfAtan2(x_axis(0, 1),x_axis(0, 0));
		cv::Matx<double,1,1> yaw_JJt = x_jacobianOfAtan2 * x_JJt2x2 * x_jacobianOfAtan2.t();
		UAV_yaw_var = yaw_JJt(0,0);

		// if (good_axes)
		// {		
			fprintf (pFile,"%s.Red.J_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJ_cam(0,0), RedJ_cam(0,1), RedJ_cam(0,2));
			fprintf (pFile,"%s.Red.J_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJ_cam(1,0), RedJ_cam(1,1), RedJ_cam(1,2));
			fprintf (pFile,"%s.Red.J_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJ_cam(2,0), RedJ_cam(2,1), RedJ_cam(2,2));
			fprintf (pFile,"%s.Blue.J_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJ_cam(0,0), BlueJ_cam(0,1), BlueJ_cam(0,2));
			fprintf (pFile,"%s.Blue.J_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJ_cam(1,0), BlueJ_cam(1,1), BlueJ_cam(1,2));
			fprintf (pFile,"%s.Blue.J_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJ_cam(2,0), BlueJ_cam(2,1), BlueJ_cam(2,2));
			fprintf (pFile,"%s.Green.J_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJ_cam(0,0), GreenJ_cam(0,1), GreenJ_cam(0,2));
			fprintf (pFile,"%s.Green.J_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJ_cam(1,0), GreenJ_cam(1,1), GreenJ_cam(1,2));
			fprintf (pFile,"%s.Green.J_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJ_cam(2,0), GreenJ_cam(2,1), GreenJ_cam(2,2));
			fprintf (pFile,"%s.Red.JJt_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_cam(0,0), RedJJt_cam(0,1), RedJJt_cam(0,2));
			fprintf (pFile,"%s.Red.JJt_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_cam(1,0), RedJJt_cam(1,1), RedJJt_cam(1,2));
			fprintf (pFile,"%s.Red.JJt_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_cam(2,0), RedJJt_cam(2,1), RedJJt_cam(2,2));
			fprintf (pFile,"%s.Blue.JJt_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_cam(0,0), BlueJJt_cam(0,1), BlueJJt_cam(0,2));
			fprintf (pFile,"%s.Blue.JJt_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_cam(1,0), BlueJJt_cam(1,1), BlueJJt_cam(1,2));
			fprintf (pFile,"%s.Blue.JJt_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_cam(2,0), BlueJJt_cam(2,1), BlueJJt_cam(2,2));
			fprintf (pFile,"%s.Green.JJt_cam(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_cam(0,0), GreenJJt_cam(0,1), GreenJJt_cam(0,2));
			fprintf (pFile,"%s.Green.JJt_cam(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_cam(1,0), GreenJJt_cam(1,1), GreenJJt_cam(1,2));
			fprintf (pFile,"%s.Green.JJt_cam(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_cam(2,0), GreenJJt_cam(2,1), GreenJJt_cam(2,2));

			fprintf (pFile,"%s.Red.JJt_ugv(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_ugv(0,0), RedJJt_ugv(0,1), RedJJt_ugv(0,2));
			fprintf (pFile,"%s.Red.JJt_ugv(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_ugv(1,0), RedJJt_ugv(1,1), RedJJt_ugv(1,2));
			fprintf (pFile,"%s.Red.JJt_ugv(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, RedJJt_ugv(2,0), RedJJt_ugv(2,1), RedJJt_ugv(2,2));
			fprintf (pFile,"%s.Blue.JJt_ugv(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_ugv(0,0), BlueJJt_ugv(0,1), BlueJJt_ugv(0,2));
			fprintf (pFile,"%s.Blue.JJt_ugv(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_ugv(1,0), BlueJJt_ugv(1,1), BlueJJt_ugv(1,2));
			fprintf (pFile,"%s.Blue.JJt_ugv(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, BlueJJt_ugv(2,0), BlueJJt_ugv(2,1), BlueJJt_ugv(2,2));
			fprintf (pFile,"%s.Green.JJt_ugv(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_ugv(0,0), GreenJJt_ugv(0,1), GreenJJt_ugv(0,2));
			fprintf (pFile,"%s.Green.JJt_ugv(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_ugv(1,0), GreenJJt_ugv(1,1), GreenJJt_ugv(1,2));
			fprintf (pFile,"%s.Green.JJt_ugv(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, GreenJJt_ugv(2,0), GreenJJt_ugv(2,1), GreenJJt_ugv(2,2));

			fprintf (pFile,"%s.uav.p_cov(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_cov(0,0), UAV_p_cov(0,1), UAV_p_cov(0,2));
			fprintf (pFile,"%s.uav.p_cov(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_cov(1,0), UAV_p_cov(1,1), UAV_p_cov(1,2));
			fprintf (pFile,"%s.uav.p_cov(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, UAV_p_cov(2,0), UAV_p_cov(2,1), UAV_p_cov(2,2));

			fprintf (pFile,"%s.uav.z_axis(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, z_axis(0,0), z_axis(1,0), z_axis(2,0));
			fprintf (pFile,"%s.uav.z_JJt(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, z_JJt(0,0), z_JJt(0,1), z_JJt(0,2));
			fprintf (pFile,"%s.uav.z_JJt(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, z_JJt(1,0), z_JJt(1,1), z_JJt(1,2));
			fprintf (pFile,"%s.uav.z_JJt(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, z_JJt(2,0), z_JJt(2,1), z_JJt(2,2));

			fprintf (pFile,"%s.uav.y_axis(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, y_axis(0,0), y_axis(1,0), y_axis(2,0));
			fprintf (pFile,"%s.uav.y_JJt(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, y_JJt(0,0), y_JJt(0,1), y_JJt(0,2));
			fprintf (pFile,"%s.uav.y_JJt(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, y_JJt(1,0), y_JJt(1,1), y_JJt(1,2));
			fprintf (pFile,"%s.uav.y_JJt(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, y_JJt(2,0), y_JJt(2,1), y_JJt(2,2));

			fprintf (pFile,"%s.uav.x_axis(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, x_axis(0,0), x_axis(1,0), x_axis(2,0));
			fprintf (pFile,"%s.uav.x_JJt(1,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, x_JJt(0,0), x_JJt(0,1), x_JJt(0,2));
			fprintf (pFile,"%s.uav.x_JJt(2,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, x_JJt(1,0), x_JJt(1,1), x_JJt(1,2));
			fprintf (pFile,"%s.uav.x_JJt(3,:,%d) = [% -6.8f, % -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, x_JJt(2,0), x_JJt(2,1), x_JJt(2,2));

			fprintf (pFile,"%s.uav.yaw_ugv(%d,1) = % -6.8f;\n", s_handle.c_str(), goodloops, yaw_ugv);
			fprintf (pFile,"%s.uav.yaw_jacobianOfAtan2(%d,:) = [% -6.8f, % -6.8f];\n", s_handle.c_str(), goodloops, x_jacobianOfAtan2(0,0), x_jacobianOfAtan2(0,1));
			fprintf (pFile,"%s.uav.UAV_yaw_var(%d,1) = % -6.8f;\n", s_handle.c_str(), goodloops, UAV_yaw_var);

			fprintf (pFile,"%s.uav.GmB(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, GmB(0,0), GmB(1,0), GmB(2,0));
			fprintf (pFile,"%s.uav.BmR(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, BmR(0,0), BmR(1,0), BmR(2,0));
			fprintf (pFile,"%s.uav.RmG(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", s_handle.c_str(), goodloops, RmG(0,0), RmG(1,0), RmG(2,0));
	}

/* ################## --------- Image Tool Functions --------- ################## */

	void showImage(cv::Mat image, const char* name)
	{
		 cv::namedWindow(name, 1);
		 cv::imshow(name, image);
		 cv::waitKey(5);
	}

	void saveImage(cv::Mat image, const char* name)
	{
		std::string s_savename;
		s_savename =  s_root + s_date + "/" + name + ".png";
		imwrite(s_savename.c_str(), image);//, compression_params);
	}

	void saveBadAxes()
	{
		s_imString = s_root + s_date + "/" + s_trial + "/circles/CirclesLeft_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), lCircle);
		s_imString = s_root + s_date + "/" + s_trial + "/circles/CirclesRight_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rCircle);

		s_imString = s_root + s_date + "/" + s_trial + "/circles/RedClosedLeft_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftRedLEDclosed);
		s_imString = s_root + s_date + "/" + s_trial + "/circles/RedClosedRight_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightRedLEDclosed);

		s_imString = s_root + s_date + "/" + s_trial + "/circles/BlueClosedLeft_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftBlueLEDclosed);
		s_imString = s_root + s_date + "/" + s_trial + "/circles/BlueClosedRight_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightBlueLEDclosed);

		s_imString = s_root + s_date + "/" + s_trial + "/circles/GreenClosedLeft_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), leftGreenLEDclosed);
		s_imString = s_root + s_date + "/" + s_trial + "/circles/GreenClosedRight_" + patch::to_string(calledCount) + ".png"; imwrite(s_imString.c_str(), rightGreenLEDclosed);
	}

	void roiIsRed(cv::Vec<double,4> Pix)
	{
		// fprintf (pFile,"%%%s.roiIsRed.calledCount = %d;\n", s_handle.c_str(), calledCount);
		/*
		 *  Pix := 4x1 vector
		 *  pix[0 1 2 3] = pix[rx ry lx ly]
		 *  xr = Pix[0]
		 *  yr = Pix[1]
		 *  xl = Pix[2]
		 *  yl = Pix[3]
		 */
		int rx = Pix[0];
		int ry = Pix[1];
		int lx = Pix[2];
		int ly = Pix[3];
		// fprintf (pFile,"%s.roiIsRed.left_center(%d,:) = [%d %d ];\n", s_handle.c_str(), calledCount, lx, ly);
		// fprintf (pFile,"%s.roiIsRed.right_center(%d,:) = [%d %d];\n", s_handle.c_str(), calledCount, rx, ry);

		// ROS_WARN("image_width image_height  = [%d %d ]", image_width, image_height);
		// ROS_WARN("Red [lx ly : rx ry] = [%d %d : %d %d ]", lx, ly, rx, ry);

		cv::Mat LeftisRed_Mat, LeftisRed_U, LeftisRed_V;
		cv::Mat RightisRed_Mat, RightisRed_U, RightisRed_V;

		int lx_low_offset, ly_low_offset;
		int rx_low_offset, ry_low_offset;
		int lx_high_offset, ly_high_offset;
		int rx_high_offset, ry_high_offset;

		// prevent roi from defining a matrix outside the dimensions of the original
		int lx_low  = lx - roi_halfwindow; if(lx_low < 0) { lx_low_offset = lx_low; lx_low = 0; } else { lx_low_offset = 0; } // then the pixel is within roi_halfwindow of the edge, so lx is set to zero because that's the left corner
		int rx_low  = rx - roi_halfwindow; if(rx_low < 0) { rx_low_offset = rx_low; rx_low = 0; } else { rx_low_offset = 0; }
		int ly_low  = ly - roi_halfwindow; if(ly_low < 0) { ly_low_offset = ly_low; ly_low = 0; } else { ly_low_offset = 0; } 
		int ry_low  = ry - roi_halfwindow; if(ry_low < 0) { ry_low_offset = ry_low; ry_low = 0; } else { ry_low_offset = 0; }

		int lx_high = lx + roi_halfwindow; if(lx_high > image_width)  { lx_high_offset = lx_high - image_width; } else { lx_high_offset = 0; }
		int rx_high = rx + roi_halfwindow; if(rx_high > image_width)  { rx_high_offset = rx_high - image_width; } else { rx_high_offset = 0; }
		int ly_high = ly + roi_halfwindow; if(ly_high > image_height) { ly_high_offset = ly_high - image_height;} else { ly_high_offset = 0; }
		int ry_high = ry + roi_halfwindow; if(ry_high > image_height) { ry_high_offset = ry_high - image_height;} else { ry_high_offset = 0; }

		// ROS_WARN("Red [lx ly : rx ry] = [%d %d : %d %d ]", lx, ly, rx, ry);
		// ROS_WARN("Red [lx low : rx low] = [%d : %d ]", lx_low, rx_low);
		// ROS_WARN("Red [ly low : ry low] = [%d : %d ]", ly_low, ry_low);

		/*--- Select ROI of channled images --- */
		cv::Mat lU_roi = lU( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat lV_roi = lV( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat rU_roi = rU( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		cv::Mat rV_roi = rV( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		// fprintf (pFile,"%s.roiIsRed.left_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, lx_low, ly_low, (roi_halfwindow - lx_high_offset), (roi_halfwindow - ly_high_offset) );
		// fprintf (pFile,"%s.roiIsRed.right_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, rx_low, ry_low, (roi_halfwindow - rx_high_offset), (roi_halfwindow - ry_high_offset) );

		/*--- Filter by Color --- */
		cv::inRange( lU_roi, Red_u_low, Red_u_high, LeftisRed_U);
		cv::inRange( rU_roi, Red_u_low, Red_u_high, RightisRed_U);

		cv::inRange( lV_roi, Red_v_low, Red_v_high, LeftisRed_V);
		cv::inRange( rV_roi, Red_v_low, Red_v_high, RightisRed_V);

		cv::bitwise_and(LeftisRed_U, LeftisRed_V, LeftisRed_Mat);
		cv::bitwise_and(RightisRed_U, RightisRed_V, RightisRed_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisRed_Mat, leftRedLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisRed_Mat, rightRedLEDclosed, 2, closingElement );
		/*--- Find Contours --- */
		leftRedContours = getContours(leftRedLEDclosed);
		rightRedContours = getContours(rightRedLEDclosed);
		/*--- Find Polygons --- */
		lRedPolygons = getPolygons(leftRedContours);
		rRedPolygons = getPolygons(rightRedContours);
		/*--- Find Minimum Enclosing circles --- */
		std::vector<cv::KeyPoint> lRoiKeypoint, rRoiKeypoint;
		lRoiKeypoint = getCircles(lRedPolygons);
		rRoiKeypoint = getCircles(rRedPolygons);



		/*--- Draw Results --- */
		lRedKeypoint = remapROIKepoints(lRoiKeypoint, lx_low, ly_low);
		rRedKeypoint = remapROIKepoints(rRoiKeypoint, rx_low, ry_low);
		drawCircles(lRedKeypoint,lCircle,RGBRed);
		drawCircles(rRedKeypoint,rCircle,RGBRed);
		cv::rectangle(lCircle, cv::Point2f(lx_low, ly_low), cv::Point2f(lx_high - lx_high_offset, ly_high - ly_high_offset), RGBRed);
		cv::rectangle(rCircle, cv::Point2f(rx_low, ry_low), cv::Point2f(rx_high - rx_high_offset, ry_high - ry_high_offset), RGBRed);
		// drawCircles(lRoiKeypoint,leftRedLEDclosed,RGBRed);
		// drawCircles(rRoiKeypoint,rightRedLEDclosed,RGBRed);


		/*--- Populate Pixel Array Results --- */
		Redpix    = testRadii(rRedKeypoint,   lRedKeypoint,   Redstr);

		//right pixels and radius
		RedRadii[0] = RawRadii[0];
		RawRedpix[0] = RawPixels[0]; //rx
		RawRedpix[1] = RawPixels[1]; //ry
		//left pixels and radius
		RedRadii[1] = RawRadii[1];
		RawRedpix[2] = RawPixels[2]; //lx
		RawRedpix[3] = RawPixels[3]; //ly
	} // end roiIsRed

	void roiIsBlue(cv::Vec<double,4> Pix)
	{
		// fprintf (pFile,"%%%s.roiIsBlue.calledCount = %d;\n", s_handle.c_str(), calledCount);
		/*
		 *  Pix := 4x1 vector
		 *  pix[0 1 2 3] = pix[rx ry lx ly]
		 *  xr = Pix[0]
		 *  yr = Pix[1]
		 *  xl = Pix[2]
		 *  yl = Pix[3]
		 */
		int rx = int(Pix[0]);
		int ry = int(Pix[1]);
		int lx = int(Pix[2]);
		int ly = int(Pix[3]);
		// fprintf (pFile,"%s.roiIsBlue.left_center(%d,:) = [%d %d ];\n", s_handle.c_str(), calledCount, lx, ly);
		// fprintf (pFile,"%s.roiIsBlue.right_center(%d,:) = [%d %d];\n", s_handle.c_str(), calledCount, rx, ry);

		cv::Mat LeftisBlue_Mat, LeftisBlue_U, LeftisBlue_V;
		cv::Mat RightisBlue_Mat, RightisBlue_U, RightisBlue_V;

		int lx_low_offset, ly_low_offset;
		int rx_low_offset, ry_low_offset;
		int lx_high_offset, ly_high_offset;
		int rx_high_offset, ry_high_offset;

		// prevent roi from defining a matrix outside the dimensions of the original
		int lx_low  = lx - roi_halfwindow; if(lx_low < 0) { lx_low_offset = lx_low; lx_low = 0; } else { lx_low_offset = 0; } // then the pixel is within roi_halfwindow of the edge, so lx is set to zero because that's the left corner
		int rx_low  = rx - roi_halfwindow; if(rx_low < 0) { rx_low_offset = rx_low; rx_low = 0; } else { rx_low_offset = 0; }
		int ly_low  = ly - roi_halfwindow; if(ly_low < 0) { ly_low_offset = ly_low; ly_low = 0; } else { ly_low_offset = 0; } 
		int ry_low  = ry - roi_halfwindow; if(ry_low < 0) { ry_low_offset = ry_low; ry_low = 0; } else { ry_low_offset = 0; }

		int lx_high = lx + roi_halfwindow; if(lx_high > image_width)  { lx_high_offset = lx_high - image_width; } else { lx_high_offset = 0; }
		int rx_high = rx + roi_halfwindow; if(rx_high > image_width)  { rx_high_offset = rx_high - image_width; } else { rx_high_offset = 0; }
		int ly_high = ly + roi_halfwindow; if(ly_high > image_height) { ly_high_offset = ly_high - image_height;} else { ly_high_offset = 0; }
		int ry_high = ry + roi_halfwindow; if(ry_high > image_height) { ry_high_offset = ry_high - image_height;} else { ry_high_offset = 0; }

		// ROS_WARN("Blue [lx ly : rx ry] = [%d %d : %d %d ]", lx, ly, rx, ry);
		// ROS_WARN("Blue [lx low : rx low] = [%d : %d ]", lx_low, rx_low);
		// ROS_WARN("Blue [ly low : ry low] = [%d : %d ]", ly_low, ry_low);

		/*--- Select ROI of channled images --- */
		cv::Mat lU_roi = lU( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat lV_roi = lV( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat rU_roi = rU( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		cv::Mat rV_roi = rV( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		// fprintf (pFile,"%s.roiIsBlue.left_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, lx_low, ly_low, (roi_halfwindow - lx_high_offset), (roi_halfwindow - ly_high_offset) );
		// fprintf (pFile,"%s.roiIsBlue.right_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, rx_low, ry_low, (roi_halfwindow - rx_high_offset), (roi_halfwindow - ry_high_offset) );

		/*--- Filter by Color --- */
		/*--- Filter by Color --- */
		cv::inRange( lU_roi, Blue_u_low, Blue_u_high, LeftisBlue_U);
		cv::inRange( rU_roi, Blue_u_low, Blue_u_high, RightisBlue_U);

		cv::inRange( lV_roi, Blue_v_low, Blue_v_high, LeftisBlue_V);
		cv::inRange( rV_roi, Blue_v_low, Blue_v_high, RightisBlue_V);

		cv::bitwise_and(LeftisBlue_U, LeftisBlue_V, LeftisBlue_Mat);
		cv::bitwise_and(RightisBlue_U, RightisBlue_V, RightisBlue_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisBlue_Mat, leftBlueLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisBlue_Mat, rightBlueLEDclosed, 2, closingElement );
		/*--- Find Contours --- */
		leftBlueContours = getContours(leftBlueLEDclosed);
		rightBlueContours = getContours(rightBlueLEDclosed);
		/*--- Find Polygons --- */
		lBluePolygons = getPolygons(leftBlueContours);
		rBluePolygons = getPolygons(rightBlueContours);
		/*--- Find Minimum Enclosing circles --- */
		std::vector<cv::KeyPoint> lRoiKeypoint, rRoiKeypoint;
		lRoiKeypoint = getCircles(lBluePolygons);
		rRoiKeypoint = getCircles(rBluePolygons);
		lBlueKeypoint = remapROIKepoints(lRoiKeypoint, lx_low, ly_low);
		rBlueKeypoint = remapROIKepoints(rRoiKeypoint, rx_low, ry_low);

		/*--- Draw Results --- */
		drawCircles(lBlueKeypoint,lCircle,RGBBlue);
		drawCircles(rBlueKeypoint,rCircle,RGBBlue);
		cv::rectangle(lCircle, cv::Point2f(lx_low, ly_low), cv::Point2f(lx_high - lx_high_offset, ly_high - ly_high_offset), RGBBlue);
		cv::rectangle(rCircle, cv::Point2f(rx_low, ry_low), cv::Point2f(rx_high - rx_high_offset, ry_high - ry_high_offset), RGBBlue);
		// drawCircles(lRoiKeypoint,leftBlueLEDclosed,RGBBlue);
		// drawCircles(rRoiKeypoint,rightBlueLEDclosed,RGBBlue);


		/*--- Populate Pixel Array Results --- */
		Bluepix   = testRadii(rBlueKeypoint,  lBlueKeypoint,  Bluestr);
		//right pixels and radius
		BlueRadii[0] = RawRadii[0];
		RawBluepix[0] = RawPixels[0]; // rx
		RawBluepix[1] = RawPixels[1]; // ry
		//left pixels and radius
		BlueRadii[1] = RawRadii[1];
		RawBluepix[2] = RawPixels[2]; // lx 
		RawBluepix[3] = RawPixels[3]; // ly
	} // end roiIsBlue

	void roiIsGreen(cv::Vec<double,4> Pix)
	{
		// fprintf (pFile,"%%%s.roiIsGreen.calledCount = %d;\n", s_handle.c_str(), calledCount);
		/*
		 *  Pix := 4x1 vector
		 *  pix[0 1 2 3] = pix[rx ry lx ly]
		 *  xr = Pix[0]
		 *  yr = Pix[1]
		 *  xl = Pix[2]
		 *  yl = Pix[3]
		 */
		int rx = int(Pix[0]);
		int ry = int(Pix[1]);
		int lx = int(Pix[2]);
		int ly = int(Pix[3]);
		// fprintf (pFile,"%s.roiIsGreen.left_center(%d,:) = [%d %d ];\n", s_handle.c_str(), calledCount, lx, ly);
		// fprintf (pFile,"%s.roiIsGreen.right_center(%d,:) = [%d %d];\n", s_handle.c_str(), calledCount, rx, ry);


		cv::Mat LeftisGreen_Mat, LeftisGreen_U, LeftisGreen_V;
		cv::Mat RightisGreen_Mat, RightisGreen_U, RightisGreen_V;

		int lx_low_offset, ly_low_offset;
		int rx_low_offset, ry_low_offset;
		int lx_high_offset, ly_high_offset;
		int rx_high_offset, ry_high_offset;

		// prevent roi from defining a matrix outside the dimensions of the original
		int lx_low  = lx - roi_halfwindow; if(lx_low < 0) { lx_low_offset = lx_low; lx_low = 0; } else { lx_low_offset = 0; } // then the pixel is within roi_halfwindow of the edge, so lx is set to zero because that's the left corner
		int rx_low  = rx - roi_halfwindow; if(rx_low < 0) { rx_low_offset = rx_low; rx_low = 0; } else { rx_low_offset = 0; }
		int ly_low  = ly - roi_halfwindow; if(ly_low < 0) { ly_low_offset = ly_low; ly_low = 0; } else { ly_low_offset = 0; } 
		int ry_low  = ry - roi_halfwindow; if(ry_low < 0) { ry_low_offset = ry_low; ry_low = 0; } else { ry_low_offset = 0; }

		int lx_high = lx + roi_halfwindow; if(lx_high > image_width)  { lx_high_offset = lx_high - image_width; } else { lx_high_offset = 0; }
		int rx_high = rx + roi_halfwindow; if(rx_high > image_width)  { rx_high_offset = rx_high - image_width; } else { rx_high_offset = 0; }
		int ly_high = ly + roi_halfwindow; if(ly_high > image_height) { ly_high_offset = ly_high - image_height;} else { ly_high_offset = 0; }
		int ry_high = ry + roi_halfwindow; if(ry_high > image_height) { ry_high_offset = ry_high - image_height;} else { ry_high_offset = 0; }

		// ROS_WARN("Green [lx ly : rx ry] = [%d %d : %d %d ]", lx, ly, rx, ry);
		// ROS_WARN("Green [lx low : rx low] = [%d : %d ]", lx_low, rx_low);
		// ROS_WARN("Green [ly low : ry low] = [%d : %d ]", ly_low, ry_low);

		/*--- Select ROI of channled images --- */
		cv::Mat lU_roi = lU( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat lV_roi = lV( cv::Rect( lx_low, ly_low, (2*roi_halfwindow - lx_high_offset), (2*roi_halfwindow - ly_high_offset) ) ); 
		cv::Mat rU_roi = rU( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		cv::Mat rV_roi = rV( cv::Rect( rx_low, ry_low, (2*roi_halfwindow - rx_high_offset), (2*roi_halfwindow - ry_high_offset) ) ); 
		// fprintf (pFile,"%s.roiIsGreen.left_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, lx_low, ly_low, (roi_halfwindow - lx_high_offset), (roi_halfwindow - ly_high_offset) );
		// fprintf (pFile,"%s.roiIsGreen.right_roi(%d,:) = [%d %d %d %d];\n", s_handle.c_str(), calledCount, rx_low, ry_low, (roi_halfwindow - rx_high_offset), (roi_halfwindow - ry_high_offset) );


		/*--- Filter by Color --- */
		cv::inRange( lU_roi, Green_u_low, Green_u_high, LeftisGreen_U);
		cv::inRange( rU_roi, Green_u_low, Green_u_high, RightisGreen_U);

		cv::inRange( lV_roi, Green_v_low, Green_v_high, LeftisGreen_V);
		cv::inRange( rV_roi, Green_v_low, Green_v_high, RightisGreen_V);

		cv::bitwise_and(LeftisGreen_U, LeftisGreen_V, LeftisGreen_Mat);
		cv::bitwise_and(RightisGreen_U, RightisGreen_V, RightisGreen_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisGreen_Mat, leftGreenLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisGreen_Mat, rightGreenLEDclosed, 2, closingElement );
		/*--- Find Contours --- */
		leftGreenContours = getContours(leftGreenLEDclosed);
		rightGreenContours = getContours(rightGreenLEDclosed);
		/*--- Find Polygons --- */
		lGreenPolygons = getPolygons(leftGreenContours);
		rGreenPolygons = getPolygons(rightGreenContours);
		/*--- Find Minimum Enclosing circles --- */
		std::vector<cv::KeyPoint> lRoiKeypoint, rRoiKeypoint;
		lRoiKeypoint = getCircles(lGreenPolygons);
		rRoiKeypoint = getCircles(rGreenPolygons);
		lGreenKeypoint = remapROIKepoints(lRoiKeypoint, lx_low, ly_low);
		rGreenKeypoint = remapROIKepoints(rRoiKeypoint, rx_low, ry_low);

		/*--- Draw Results --- */
		drawCircles(lGreenKeypoint,lCircle,RGBGreen);
		drawCircles(rGreenKeypoint,rCircle,RGBGreen);
		cv::rectangle(lCircle, cv::Point2f(lx_low, ly_low), cv::Point2f(lx_high - lx_high_offset, ly_high - ly_high_offset), RGBGreen);
		cv::rectangle(rCircle, cv::Point2f(rx_low, ry_low), cv::Point2f(rx_high - rx_high_offset, ry_high - ry_high_offset), RGBGreen);
		// drawCircles(lRoiKeypoint,leftGreenLEDclosed,RGBGreen);
		// drawCircles(rRoiKeypoint,rightGreenLEDclosed,RGBGreen);

		Greenpix  = testRadii(rGreenKeypoint, lGreenKeypoint, Greenstr);
		//right pixels and radius
		GreenRadii[0] = RawRadii[0];
		RawGreenpix[0] = RawPixels[0];
		RawGreenpix[1] = RawPixels[1];
		//left pixels and radius
		GreenRadii[1] = RawRadii[1];
		RawGreenpix[2] = RawPixels[2];
		RawGreenpix[3] = RawPixels[3];
	} // end roiIsGreen

	void isRed()
	{
		fprintf (pFile,"%%%s.isRed.calledCount = %d;\n", s_handle.c_str(), calledCount);

		cv::Mat LeftisRed_Mat, LeftisRed_U, LeftisRed_V;
		cv::Mat RightisRed_Mat, RightisRed_U, RightisRed_V;
		
		/*--- Filter by Color --- */
		cv::inRange( lU, Red_u_low, Red_u_high, LeftisRed_U);
		cv::inRange( rU, Red_u_low, Red_u_high, RightisRed_U);

		cv::inRange( lV, Red_v_low, Red_v_high, LeftisRed_V);
		cv::inRange( rV, Red_v_low, Red_v_high, RightisRed_V);

		cv::bitwise_and(LeftisRed_U, LeftisRed_V, LeftisRed_Mat);
		cv::bitwise_and(RightisRed_U, RightisRed_V, RightisRed_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisRed_Mat, leftRedLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisRed_Mat, rightRedLEDclosed, 2, closingElement );

		/*--- Find Contours --- */
		leftRedContours = getContours(leftRedLEDclosed);
		rightRedContours = getContours(rightRedLEDclosed);
		/*--- Find Polygons --- */
		lRedPolygons = getPolygons(leftRedContours);
		rRedPolygons = getPolygons(rightRedContours);
		/*--- Find Minimum Enclosing circles --- */
		lRedKeypoint = getCircles(lRedPolygons);
		rRedKeypoint = getCircles(rRedPolygons);
		/*--- Draw Results --- */
		drawCircles(lRedKeypoint,lCircle,RGBRed);
		drawCircles(rRedKeypoint,rCircle,RGBRed);

		/*--- Populate Pixel Array Results --- */
		Redpix    = testRadii(rRedKeypoint,   lRedKeypoint,   Redstr);
		//right pixels and radius
		RedRadii[0] = RawRadii[0];
		RawRedpix[0] = RawPixels[0]; //rx
		RawRedpix[1] = RawPixels[1]; //ry
		//left pixels and radius
		RedRadii[1] = RawRadii[1];
		RawRedpix[2] = RawPixels[2]; //lx
		RawRedpix[3] = RawPixels[3]; //ly
	}//end isRed()

	void isBlue()
	{
		fprintf (pFile,"%%%s.isBlue.calledCount = %d;\n", s_handle.c_str(), calledCount);

		cv::Mat LeftisBlue_Mat, LeftisBlue_U, LeftisBlue_V;
		cv::Mat RightisBlue_Mat, RightisBlue_U, RightisBlue_V;

		/*--- Filter by Color --- */
		cv::inRange( lU, Blue_u_low, Blue_u_high, LeftisBlue_U);
		cv::inRange( rU, Blue_u_low, Blue_u_high, RightisBlue_U);

		cv::inRange( lV, Blue_v_low, Blue_v_high, LeftisBlue_V);
		cv::inRange( rV, Blue_v_low, Blue_v_high, RightisBlue_V);

		cv::bitwise_and(LeftisBlue_U, LeftisBlue_V, LeftisBlue_Mat);
		cv::bitwise_and(RightisBlue_U, RightisBlue_V, RightisBlue_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisBlue_Mat, leftBlueLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisBlue_Mat, rightBlueLEDclosed, 2, closingElement );
		/*--- Find Contours --- */
		leftBlueContours = getContours(leftBlueLEDclosed);
		rightBlueContours = getContours(rightBlueLEDclosed);
		/*--- Find Polygons --- */
		lBluePolygons = getPolygons(leftBlueContours);
		rBluePolygons = getPolygons(rightBlueContours);
		/*--- Find Minimum Enclosing circles --- */
		lBlueKeypoint = getCircles(lBluePolygons);
		rBlueKeypoint = getCircles(rBluePolygons);

		/*--- Draw Results --- */
		drawCircles(lBlueKeypoint,lCircle,RGBBlue);
		drawCircles(rBlueKeypoint,rCircle,RGBBlue);

		/*--- Populate Pixel Array Results --- */
		// Bluepix   = testPix(rBlueKeypoint,  lBlueKeypoint,  Bluestr);
		Bluepix   = testRadii(rBlueKeypoint,  lBlueKeypoint,  Bluestr);

		//right pixels and radius
			BlueRadii[0] = RawRadii[0];
			RawBluepix[0] = RawPixels[0];
			RawBluepix[1] = RawPixels[1];
		//left pixels and radius
			BlueRadii[1] = RawRadii[1];
			RawBluepix[2] = RawPixels[2];
			RawBluepix[3] = RawPixels[3];
	}//end isBlue()

	void isGreen()
	{
		fprintf (pFile,"%%%s.isGreen.calledCount = %d;\n", s_handle.c_str(), calledCount);

		cv::Mat LeftisGreen_Mat, LeftisGreen_U, LeftisGreen_V;
		cv::Mat RightisGreen_Mat, RightisGreen_U, RightisGreen_V;

		/*--- Filter by Color --- */
		cv::inRange( lU, Green_u_low, Green_u_high, LeftisGreen_U);
		cv::inRange( rU, Green_u_low, Green_u_high, RightisGreen_U);

		cv::inRange( lV, Green_v_low, Green_v_high, LeftisGreen_V);
		cv::inRange( rV, Green_v_low, Green_v_high, RightisGreen_V);

		cv::bitwise_and(LeftisGreen_U, LeftisGreen_V, LeftisGreen_Mat);
		cv::bitwise_and(RightisGreen_U, RightisGreen_V, RightisGreen_Mat);

		/*--- Morph binary image --- */
		cv::morphologyEx( LeftisGreen_Mat, leftGreenLEDclosed, 2, closingElement );
		cv::morphologyEx( RightisGreen_Mat, rightGreenLEDclosed, 2, closingElement );
		/*--- Find Contours --- */
		leftGreenContours = getContours(leftGreenLEDclosed);
		rightGreenContours = getContours(rightGreenLEDclosed);
		/*--- Find Polygons --- */
		lGreenPolygons = getPolygons(leftGreenContours);
		rGreenPolygons = getPolygons(rightGreenContours);
		/*--- Find Minimum Enclosing circles --- */
		lGreenKeypoint = getCircles(lGreenPolygons);
		rGreenKeypoint = getCircles(rGreenPolygons);
		/*--- Draw Results --- */
		drawCircles(lGreenKeypoint,lCircle,RGBGreen);
		drawCircles(rGreenKeypoint,rCircle,RGBGreen);
		drawPolygons(lGreenPolygons,lContours,RGBGreen);
		drawPolygons(rGreenPolygons,rContours,RGBGreen);

		Greenpix  = testRadii(rGreenKeypoint, lGreenKeypoint, Greenstr);
		//right pixels and radius
		GreenRadii[0] = RawRadii[0];
		RawGreenpix[0] = RawPixels[0];
		RawGreenpix[1] = RawPixels[1];
		//left pixels and radius
		GreenRadii[1] = RawRadii[1];
		RawGreenpix[2] = RawPixels[2];
		RawGreenpix[3] = RawPixels[3];
	} //end isGreen()

	cv::Vec<double,4> testRadii(std::vector<cv::KeyPoint> kpr, std::vector<cv::KeyPoint> kpl, const char* color)
	{

		cv::Vec<double,4> pix;
			double bestLeftRadius, bestRightRadius; // assume first radius is best
			int bestLeftIndex, bestRightIndex; // assume first radius is best

		if (kpl.size() > 0)
		{
			bestLeftRadius = 0;
			bestLeftIndex = 0;
			if (kpl.size() > 1)
			{//begin if (kpr.size() > 1)
				// ROS_WARN("ugvStereo: Error: Detected %i %s Circles in left image", int(kpl.size()), color);
				for (unsigned int i=0; i<kpl.size(); i++)
				{
					// ROS_INFO("radius %i : % -6.8f", i, kpl[i].size);
					if (kpl[i].size > bestLeftRadius)
					{ // only change indices if the radius is better
						bestLeftRadius = kpl[i].size;
						bestLeftIndex = i;
					} // if (Radius > bestRightRadius)
				}//end for (unsigned int i=0; i<kpr.size(); i++)
				// ROS_INFO("bestLeftRadius = % -6.8f", bestLeftRadius);
				// ROS_INFO("bestLeftIndex = %d ", bestLeftIndex);
			}//end if (kpr.size() > 1)
			pix[2] = Cx - kpl[bestLeftIndex].pt.x;
			pix[3] = Cy - kpl[bestLeftIndex].pt.y;
			RawRadii[1] = kpl[bestLeftIndex].size;
			RawPixels[2] = kpl[bestLeftIndex].pt.x;
			RawPixels[3] = kpl[bestLeftIndex].pt.y;
		} else { //else kpl[i].size !> 0
			stereoflag = false;
			pix[2] = 0;
			pix[3] = 0;
			RawRadii[1] = 0;
			RawPixels[2] = 0;
			RawPixels[3] = 0;
		} // end if ( 0 < kpl[i].size)

		if (kpr.size() > 0)
		{
			bestRightRadius = 0;
			bestRightIndex = 0;
			if (kpr.size() > 1)
			{//begin if (kpr.size() > 1)
				// ROS_WARN("ugvStereo: Error: Detected %i %s Circles in left image", int(kpl.size()), color);
				for (unsigned int i=0; i<kpr.size(); i++)
				{
					// ROS_INFO("radius %i : % -6.8f", i, kpr[i].size);
					// ROS_INFO("Radius %d = % -6.8f", i, Radius);
					if (kpr[i].size > bestRightRadius)
					{ // only change indices if the radius is better
						bestRightRadius = kpr[i].size;
						bestRightIndex = i;
					} // if (Radius > bestRightRadius)
				}//end for (unsigned int i=0; i<kpr.size(); i++)
				// ROS_INFO("bestRightRadius = % -6.8f", bestRightRadius);
				// ROS_INFO("bestRightIndex = %d ", bestRightIndex);
			}//end if (kpr.size() > 1)
			pix[0] = Cx - kpr[bestRightIndex].pt.x;
			pix[1] = Cy - kpr[bestRightIndex].pt.y;
			RawPixels[0] = kpr[bestRightIndex].pt.x;
			RawPixels[1] = kpr[bestRightIndex].pt.y;
			RawRadii[0] = kpr[bestRightIndex].size;
		} else { // else kpr[i].size !> 0
			stereoflag = false;
			pix[0] = 0;
			pix[1] = 0;
			RawPixels[0] = 0;
			RawPixels[1] = 0;
			RawRadii[0] = 0;
		} // end if ( 0 < kpr[i].size)

		return pix;
	}

	cv::Vec<double,4> testPix(std::vector<cv::KeyPoint> kpr, std::vector<cv::KeyPoint> kpl, const char* color)
	{
		cv::Vec<double,4> pix;
			if (kpr.size() == 1)
			{//begin if
				if (kpl.size() == 1)
				{//begin if
					pix[0] = kpr[0].pt.x - Cx;
					pix[1] = Cy - kpr[0].pt.y;
					pix[2] = kpl[0].pt.x - Cx;
					pix[3] = Cy - kpl[0].pt.y;

					RawPixels[0] = kpr[0].pt.x;
					RawPixels[1] = kpr[0].pt.y;
					RawPixels[2] = kpl[0].pt.x;
					RawPixels[3] = kpl[0].pt.y;
					RawRadii[0] = kpr[0].size;
					RawRadii[1] = kpl[0].size;
				} //end if
				else
				{ //begin else
					// ROS_INFO("ugvStereo: Error: Detected %i %s Circles in left image", int(kpl.size()), color);
					stereoflag = false;
					pix[0] = 0;
					pix[1] = 0;
					pix[2] = 0;
					pix[3] = 0;
					RawRadii[0] = 0;
					RawRadii[1] = 0;
				} // end else
			} //end if
			else
			{ //begin else
				// ROS_INFO("ugvStereo: Error: Detected %i %s Circles in right image", int(kpr.size()), color);
				stereoflag = false;
					pix[0] = 0;
					pix[1] = 0;
					pix[2] = 0;
					pix[3] = 0;
					RawRadii[0] = 0;
					RawRadii[1] = 0;
			} // end else
			return pix;
	}

	std::vector<std::vector<cv::Point> > getContours(cv::Mat src_image)
	{
		/// Detect edges using Threshold
		cv::Mat threshold_output;
		double max_value = 255; // in image?
		cv::threshold( src_image, threshold_output, EdgeThreshold, max_value, cv::THRESH_BINARY );

		/// Find contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
		// cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		// cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, cv::Point(0, 0) );
		// cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

		return contours;
	}

	std::vector<std::vector<cv::Point> > getPolygons(std::vector<std::vector<cv::Point> > contours)
	{
		/// Approximate contours to polygons + get bounding rects and circles
		std::vector<std::vector<cv::Point> > polygons( contours.size() );
		double eps = 0.25;
		for( size_t i = 0; i < contours.size(); i++ )
		{cv::approxPolyDP( cv::Mat(contours[i]), polygons[i], eps, true );}
		return polygons;
	} //end getPolygons

	void drawPolygons(std::vector<std::vector<cv::Point> > polygons, cv::Mat dest_image,cv::Scalar color )
	{
		for( size_t i = 0; i < polygons.size(); i++ )
		{cv::drawContours(dest_image, polygons, int(i), color , 1,8,std::vector<cv::Vec4i>(), 0, cv::Point());}
	} //end drawPolygons

	std::vector<cv::KeyPoint> remapROIKepoints(std::vector<cv::KeyPoint> roiKP, double x_offset, double y_offset)
	{
		/// Approximate contours to polygons + get bounding rects and circles
		std::vector<cv::KeyPoint> keypoints;
		std::vector<cv::Point2f> center( roiKP.size() );
		std::vector<float>radius( roiKP.size() );
		for( size_t i = 0; i < roiKP.size(); i++ )
		{
			center[i].x = roiKP[i].pt.x + x_offset;
			center[i].y = roiKP[i].pt.y + y_offset;
			radius[i] = roiKP[i].size;
			keypoints.push_back(cv::KeyPoint(center[i], radius[i]));
		}
		return keypoints;
	} //end remapROIKepoints

	std::vector<cv::KeyPoint> getCircles(std::vector<std::vector<cv::Point> > polygons)
	{
		/// Approximate contours to polygons + get bounding rects and circles
		std::vector<cv::KeyPoint> keypoints;
		std::vector<cv::Point2f> center( polygons.size() );
		std::vector<float>radius( polygons.size() );
		for( size_t i = 0; i < polygons.size(); i++ )
		{
			cv::minEnclosingCircle( polygons[i], center[i], radius[i] );
			keypoints.push_back(cv::KeyPoint(center[i], radius[i]));
		}
		return keypoints;
	} //end getCircles

	void drawCircles(std::vector<cv::KeyPoint> keypoints, cv::Mat dest_image, cv::Scalar color )
	{
		for (unsigned int i=0; i<keypoints.size(); i++)
			{
			// std::cout << "kpl i = " << (i+1) << std::endl;
			cv::Point Center(cvRound(keypoints[i].pt.x), cvRound(keypoints[i].pt.y));
			int Radius = cvRound(keypoints[i].size);
			cv::circle(dest_image, Center, 1, color, -1, 8, 0);
			cv::circle(dest_image, Center, Radius, color, 1, 8, 0);
			double font_scale = 1;
			int thickness = 1;
			int line_type = cv::LINE_8;
			// bool image_originbottom_left = false;
			// cv::putText(dest_image, "x", Center, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, line_type, false );
			} //end for (int i=0; i<keypoints.size(); i++)
	} //end drawCircles

// void cv::putText 	( 	InputOutputArray  	img,
// 		const String &  	text,
// 		Point  	org,
// 		int  	fontFace,
// 		double  	fontScale,
// 		Scalar  	color,
// 		int  	thickness = 1,
// 		int  	lineType = LINE_8,
// 		bool  	bottomLeftOrigin = false 
// 	) 		

// Draws a text string.

// The function putText renders the specified text string in the image. Symbols that cannot be rendered using the specified font are replaced by question marks. See getTextSize for a text rendering code example.

// Parameters
//     img	Image.
//     text	Text string to be drawn.
//     org	Bottom-left corner of the text string in the image.
//     fontFace	Font type, see cv::HersheyFonts.
//     fontScale	Font scale factor that is multiplied by the font-specific base size.
//     color	Text color.
//     thickness	Thickness of the lines used to draw a text.
//     lineType	Line type. See the line for details.
//     bottomLeftOrigin	When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner. 


/* ################## --------- Math Tool Functions --------- ################## */

	cv::Matx<double,4,1> Cross(cv::Matx<double,4,1> A, cv::Matx<double,4,1> B)
	{//computes A x B
		cv::Matx<double,4,1> C;
		C(0,0) =   ((A(1,0) * B(2,0)) - (A(2,0) * B(1,0)));
		C(1,0) = - ((A(0,0) * B(2,0)) - (A(2,0) * B(0,0)));
		C(2,0) =   ((A(0,0) * B(1,0)) - (A(1,0) * B(0,0)));
		C(3,0) = 0;
		return C;
	}

	cv::Matx<double,3,3> calcJ(cv::Vec<double,4> Pix)
	{
		/*  Calculate Jacobian Matrix for one target
		 *  Pix := 4x1 vector
		 *  pix[0 1 2 3] = pix[rx ry lx ly]
		 *  xr = Pix[0]
		 *  yr = Pix[1]
		 *  xl = Pix[2]
		 *  yl = Pix[3]
		 */
		cv::Matx33d J;
		double disparity = (Pix[2] - Pix[0]);
		// double disparity = std::abs(Pix[2] - Pix[0]);
		J = cv::Matx33d(
			// row 1
				-(Baseline * Pix[0]) / (2 * disparity * disparity), // (-b xr / 2 d^2)
				 (Baseline * Pix[2]) / (2 * disparity * disparity), // (b xl / 2 d^2)
				 0, // 0

			// row 2
			 -(Baseline * Pix[3]) / (disparity * disparity), // - b yl / d^2
				(Baseline * Pix[3]) / (disparity * disparity), //   b yl / d^2
				 Baseline / disparity, // b/d

			// row 3
			 -(Baseline * FocalLength) / (disparity * disparity), // -bf/d
				(Baseline * FocalLength) / (disparity * disparity), // bf/d
				0); //0

		return J;
	}

	cv::Matx<double,4,1> calcVec(cv::Vec<double,4> Pix)
	{
		/*  Calculate 3d position from corresponding pixels
		 *  Pix := 4x1 vector
		 *  pix[0 1 2 3] = pix[rx ry lx ly]
		 *  xr = Pix[0]
		 *  yr = Pix[1]
		 *  xl = Pix[2]
		 *  yl = Pix[3]
		 */
		cv::Matx<double,4,1> Vecpix;
		// double disparity = std::abs(Pix[2] - Pix[0]);
		double disparity = std::abs(Pix[2] - Pix[0]);
		// w = 0; 4th dimension veci[x y z w]
		Vecpix(3,0) = 1;
		Vecpix(2,0) = (Baseline*FocalLength) / disparity;
		Vecpix(1,0) = Vecpix(2,0) * (Pix[1] + Pix[3] ) / ( 2 * FocalLength );
		Vecpix(0,0) = Vecpix(2,0) * (Pix[0] + Pix[2] ) / ( 2 * FocalLength );
		return Vecpix;
	}

	cv::Matx<double,3,3> wedgevector(cv::Matx<double,4,1> V)
	{
		cv::Matx33d Vx;
		Vx = cv::Matx33d(
						0,  V(2,0), -V(1,0),// row 1
			-V(2,0),       0,  V(0,0), // row 2
			 V(1,0), -V(0,0),      0); // row 3
			return Vx;
	}

	cv::Matx<double,3,3> makeRz(double theta) //in radians
	{
		cv::Matx33d R;
		R = cv::Matx33d(
			 cos(theta), sin(theta), 0,// row 1
			-sin(theta), cos(theta), 0, // row 2
								0, 					0, 1); // row 3
			return R;
	}
	cv::Matx<double,3,3> makeRx(double theta) //in radians
	{
		cv::Matx33d R;
		R = cv::Matx33d(
			 1, 				 0, 				 0, // row 1
			 0, cos(theta), sin(theta), // row 2
			 0,-sin(theta), cos(theta));// row 3
			return R;
	}

	cv::Matx<double,3,3> jacobianOfNorm(cv::Matx<double,4,1> p)
	{
		double x, y, z, A;
		x = p(0,0);
		y = p(1,0);
		z = p(2,0);
		A = 1 / pow(x*x + y*y + z*z, 1.5);

		cv::Matx33d B;
		B = cv::Matx33d(
			y*y + z*z,    -x*y,    -x*z,
					 -x*y, x*x+z*z,    -y*z,
					 -x*z,    -y*z, x*x+y*y);
	 return A*B;
	}

	cv::Matx<double,1,2> jacobianOfAtan2(double y, double x)
	{
		double A = 1 / (x*x+y*y);
		cv::Matx12d px;
		px = A*cv::Matx12d(-y,x);
		return px;
	}

}; //end stereoObs class

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereoLocalizer");
	stereoObs dP;
	ros::spin();
	return 0;
}
