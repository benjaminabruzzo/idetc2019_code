// This node loads images from the hard drive, then publishes them as a ros-topic
	//ROS Communications
	#include <ros/ros.h>
		// Messages
		#include <ardrone_autonomy/Navdata.h>
		#include <std_msgs/Empty.h>
		#include <geometry_msgs/Twist.h>
		#include <nav_msgs/Odometry.h>
		#include <tf/transform_listener.h>
		#include <tf/transform_broadcaster.h>

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
	
		#include <unistd.h>
		#include <vector>
		#include <image_transport/image_transport.h>
		#include <cv_bridge/cv_bridge.h>
		#include <opencv2/opencv.hpp>
		#include <opencv2/highgui/highgui.hpp>
		#include <sensor_msgs/CameraInfo.h>

		#include <camera_calibration_parsers/parse_yml.h>

		


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << std::setw(5) << std::setfill('0')  << n ;
        return stm.str() ;
    }
}

class rawImagePub
{
	private:
	/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
		/*----- image transport Channels */
		image_transport::ImageTransport it;
		image_transport::Publisher left_pub, right_pub;
		ros::Subscriber HastShutDown_sub;

		ros::Publisher left_info_pub, right_info_pub;
	    sensor_msgs::ImagePtr left_msg, right_msg;
	    sensor_msgs::CameraInfo left_info_msg, right_info_msg;

		std::string left_topic, right_topic;
		std::string left_image_frame_id, right_image_frame_id;
		std::string left_info_topic, right_info_topic;
		std::string left_cal_yaml, right_cal_yaml;
		std::string s_LeftOriginal, s_RightOriginal;
		std::string s_root, s_date, s_run, s_user;

	    cv::Mat left_image, right_image;

		bool success;

		std::string camera_name;
	public:
		rawImagePub()
		: it(n)
		{
			if(ros::param::get("~image_src_path",s_root)){} else {s_root = "/home/benjamin/ros/data/17180201/025";} 
			if(ros::param::get("~left_cal_yaml",left_cal_yaml)){} else {left_cal_yaml = "/home/benjamin/ros/src/hast/cam_info/16369047.yaml";} 
			if(ros::param::get("~right_cal_yaml",right_cal_yaml)){} else {right_cal_yaml = "/home/benjamin/ros/src/hast/cam_info/16306423.yaml";} 
			if(ros::param::get("~left_image_frame_id",left_image_frame_id)){} else {left_image_frame_id = "/hast/kobuki/stereo_center";} 
			if(ros::param::get("~right_image_frame_id",right_image_frame_id)){} else {right_image_frame_id = "/hast/kobuki/stereo_center";} 

			if(ros::param::get("~left_topic",left_topic)){} else {left_topic = "/pgrstereo/left/image_raw_color";} 
				left_pub = it.advertise(left_topic.c_str(), 1);
			if(ros::param::get("~right_topic",right_topic)){} else {right_topic = "/pgrstereo/right/image_raw_color";} 
				right_pub = it.advertise(right_topic.c_str(), 1);
			

			if(ros::param::get("~left_info_topic",left_info_topic)){}    else {left_info_topic = "/pgrstereo/left/camera_info";} 
				 left_info_pub = n.advertise<sensor_msgs::CameraInfo>(left_info_topic.c_str(), 1);
			if(ros::param::get("~right_info_topic",right_info_topic)){} else {right_info_topic = "/pgrstereo/right/camera_info";} 
				right_info_pub = n.advertise<sensor_msgs::CameraInfo>(right_info_topic.c_str(), 1);
			
			HastShutDown_sub = n.subscribe("/hast/shutdown", 10, &rawImagePub::nodeShutDown, this);



			ROS_INFO("loading yaml files");
			success = camera_calibration_parsers::readCalibrationYml(left_cal_yaml.c_str(), camera_name, left_info_msg);
			success = camera_calibration_parsers::readCalibrationYml(right_cal_yaml.c_str(), camera_name, right_info_msg);
			left_info_msg.header.frame_id = left_image_frame_id;
			right_info_msg.header.frame_id = right_image_frame_id;

			spinPublisher();

		}

		void spinPublisher()
		{

			uint b=1;
			while (n.ok())
			// for(int b = 1; b < 482; ++b)
			{
				s_LeftOriginal = s_root + "/original/left_raw_" + patch::to_string(b) + ".png"; // ROS_INFO("Loading %s", s_LeftOriginal.c_str());
				s_RightOriginal = s_root + "/original/left_raw_" + patch::to_string(b) + ".png"; // ROS_INFO("Loading %s", s_RightOriginal.c_str());

				try { left_image = cv::imread(s_LeftOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(0); } 
				catch (...) { ros::shutdown();	}
				try { right_image = cv::imread(s_RightOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(0); } 
				catch (...) { ros::shutdown();	}

				// left_image = cv::imread(s_LeftOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(30);
				// right_image = cv::imread(s_RightOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(30);
				
				left_info_msg.header.seq = b;
				right_info_msg.header.seq = b;
			    left_info_msg.header.stamp = ros::Time::now();
			    right_info_msg.header.stamp = left_info_msg.header.stamp;

				left_msg = cv_bridge::CvImage(left_info_msg.header, "bgr8", left_image).toImageMsg();
				right_msg = cv_bridge::CvImage(right_info_msg.header, "bgr8", right_image).toImageMsg();

				left_pub.publish(left_msg); left_info_pub.publish(left_info_msg);
				right_pub.publish(right_msg); right_info_pub.publish(right_info_msg);

				ros::spinOnce();
				++b;
				// usleep(100000);
				ros::Duration(0.2).sleep(); 
			}
		}



	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{ // node cleanup for end of experiment
		if(ShutDown->flag)
			{
				ROS_INFO("rawImagePub: Shutting Down...");

				ros::shutdown();
			}
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rawImagePub");
	rawImagePub rI;
	ros::spin();
	return 0;
}
