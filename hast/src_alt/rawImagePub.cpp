// This node loads images from the hard drive, then publishes them as a ros-topic
#include <ros/ros.h>
	#include <unistd.h>
	#include <vector>
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>
	#include <opencv2/opencv.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <sensor_msgs/CameraInfo.h>


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << std::setw(5) << std::setfill('0')  << n ;
        return stm.str() ;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	std::string left_topic, right_topic, left_info_topic, right_info_topic;
	if(ros::param::get("~left_topic",left_topic)){} else {left_topic = "/pgrstereo/left/image_raw_color";} // proportional gain on position control
	if(ros::param::get("~right_topic",right_topic)){} else {right_topic = "/pgrstereo/left/image_raw_color";} // proportional gain on position control
	if(ros::param::get("~left_info_topic",left_info_topic)){} else {left_info_topic = "/pgrstereo/left/image_raw_color";} // proportional gain on position control
	if(ros::param::get("~right_info_topic",right_info_topic)){} else {right_info_topic = "/pgrstereo/left/image_raw_color";} // proportional gain on position control

	image_transport::Publisher left_pub = it.advertise(left_topic.c_str(), 1);
	image_transport::Publisher right_pub = it.advertise(right_topic.c_str(), 1);
	// ros::Publisher left_info_pub = nh.advertise(left_info_topic.c_str(), 1);
	// ros::Publisher right_info_pub = nh.advertise(right_info_topic.c_str(), 1);
	
	std::string s_LeftOriginal, s_RightOriginal, s_root, s_date, s_run, s_user;

	if(ros::param::get("~image_src_path",s_root)){} else {s_root = "/home/benjamin/ros/data/$(arg date)/$(arg run)";} // proportional gain on position control

    cv::Mat left_image, right_image;
    sensor_msgs::ImagePtr left_msg, right_msg;
    sensor_msgs::CameraInfo left_info_msg, right_info_msg;

	left_info_msg.header.frame_id = "/hast/kobuki/stereo_center";
	right_info_msg.header.frame_id = "/hast/kobuki/stereo_center";

	uint b=1;
	while (nh.ok())
	// for(int b = 1; b < 482; ++b)
	{
		s_LeftOriginal = s_root + s_date + "/" + s_run + "/original/LeftOriginal_" + patch::to_string(b) + ".png"; // ROS_INFO("Loading %s", s_LeftOriginal.c_str());
		s_RightOriginal = s_root + s_date + "/" + s_run + "/original/RightOriginal_" + patch::to_string(b) + ".png"; // ROS_INFO("Loading %s", s_RightOriginal.c_str());
		
		left_image = cv::imread(s_LeftOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(30);
		right_image = cv::imread(s_RightOriginal, CV_LOAD_IMAGE_COLOR);cv::waitKey(30);
		
		left_info_msg.header.seq = b;
		right_info_msg.header.seq = b;
	    left_info_msg.header.stamp = ros::Time::now();
	    right_info_msg.header.stamp = left_info_msg.header.stamp;

		left_msg = cv_bridge::CvImage(left_info_msg.header, "bgr8", left_image).toImageMsg();
		right_msg = cv_bridge::CvImage(right_info_msg.header, "bgr8", right_image).toImageMsg();

		left_pub.publish(left_msg);
		right_pub.publish(right_msg);

		ros::spinOnce();
		++b;
		usleep(100000);
	}
}



using namespace cv;
using namespace std;

int main()
{
    FileStorage fs("calib.yml", FileStorage::READ);

    string time;
    int calibrationImageWidth;
    int calibrationImageHeight;
    int numberOfCrossPointsInWidth;
    int numberOfCrossPointsInHeight;
    double squareSize;
    int numberOfCalibratedImages;
    Mat cameraMatrix1;
    Mat distortionCoefficient1;
    vector<Mat> rotationMatrix1;
    vector<Mat> translationMatrix1;

    // Read data
    FileNode fn_time = fs.root();
    time = fn_time["time"];

    calibrationImageWidth = fs["calibrationImageWidth"];
    calibrationImageHeight = fs["calibrationImageHeight"];
    numberOfCrossPointsInWidth = fs["numberOfCrossPointsInWidth"];
    numberOfCrossPointsInHeight = fs["numberOfCrossPointsInHeight"];
    squareSize = fs["squareSize"];
    numberOfCalibratedImages = fs["numberOfCalibratedImages"];

    fs["cameraMatrix1"] >> cameraMatrix1;
    fs["distortionCoefficient1"] >> distortionCoefficient1;

    FileNode fn_rot = fs["rotationMatrix1"];
    FileNodeIterator fn_rot_it = fn_rot.begin(), fn_rot_it_end = fn_rot.end();
    for (; fn_rot_it != fn_rot_it_end; ++fn_rot_it)
    {
        Mat tmp;
        (*fn_rot_it) >> tmp;

        rotationMatrix1.push_back(tmp.clone());
    }

    FileNode fn_tr = fs["translationMatrix1"];
    FileNodeIterator fn_tr_it = fn_tr.begin(), fn_tr_it_end = fn_tr.end();
    for (; fn_tr_it != fn_tr_it_end; ++fn_tr_it)
    {
        Mat tmp;
        (*fn_tr_it) >> tmp;

        translationMatrix1.push_back(tmp.clone());
    }

    return 0;
}