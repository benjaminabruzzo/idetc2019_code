#include "genheaders.hpp"

namespace enc = sensor_msgs::image_encodings;
static const char downWindow[] = "Down Image";

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << std::setw(5) << std::setfill('0')  << n ;
        return stm.str() ;
    }
}


class threshImage
{
private:
  /*--------- ROS Communication Containers ------------- */
  ros::NodeHandle n;
    /*----- image transport Channels */
    image_transport::ImageTransport it;
    image_transport::Subscriber Image_sub;

    ros::Subscriber HastShutDown_sub;

  /*---------  File Recorder ------------- */
    std::string s_imString, s_run, s_root, s_date, s_user;
  /*--------- Time, Counters, and Flags Containers ------------- */
    bool newdown, SaveImages;
    int calledCount;
    cv::Mat bwimage, down_image;

public:
  threshImage()
  : it(n)
  {
    ROS_INFO("Constructing threshImage() ..");
    /*--------- Initialize ROS Communication ------------- */
    /*-----  Publishers and Subscribers */
    Image_sub   = it.subscribe("/ardrone/bottom/image_raw",  1,  &threshImage::downCvBridge, this);
    HastShutDown_sub = n.subscribe("/hast/shutdown",           10, &threshImage::nodeShutDown, this);

    if(n.getParam("/hast/user", s_user)) {} else {s_user = "turtlebot";}
    if(n.getParam("/hast/run", s_run)){} else {s_run = "000";}
    if(n.getParam("/hast/date", s_date)){} else {s_date = "2017";}
    if(n.getParam("/hast/stereo/SaveImages", SaveImages)){} else {SaveImages = false;}
    if (SaveImages){ROS_WARN("Saving Images");}

    s_root = "/home/" + s_user + "/ros/data/";
    calledCount = 0;

  }//close constructor

  ~threshImage()
  {
    cv::destroyWindow(downWindow);
  }//close destructor


  void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
  { // node cleanup for end of experiment
    if(ShutDown->flag)
      {
        ROS_INFO("Shutting Down...");
        ros::shutdown();
      }
  }

  void downCvBridge(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    // {cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
    {cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);}
    catch (cv_bridge::Exception& e)
    {ROS_ERROR("cv_bridge exception: %s", e.what());return;}
    down_image=cv_ptr->image;
    newdown = true;
    // ROS_INFO("newdown = true;");
    showImage(down_image, "MONO8");

    threshold( down_image, bwimage, 100, 255,0 );
    showImage(bwimage, "bwimage");
    if (SaveImages)
    {
      s_imString = s_root + s_date + "/" + s_run + "/original/down_image_" + patch::to_string(++calledCount) + ".png";
        imwrite(s_imString.c_str(), down_image);
    }
  } // end void CBr

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

}; //end threshImage class

int main(int argc, char **argv)
{
  ros::init(argc, argv, "threshImage");
  threshImage dP;
  ros::spin();
  return 0;
}
