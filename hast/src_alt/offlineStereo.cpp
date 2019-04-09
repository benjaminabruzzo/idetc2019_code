// This node loads images from the hard drive, then publishes them as a ros-topic
// Programming tools
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <vector>

//Vision tools
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d.hpp>
// #include <features2d.hpp>
#include <sensor_msgs/image_encodings.h>


//ROS Communications
#include <ros/ros.h>
  // Messages
  #include <hast/flag.h>
  #include <hast/matrix3x3.h>
  #include <ardrone_autonomy/Navdata.h>
  #include <hast/pixels.h>
	#include <hast/ugvstate.h>
	#include <hast/uavstate.h>
  #include <hast/vector3.h>
  #include <std_msgs/Empty.h>
  #include <geometry_msgs/Twist.h>
  #include <nav_msgs/Odometry.h>
  #include <tf/transform_listener.h>

  // Services
  #include <hast/flipflop.h>
  #include <hast/stereoodomcs.h>

static const char rWindow[] = "Right Image";
static const char lWindow[] = "Left Image";

static const char Bluestr[] = "Blue";
static const char Greenstr[] = "Green";
static const char Redstr[] = "Red";


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << std::setw(5) << std::setfill('0')  << n ;
        return stm.str() ;
    }
}


class offlineStereo
{
private:
	/*--------- ROS Communication Containers ------------- */
	ros::Subscriber HastShutDown_sub;
  /*--------- Constants ------------- */
    int L2Norm;// Frobenius norm for CV norm() function
    double FocalLength, Baseline, Pi;
    cv::Scalar RGBRed, RGBBlue, RGBGreen;

/*---------  File Recorder ------------- */
  std::string s_filename, s_run, s_dotm, s_root, s_handle, s_date, s_user;
  std::FILE * pFile;
  bool stereoflag, SaveImages;

/*---------Imagery Variables ------------- */
  /*--------- Constant containers ------------- */
  //cv::GaussianBlur(src, dst, Gkx, Gky, GsigmaX, GsigmaY); // blur must be an odd window size
  int Gkx, Gky; /* Window size for mxn blurring window*/
  double GsigmaX, GsigmaY; /* Gaussian Standard Deviation in X and Y*/
  double Cx, Cy;

  /*--------- Image Containers ------------- */
  cv::Mat left_image, right_image, left_imageg, right_imageg;
  cv::Mat rCircle, lCircle;
  cv::Mat rBlob, lBlob;
  cv::Mat lContours, rContours;

  std::string s_LeftOriginal, s_RightOriginal;
  std::string s_lCircle, s_rCircle;
  std::string s_lContour, s_rContour;

  cv::Mat lYUV, lYUVg, rYUV, rYUVg;
  cv::Mat lY, lU, lV;
  cv::Mat rY, rU, rV;
  std::string s_imString;

  /*--- KeyPoint Containers --- */
  std::vector<cv::KeyPoint> lRedKeypoint, rRedKeypoint, lRedBlobpoint, rRedBlobpoint;
  std::vector<cv::KeyPoint> lBlueKeypoint, rBlueKeypoint, lBlueBlobpoint, rBlueBlobpoint;
  std::vector<cv::KeyPoint> lGreenKeypoint, rGreenKeypoint, lGreenBlobpoint, rGreenBlobpoint;

  /*--- Imagery Containers --- */
  cv::Vec<double,4> Bluepix, Greenpix, Redpix; // pixel vectors [rX rY lX lY]
  cv::Vec<double,4> RawBluepix, RawGreenpix, RawRedpix, RawPixels; // pixel vectors [rX rY lX lY]
  cv::Vec<double,2> RedRadii, BlueRadii, GreenRadii, RawRadii; // Target radii, [right left]
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

/*--------- 3D Position Containers ------------- */
  cv::Matx<double,4,1> BlueP_cam, GreenP_cam, RedP_cam, uav_p_cam;
  cv::Matx<double,3,3> Redscale, Bluescale, Greenscale;

  cv::Matx<double,3,3> BlueJ_cam, RedJ_cam, GreenJ_cam;

  cv::Matx<double,4,1> BlueP_ugv, GreenP_ugv, RedP_ugv, uav_p_ugv;
  cv::Matx<double,3,3> BlueJJt_ugv, RedJJt_ugv, GreenJJt_ugv;

  /*--------- Camera to Turtlebot Transform Matrices ------------- */
    double Wedge, cw, sw, camZOffset, camXOffset, RightOffset; //Blue wedge angle, cosine/sine of the wedge angle X/Z offset of camera axis
    cv::Matx<double,4,4> cam2tb;
    cv::Matx<double,3,3> cam2tb33, Rz;

    // Matrices for the cross product orientation uncertainty
    cv::Matx<double,4,1> GmB, RmB, BmR, znum;
    cv::Matx<double,4,1> x_axis, y_axis, z_axis; //Calculated uav axes from Circle centers
    double yaw_ugv, uav_yaw_cov;
    cv::Matx<double,3,3> uavObsR_ugv, uav_p_cov;

    // subscribing to rectified images generated from raw images
    bool from_raw;  // flag for whether or not to start subscriber

public:
  int b, numimages; // counter for image loading
	ros::NodeHandle nh;

  offlineStereo() // constructor
  {
    ROS_INFO("offlineStereo: Constructing offlineStereo() ..");
    if(nh.getParam("/hast/stereo/numimages", numimages)){} else {numimages = 20;}

    ros::param::get("~from_raw", from_raw);

    /*---------  File Recorder Initilizer ------------- */
    if(nh.getParam("/hast/user", s_user)) {} else {s_user = "turtlebot";}
    if(nh.getParam("/hast/run", s_run)){} else {s_run = "000";}
    if(nh.getParam("/hast/date", s_date)){} else {s_date = "";}
    s_handle = "offlineStereo_";
    s_root = "/home/" + s_user + "/ros/data/";
    s_dotm = ".m";
    s_filename = s_root + s_date + "/" + s_run + "/" + s_handle + s_run + s_dotm;
    ROS_INFO("offlineStereo: %s", s_filename.c_str());
    pFile = std::fopen (s_filename.c_str(),"w");

    /*--------- Math Constants ------------- */
    L2Norm = 4; // Frobenius norm for CV norm() function
    Pi  = atan(1)*4; // 3.14159...
    RGBRed = cv::Scalar(0,255,255);
    RGBBlue = cv::Scalar(255,255,0);
    RGBGreen = cv::Scalar(255,0,255);
    b=1;

    EdgeThreshold = 100;
    closingElement = getStructuringElement(2,cv::Size( 5, 5 ),cv::Point(-1,-1));

    Gkx = 5; /* Window size for mxn blurring window*/
    Gky = 5; /* Window size for mxn blurring window*/
    GsigmaX = 2.5; /* Gaussian Standard Deviation in X and Y*/
    GsigmaY = 2.5; /* Gaussian Standard Deviation in X and Y*/

    /*--------- Time, Counters, and Flags Containers ------------- */
    stereoflag = false;
    if(nh.getParam("/hast/stereo/SaveImages", SaveImages)){} else {SaveImages = false;}
    if (SaveImages){ROS_WARN("Saving Images");}


    /*--------- camera Constants ------------- */
    if(nh.getParam("/hast/stereo/FocalLength", FocalLength)){} else {FocalLength = 700;}
        fprintf (pFile,"offlineStereo.Original.FocalLength = % -6.4f;\n", FocalLength);
    if(nh.getParam("/hast/stereo/Baseline", Baseline)){} else {Baseline = 0.155;}
        fprintf (pFile,"offlineStereo.Original.Baseline = % -6.4f;\n", Baseline);
    if(nh.getParam("/hast/stereo/Cx", Cx)){} else {Cx = 400;}
        Cx+=5;
        fprintf (pFile,"offlineStereo.Original.Cx = [% -6.4f];\n", Cx);
    if(nh.getParam("/hast/stereo/Cy", Cy)){} else {Cy = 235;}
        Cy-=5;
        fprintf (pFile,"offlineStereo.Original.Cy = [% -6.4f];\n", Cy);
    if(nh.getParam("/hast/stereo/RightOffset", RightOffset)){} else {RightOffset = -100;}
        fprintf (pFile,"offlineStereo.Original.RightOffset = [% -6.4f];\n", RightOffset);

    fprintf (pFile,"offlineStereo.Original.Projection.Left(1,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n", FocalLength,     0.0,  Cx, 0.0);
    fprintf (pFile,"offlineStereo.Original.Projection.Left(2,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",     0.0, FocalLength,  Cy, 0.0);
    fprintf (pFile,"offlineStereo.Original.Projection.Left(3,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",     0.0,     0.0, 1.0, 0.0);
    fprintf (pFile,"offlineStereo.Original.Projection.Right(1,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n", FocalLength,     0.0,  Cx, RightOffset);
    fprintf (pFile,"offlineStereo.Original.Projection.Right(2,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",     0.0, FocalLength,  Cy, 0.0);
    fprintf (pFile,"offlineStereo.Original.Projection.Right(3,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",     0.0,     0.0, 1.0, 0.0);

    ROS_INFO("Original.Cx = % -6.4f", Cx);
    ROS_INFO("Original.Cy = % -6.4f", Cy);
    ROS_INFO("Original.b = % -6.4f", Baseline);
    ROS_INFO("Original.f = % -6.4f", FocalLength);

    fprintf (pFile,"offlineStereo.Baseline = % -6.4f;\n", Baseline);
    fprintf (pFile,"offlineStereo.FocalLength = % -6.4f;\n", FocalLength);

    /*--------- Camera to Turtlebot Transform Matrix ------------- */
    Wedge = 20-90; //degrees of Blue wedge supporting cameras
    if(nh.getParam("/hast/stereo/camXOffset", camXOffset)){} else {camXOffset = 0.055;}  // [m]
    if(nh.getParam("/hast/stereo/camZOffset", camZOffset)){} else {camZOffset = 0.225;}  // [m]
    cw  = cos(Wedge*Pi/180.0);
    sw  = sin(Wedge*Pi/180.0);
    cam2tb = cv::Matx44d(
                0, -cw,-sw, camXOffset,
                1,   0,  0, 0,
                0, -sw, cw, camZOffset,
                0,   0,  0, 1);
    cam2tb33 = cv::Matx33d(
                0, -cw,-sw,
                1,   0,  0,
                0, -sw, cw);
    fprintf (pFile,"offlineStereo.cam2tb(1,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",cam2tb(0,0),cam2tb(0,1),cam2tb(0,2),cam2tb(0,3));
    fprintf (pFile,"offlineStereo.cam2tb(2,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",cam2tb(1,0),cam2tb(1,1),cam2tb(1,2),cam2tb(1,3));
    fprintf (pFile,"offlineStereo.cam2tb(3,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",cam2tb(2,0),cam2tb(2,1),cam2tb(2,2),cam2tb(2,3));
    fprintf (pFile,"offlineStereo.cam2tb(4,:) = [% -6.4f, % -6.4f, % -6.4f, % -6.4f];\n",cam2tb(3,0),cam2tb(3,1),cam2tb(3,2),cam2tb(3,3));

    /*--------- Initialize 3d position containers ------------- */
    RedP_cam  = cv::Matx41d(0, 0, 0, 1);
    BlueP_cam = cv::Matx41d(0, 0, 0, 1);
    GreenP_cam  = cv::Matx41d(0, 0, 0, 1);
    uav_p_cam  = cv::Matx41d(0, 0, 0, 1);

    RedJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
    BlueJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
    GreenJ_cam  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);

    Redscale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // scale for JJt calculation, Red is the best
    // Bluescale  = cv::Matx33d(9.068,0,0, 0,8.7718,0, 0,0,1.7534); // ratio of Blue to Red [xl xr y] = [9.068 8.7718 1.7534]
    // Greenscale  = cv::Matx33d(31.8011,0,0, 0,54.8634,0, 0,0,2.4954); // ratio of Green to Red [xl xr y] = [31.8011 54.8634 2.4954]
    Bluescale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // ratio of Blue to Red [xl xr y] = [9.068 8.7718 1.7534]
    Greenscale  = cv::Matx33d(1,0,0, 0,1,0, 0,0,1); // ratio of Green to Red [xl xr y] = [31.8011 54.8634 2.4954]

    RedP_ugv   = cv::Matx41d(0, 0, 0, 1);
    BlueP_ugv  = cv::Matx41d(0, 0, 0, 1);
    GreenP_ugv = cv::Matx41d(0, 0, 0, 1);

    uav_p_ugv = cv::Matx41d(0, 0, 0, 1);
    BlueJJt_ugv  = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
    GreenJJt_ugv = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);
    RedJJt_ugv   = cv::Matx33d(0,0,0, 0,0,0, 0,0,0);

    /*---------  ------------- */
    ROS_INFO("offlineStereo: Watching..");

  }

  void incrementImages()
  {
      s_LeftOriginal = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(b) + ".png";
      // s_LeftOriginal = s_root + s_date + "/" + s_run + "/original/stereo/left_image_" + patch::to_string(b) + ".png";
      left_image = cv::imread(s_LeftOriginal, CV_LOAD_IMAGE_COLOR);
      // showImage(left_image, "left_image");
      // cv::waitKey(30);

      s_RightOriginal = s_root + s_date + "/" + s_run + "/original/right_image_" + patch::to_string(b) + ".png";
      // s_RightOriginal = s_root + s_date + "/" + s_run + "/original/stereo/right_image_" + patch::to_string(b) + ".png";
      right_image = cv::imread(s_RightOriginal, CV_LOAD_IMAGE_COLOR);
      // showImage(right_image, "right_image");
      // cv::waitKey(30);

      calculate();



      // showImage(lCircle, "lCircle");
      // showImage(rCircle, "rCircle");
      showImage(lBlob, "lBlob");
      showImage(rBlob, "rBlob");
      // showImage(lContours, "lContours");
      // showImage(rContours, "rContours");

      // showImage(lY, "lY");
      // showImage(lU, "lU");
      // showImage(lV, "lV");

      for (unsigned int i=0; i<lRedKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Red.left.radii(%d,%d+1) = % -6.4f;\n", b, i, lRedKeypoint[i].size);}
      for (unsigned int i=0; i<rRedKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Red.right.radii(%d,%d+1) = % -6.4f;\n", b, i, rRedKeypoint[i].size);}
      for (unsigned int i=0; i<lBlueKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Blue.left.radii(%d,%d+1) = % -6.4f;\n", b, i, lBlueKeypoint[i].size);}
      for (unsigned int i=0; i<rBlueKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Blue.right.radii(%d,%d+1) = % -6.4f;\n", b, i, rBlueKeypoint[i].size);}
      for (unsigned int i=0; i<lGreenKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Green.left.radii(%d,%d+1) = % -6.4f;\n", b, i, lGreenKeypoint[i].size);}
      for (unsigned int i=0; i<rGreenKeypoint.size(); i++){fprintf (pFile,"offlineStereo.Green.right.radii(%d,%d+1) = % -6.4f;\n", b, i, rGreenKeypoint[i].size);}

      if (SaveImages)
      {
        // s_imString = s_root + s_date + "/" + s_run + "/good/LEFT/lContours_" + patch::to_string(b) + ".png";
        //   imwrite(s_imString.c_str(), lContours);//, compression_params);
        s_imString = s_root + s_date + "/" + s_run + "/circles/left_circles_" + patch::to_string(b) + ".png";
          imwrite(s_imString.c_str(), lCircle);//, compression_params);

        // s_imString = s_root + s_date + "/" + s_run + "/good/RIGHT/rContours_" + patch::to_string(b) + ".png";
        //   imwrite(s_imString.c_str(), rContours);//, compression_params);
        s_imString = s_root + s_date + "/" + s_run + "/circles/right_circles_" + patch::to_string(b) + ".png";
          imwrite(s_imString.c_str(), rCircle);//, compression_params);
      }
      ros::spinOnce();
  		// ros::Duration(0.05).sleep(); // sleep for 'x' second(s).
      ++b;
      // usleep(100000);
  }

  void showImage(cv::Mat image, const char* name)
  {
     cv::namedWindow(name, 1);
     cv::imshow(name, image);
     cv::waitKey(5);
  }

  void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
  { // node cleanup for end of experiment
    if(ShutDown->flag)
      {
        ROS_INFO("offlineStereo: Offline Stereo Shutting Down...");
        ros::shutdown();
      }
  }

  void calculate()
  {//begin calculate

    /*---------images used for drawing detected Circles------------- */
    stereoflag = true;
      left_image.copyTo(lCircle);
      right_image.copyTo(rCircle);
      left_image.copyTo(lContours);
      right_image.copyTo(rContours);
      left_image.copyTo(lBlob);
      right_image.copyTo(rBlob);

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

      calcXYZ();
      crossOrientation();

  } // end calculate

  void crossOrientation()
  {
    // z_axis = (Pg - Pb) X (Pr - Pb) / || (Pg - Pb) X (Pr - Pb) ||
    // y_axis = (Pb - Pr) / || (Pb - Pr) ||
    // x_axis = y_axis X z_axis

    /* ------- Compute Axes and Yaw----------- */
    GmB = cv::Matx41d(GreenP_ugv(0,0) - BlueP_ugv(0,0), GreenP_ugv(1,0) - BlueP_ugv(1,0), GreenP_ugv(2,0) - BlueP_ugv(2,0), 0);
    RmB = cv::Matx41d(RedP_ugv(0,0) - BlueP_ugv(0,0), RedP_ugv(1,0) - BlueP_ugv(1,0), RedP_ugv(2,0) - BlueP_ugv(2,0), 0);
    BmR = cv::Matx41d(BlueP_ugv(0,0) - RedP_ugv(0,0), BlueP_ugv(1,0) - RedP_ugv(1,0), BlueP_ugv(2,0) - RedP_ugv(2,0), 0);

    znum = Cross(GmB, RmB);
    double normz = norm(znum, L2Norm);
    double normy = norm(BmR, L2Norm);
    z_axis = (1.0 / normz) * znum;
    y_axis = (1.0 / normy) * BmR;
    x_axis = Cross(y_axis, z_axis);

    yaw_ugv = atan2(x_axis(0, 1), x_axis(0, 0)) * 180 / Pi; //heading angle in degrees in tb frame

    /* ------- Propegate Covariances ----------- */
    cv::Matx<double,3,3> z_JJt, y_JJt, x_JJt;
    cv::Matx<double,3,3> z_jacobianOfNorm, y_jacobianOfNorm;
    cv::Matx<double,3,3> GandB_JJt = GreenJJt_ugv + BlueJJt_ugv;
    cv::Matx<double,3,3> RandB_JJt = RedJJt_ugv + BlueJJt_ugv;

    z_jacobianOfNorm = jacobianOfNorm(znum);
    z_JJt = wedgevector(BmR) * GandB_JJt * wedgevector(BmR).t() + wedgevector(GmB) * RandB_JJt * wedgevector(GmB).t();
    z_JJt = z_jacobianOfNorm * z_JJt * z_jacobianOfNorm.t();

    y_jacobianOfNorm = jacobianOfNorm(BmR);
    y_JJt = y_jacobianOfNorm * RandB_JJt * y_jacobianOfNorm.t();

    x_JJt = wedgevector(-z_axis) * y_JJt * wedgevector(-z_axis).t() + wedgevector(y_axis) * z_JJt * wedgevector(y_axis).t();
    cv::Matx<double,2,2> x_JJt2x2 = cv::Matx22d(
      x_JJt(0,0), x_JJt(0,1),
      x_JJt(1,0), x_JJt(1,1));

    cv::Matx<double,1,2> x_jacobianOfAtan2 = jacobianOfAtan2(x_axis(0, 0),x_axis(0, 1));
    cv::Matx<double,1,1> yaw_JJt = x_jacobianOfAtan2 * x_JJt2x2 * x_jacobianOfAtan2.t();
    uav_yaw_cov = yaw_JJt(0,0);

    fprintf (pFile,"offlineStereo.uav.z_axis(%d,:) = [% -6.4f % -6.4f % -6.4f];\n", b, z_axis(0,0), z_axis(1,0), z_axis(2,0));
    fprintf (pFile,"offlineStereo.uav.z_JJt(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, z_JJt(0,0), z_JJt(0,1), z_JJt(0,2));
    fprintf (pFile,"offlineStereo.uav.z_JJt(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, z_JJt(1,0), z_JJt(1,1), z_JJt(1,2));
    fprintf (pFile,"offlineStereo.uav.z_JJt(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, z_JJt(2,0), z_JJt(2,1), z_JJt(2,2));

    fprintf (pFile,"offlineStereo.uav.y_axis(%d,:) = [% -6.4f % -6.4f % -6.4f];\n", b, y_axis(0,0), y_axis(1,0), y_axis(2,0));
    fprintf (pFile,"offlineStereo.uav.y_JJt(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, y_JJt(0,0), y_JJt(0,1), y_JJt(0,2));
    fprintf (pFile,"offlineStereo.uav.y_JJt(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, y_JJt(1,0), y_JJt(1,1), y_JJt(1,2));
    fprintf (pFile,"offlineStereo.uav.y_JJt(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, y_JJt(2,0), y_JJt(2,1), y_JJt(2,2));

    fprintf (pFile,"offlineStereo.uav.x_axis(%d,:) = [% -6.4f % -6.4f % -6.4f];\n", b, x_axis(0,0), x_axis(1,0), x_axis(2,0));
    fprintf (pFile,"offlineStereo.uav.x_JJt(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, x_JJt(0,0), x_JJt(0,1), x_JJt(0,2));
    fprintf (pFile,"offlineStereo.uav.x_JJt(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, x_JJt(1,0), x_JJt(1,1), x_JJt(1,2));
    fprintf (pFile,"offlineStereo.uav.x_JJt(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, x_JJt(2,0), x_JJt(2,1), x_JJt(2,2));

    fprintf (pFile,"offlineStereo.uav.yaw_ugv(%d,1) = % -6.4f;\n", b, yaw_ugv);
    fprintf (pFile,"offlineStereo.uav.yaw_jacobianOfAtan2(%d,:) = [% -6.4f, % -6.4f];\n", b, x_jacobianOfAtan2(0,0), x_jacobianOfAtan2(0,1));
    fprintf (pFile,"offlineStereo.uav.uav_yaw_cov(%d,1) = % -6.4f;\n", b, uav_yaw_cov);

    ROS_INFO("[yaw cov]_ugv : [% -6.4f % -6.4f]", yaw_ugv, uav_yaw_cov);
  }


  void calcXYZ()
  {
    /* Calling this function will update the member variables that
     * Calculate the location of the target with respect to the focal
     * point of the left camera (the local coordinate frame origin)
     * Z = (b f)/ d  = ([in][px])/[px] = [in]
     * Y = Z (yl + yr) / (2 f) = ([in][px]) / ([px]) = [in]
     * X = - Z (xl + xr) / (2 f) = ([in][px]) / ([px]) = [in]
     * pix[0 1 2 3] = pix[rx ry lx ly]
     */

    /* ------ Pixels to 3D ---------*/
      RedP_cam = calcVec(Redpix); Redscale = RedP_cam(2,0) * RedP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
      BlueP_cam = calcVec(Bluepix); Bluescale = BlueP_cam(2,0) * BlueP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
      GreenP_cam = calcVec(Greenpix); Greenscale = GreenP_cam(2,0) * GreenP_cam(2,0) * cv::Matx33d(1,0,0, 0,1,0, 0,0,1);
      uav_p_cam = 0.5 * (BlueP_cam + RedP_cam);
      RedJ_cam = calcJ(Redpix);
      BlueJ_cam = calcJ(Bluepix);
      GreenJ_cam = calcJ(Greenpix);

      /* rotate to turtlebot frame ---------*/
      RedP_ugv   = cam2tb * RedP_cam;
      BlueP_ugv  = cam2tb * BlueP_cam;
      GreenP_ugv = cam2tb * GreenP_cam;
      uav_p_ugv = cam2tb * uav_p_cam;

      RedJJt_ugv = cam2tb33 * RedJ_cam * Redscale * RedJ_cam.t() * cam2tb33.t();
      BlueJJt_ugv = cam2tb33 * BlueJ_cam * Bluescale * BlueJ_cam.t() * cam2tb33.t();
      GreenJJt_ugv = cam2tb33 * GreenJ_cam * Greenscale * GreenJ_cam.t() * cam2tb33.t();
      uav_p_cov = RedJJt_ugv + BlueJJt_ugv;

      fprintf (pFile,"offlineStereo.Red.P_cam(%d,:)    = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedP_cam(0,0) , RedP_cam(1,0)   , RedP_cam(2,0));
      fprintf (pFile,"offlineStereo.Blue.P_cam(%d,:)   = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueP_cam(0,0)  , BlueP_cam(1,0)  , BlueP_cam(2,0));
      fprintf (pFile,"offlineStereo.Green.P_cam(%d,:)  = [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenP_cam(0,0) , GreenP_cam(1,0) , GreenP_cam(2,0));
      fprintf (pFile,"offlineStereo.uav.Obs_cam(%d,:)  = [% -6.4f, % -6.4f, % -6.4f];\n", b, uav_p_cam(0,0) , uav_p_cam(1,0) , uav_p_cam(2,0));
      fprintf (pFile,"offlineStereo.Red.J_cam(1,:,%d)  = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJ_cam(0,0), RedJ_cam(0,1), RedJ_cam(0,2));
      fprintf (pFile,"offlineStereo.Red.J_cam(2,:,%d)  = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJ_cam(1,0), RedJ_cam(1,1), RedJ_cam(1,2));
      fprintf (pFile,"offlineStereo.Red.J_cam(3,:,%d)  = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJ_cam(2,0), RedJ_cam(2,1), RedJ_cam(2,2));
      fprintf (pFile,"offlineStereo.Blue.J_cam(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJ_cam(0,0), BlueJ_cam(0,1), BlueJ_cam(0,2));
      fprintf (pFile,"offlineStereo.Blue.J_cam(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJ_cam(1,0), BlueJ_cam(1,1), BlueJ_cam(1,2));
      fprintf (pFile,"offlineStereo.Blue.J_cam(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJ_cam(2,0), BlueJ_cam(2,1), BlueJ_cam(2,2));
      fprintf (pFile,"offlineStereo.Green.J_cam(1,:,%d)= [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJ_cam(0,0), GreenJ_cam(0,1), GreenJ_cam(0,2));
      fprintf (pFile,"offlineStereo.Green.J_cam(2,:,%d)= [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJ_cam(1,0), GreenJ_cam(1,1), GreenJ_cam(1,2));
      fprintf (pFile,"offlineStereo.Green.J_cam(3,:,%d)= [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJ_cam(2,0), GreenJ_cam(2,1), GreenJ_cam(2,2));

      fprintf (pFile,"offlineStereo.Red.P_ugv(%d,:)   = [% -6.4f, % -6.4f %, -6.4f];\n", b, RedP_ugv(0,0)   , RedP_ugv(1,0)  , RedP_ugv(2,0));
      fprintf (pFile,"offlineStereo.Blue.P_ugv(%d,:)  = [% -6.4f, % -6.4f %, -6.4f];\n", b, BlueP_ugv(0,0)  , BlueP_ugv(1,0) , BlueP_ugv(2,0));
      fprintf (pFile,"offlineStereo.Green.P_ugv(%d,:) = [% -6.4f, % -6.4f %, -6.4f];\n", b, GreenP_ugv(0,0) , GreenP_ugv(1,0)  , GreenP_ugv(2,0));
      fprintf (pFile,"offlineStereo.uav.P_ugv(%d,:)   = [% -6.4f, % -6.4f %, -6.4f];\n", b, uav_p_ugv(0,0) , uav_p_ugv(1,0)  , uav_p_ugv(2,0));
      fprintf (pFile,"offlineStereo.Red.JJt_ugv(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJJt_ugv(0,0), RedJJt_ugv(0,1), RedJJt_ugv(0,2));
      fprintf (pFile,"offlineStereo.Red.JJt_ugv(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJJt_ugv(1,0), RedJJt_ugv(1,1), RedJJt_ugv(1,2));
      fprintf (pFile,"offlineStereo.Red.JJt_ugv(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, RedJJt_ugv(2,0), RedJJt_ugv(2,1), RedJJt_ugv(2,2));
      fprintf (pFile,"offlineStereo.Blue.JJt_ugv(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJJt_ugv(0,0), BlueJJt_ugv(0,1), BlueJJt_ugv(0,2));
      fprintf (pFile,"offlineStereo.Blue.JJt_ugv(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJJt_ugv(1,0), BlueJJt_ugv(1,1), BlueJJt_ugv(1,2));
      fprintf (pFile,"offlineStereo.Blue.JJt_ugv(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, BlueJJt_ugv(2,0), BlueJJt_ugv(2,1), BlueJJt_ugv(2,2));

      fprintf (pFile,"offlineStereo.Green.JJt_ugv(1,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJJt_ugv(0,0), GreenJJt_ugv(0,1), GreenJJt_ugv(0,2));
      fprintf (pFile,"offlineStereo.Green.JJt_ugv(2,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJJt_ugv(1,0), GreenJJt_ugv(1,1), GreenJJt_ugv(1,2));
      fprintf (pFile,"offlineStereo.Green.JJt_ugv(3,:,%d) = [% -6.4f, % -6.4f, % -6.4f];\n", b, GreenJJt_ugv(2,0), GreenJJt_ugv(2,1), GreenJJt_ugv(2,2));


  }//end void calcXYZ

  void isRed()
  {
    int ulow = 140;
    int uhigh = 255;

    int vlow = 00;
    int vhigh = 100;

    cv::Mat LeftisRed_Mat, LeftisRed_U, LeftisRed_V;
    cv::Mat RightisRed_Mat, RightisRed_U, RightisRed_V;

      /*--- Filter by Color --- */
    cv::inRange( lU, ulow, uhigh, LeftisRed_U);
    cv::inRange( rU, ulow, uhigh, RightisRed_U);

    cv::inRange( lV, vlow, vhigh, LeftisRed_V);
    cv::inRange( rV, vlow, vhigh, RightisRed_V);

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
    /*--- Find Blobs --- */
    lRedBlobpoint = Blobbing(leftRedLEDclosed);
    rRedBlobpoint = Blobbing(rightRedLEDclosed);
    /*--- Draw Results --- */
    drawCircles(lRedKeypoint,lCircle,RGBBlue);
    drawCircles(rRedKeypoint,rCircle,RGBBlue);
    drawPolygons(lRedPolygons,lContours,RGBRed);
    drawPolygons(rRedPolygons,rContours,RGBRed);
    drawBlobs(rRedBlobpoint, lRedBlobpoint, cv::Scalar(0,255,255));

    /*--- Populate Pixel Array Results --- */
    Redpix    = testRadii(rRedKeypoint,   lRedKeypoint,   Redstr);
    // Redpix    = testPix(rRedKeypoint,   lRedKeypoint,   Redstr);

    /*--- Populate Pixel Array Results from Blobbing--- */
    // Redpix    = testPix(rRedBlobpoint,   lRedBlobpoint,   Redstr);
    //right pixels and radius
    RedRadii[0] = RawRadii[0];
    RawRedpix[0] = RawPixels[0]; //rx
    RawRedpix[1] = RawPixels[1]; //ry
    //left pixels and radius
    RedRadii[1] = RawRadii[1];
    RawRedpix[2] = RawPixels[2]; //lx
    RawRedpix[3] = RawPixels[3]; //ly

    fprintf (pFile,"offlineStereo.Red.right.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Redpix[0], Redpix[1]);
    fprintf (pFile,"offlineStereo.Red.right.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawRedpix[0], RawRedpix[1]);
    fprintf (pFile,"offlineStereo.Red.right.radius(%d,:) = % -6.4f;\n", b, RedRadii[0]);
    fprintf (pFile,"offlineStereo.Red.left.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Redpix[2], Redpix[3]);
    fprintf (pFile,"offlineStereo.Red.left.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawRedpix[2], RawRedpix[3]);
    fprintf (pFile,"offlineStereo.Red.left.radius(%d,:) = % -6.4f;\n", b, RedRadii[1]);
  }//end isRed()

  void isBlue()
  {
    int vlow = 125;
    int vhigh = 255;

    int ulow = 0;
    int uhigh = 110;

    cv::Mat LeftisBlue_Mat, LeftisBlue_U, LeftisBlue_V;
    cv::Mat RightisBlue_Mat, RightisBlue_U, RightisBlue_V;

      /*--- Filter by Color --- */
    cv::inRange( lU, ulow, uhigh, LeftisBlue_U);
    cv::inRange( rU, ulow, uhigh, RightisBlue_U);

    cv::inRange( lV, vlow, vhigh, LeftisBlue_V);
    cv::inRange( rV, vlow, vhigh, RightisBlue_V);

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
    /*--- Find Blobs --- */
    lBlueBlobpoint = Blobbing(leftBlueLEDclosed);
    rBlueBlobpoint = Blobbing(rightBlueLEDclosed);
    /*--- Draw Results --- */
    drawCircles(lBlueKeypoint,lCircle,RGBBlue);
    drawCircles(rBlueKeypoint,rCircle,RGBBlue);
    drawPolygons(lBluePolygons,lContours,RGBBlue);
    drawPolygons(rBluePolygons,rContours,RGBBlue);
    drawBlobs(rBlueBlobpoint, lBlueBlobpoint, cv::Scalar(255,255,0));

    /*--- Populate Pixel Array Results --- */
    Bluepix   = testRadii(rBlueKeypoint,  lBlueKeypoint,  Bluestr);
    // Bluepix   = testPix(rBlueKeypoint,  lBlueKeypoint,  Bluestr);
    /*--- Populate Pixel Array Results from Blobbing--- */
    // Bluepix   = testPix(rBlueBlobpoint,  lBlueBlobpoint,  Bluestr);
    //right pixels and radius
      BlueRadii[0] = RawRadii[0];
      RawBluepix[0] = RawPixels[0];
      RawBluepix[1] = RawPixels[1];
    //left pixels and radius
      BlueRadii[1] = RawRadii[1];
      RawBluepix[2] = RawPixels[2];
      RawBluepix[3] = RawPixels[3];
      fprintf (pFile,"offlineStereo.Blue.right.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Bluepix[0], Bluepix[1]);
      fprintf (pFile,"offlineStereo.Blue.right.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawBluepix[0], RawBluepix[1]);
      fprintf (pFile,"offlineStereo.Blue.right.radius(%d,:) = % -6.4f;\n", b, BlueRadii[0]);
      fprintf (pFile,"offlineStereo.Blue.left.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Bluepix[2], Bluepix[3]);
      fprintf (pFile,"offlineStereo.Blue.left.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawBluepix[2], RawBluepix[3]);
      fprintf (pFile,"offlineStereo.Blue.left.radius(%d,:) = % -6.4f;\n", b, BlueRadii[1]);

  }//end isBlue()

  void isGreen()
  {
    int vlow = 10;
    int vhigh = 100;

    int ulow = 0;
    int uhigh = 110;

    cv::Mat LeftisGreen_Mat, LeftisGreen_U, LeftisGreen_V;
    cv::Mat RightisGreen_Mat, RightisGreen_U, RightisGreen_V;

      /*--- Filter by Color --- */
    cv::inRange( lU, ulow, uhigh, LeftisGreen_U);
    cv::inRange( rU, ulow, uhigh, RightisGreen_U);

    cv::inRange( lV, vlow, vhigh, LeftisGreen_V);
    cv::inRange( rV, vlow, vhigh, RightisGreen_V);

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
    /*--- Find Blobs --- */
    lGreenBlobpoint = Blobbing(leftGreenLEDclosed);
    rGreenBlobpoint = Blobbing(rightGreenLEDclosed);
    /*--- Draw Results --- */
    drawCircles(lGreenKeypoint,lCircle,RGBGreen);
    drawCircles(rGreenKeypoint,rCircle,RGBGreen);
    drawPolygons(lGreenPolygons,lContours,RGBGreen);
    drawPolygons(rGreenPolygons,rContours,RGBGreen);
		drawBlobs(rGreenBlobpoint, lGreenBlobpoint, cv::Scalar(255,0,255));

    /*--- Populate Pixel Array Results --- */
      Greenpix  = testRadii(rGreenKeypoint, lGreenKeypoint, Greenstr);
      // Greenpix  = testPix(rGreenKeypoint, lGreenKeypoint, Greenstr);
      /*--- Populate Pixel Array Results from Blobbing--- */
      // Greenpix  = testPix(rGreenBlobpoint, lGreenBlobpoint, Greenstr);
      //right pixels and radius
        GreenRadii[0] = RawRadii[0];
        RawGreenpix[0] = RawPixels[0];
        RawGreenpix[1] = RawPixels[1];
      //left pixels and radius
        GreenRadii[1] = RawRadii[1];
        RawGreenpix[2] = RawPixels[2];
        RawGreenpix[3] = RawPixels[3];
    fprintf (pFile,"offlineStereo.Green.right.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Greenpix[0], Greenpix[1]);
    fprintf (pFile,"offlineStereo.Green.right.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawGreenpix[0], RawGreenpix[1]);
    fprintf (pFile,"offlineStereo.Green.right.radius(%d,:) = % -6.4f;\n", b, GreenRadii[0]);
    fprintf (pFile,"offlineStereo.Green.left.xy(%d,:) = [% -6.4f % -6.4f];\n", b, Greenpix[2], Greenpix[3]);
    fprintf (pFile,"offlineStereo.Green.left.rawxy(%d,:) = [% -6.4f % -6.4f];\n", b, RawGreenpix[2], RawGreenpix[3]);
    fprintf (pFile,"offlineStereo.Green.left.radius(%d,:) = % -6.4f;\n", b, GreenRadii[1]);
  } //end isGreen()

  cv::Vec<double,4> testRadii(std::vector<cv::KeyPoint> kpr, std::vector<cv::KeyPoint> kpl, const char* color)
  {
    cv::Vec<double,4> pix;

    if (kpl.size() > 0)
    {
      int bestLeftRadius = 0; // assume first radius is best
      int bestLeftIndex = 0; // assume first radius is best
      if (kpl.size() > 1)
      {//begin if (kpr.size() > 1)
        for (unsigned int i=0; i<kpl.size(); i++)
        {
          int Radius = kpl[i].size;
          if (Radius > bestLeftRadius)
          { // only change indices if the radius is better
            bestLeftRadius = Radius;
            bestLeftIndex = i;
          } // if (Radius > bestRightRadius)
        }//end for (unsigned int i=0; i<kpr.size(); i++)
      }//end if (kpr.size() > 1)

      pix[2] = kpl[bestLeftIndex].pt.x - Cx;
      pix[3] = Cy - kpl[bestLeftIndex].pt.y;
      RawRadii[1] = kpl[bestLeftIndex].size;
      RawPixels[2] = kpl[bestLeftIndex].pt.x;
      RawPixels[3] = kpl[bestLeftIndex].pt.y;
    } else { // else if ( 0 < kpl[i].size)
      stereoflag = false;
      pix[2] = 0;
      pix[3] = 0;
      RawRadii[1] = 0;
      RawPixels[2] = 0;
      RawPixels[3] = 0;
    } // end if ( 0 < kpl[i].size)

    if (kpr.size() > 0)
    {
      int bestRightRadius = 0; // assume first radius is best
      int bestRightIndex = 0; // assume first radius is best
      if (kpr.size() > 1)
      {//begin if (kpr.size() > 1)
        for (unsigned int i=0; i<kpr.size(); i++)
        {
          int Radius = kpr[i].size;
          if (Radius > bestRightRadius)
          { // only change indices if the radius is better
            bestRightRadius = Radius;
            bestRightIndex = i;
          } // if (Radius > bestRightRadius)
        }//end for (unsigned int i=0; i<kpr.size(); i++)
      }//end if (kpr.size() > 1)
      pix[0] = kpr[bestRightRadius].pt.x - Cx;
      pix[1] = Cy - kpr[bestRightRadius].pt.y;
      RawPixels[0] = kpr[bestRightRadius].pt.x;
      RawPixels[1] = kpr[bestRightRadius].pt.y;
      RawRadii[0] = kpr[bestRightRadius].size;
    } else { //else if ( 0 < kpr[i].size)
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
          // ROS_INFO("offlineStereo: Error: Detected %i %s Circles in left image", int(kpl.size()), color);
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
        // ROS_INFO("offlineStereo: Error: Detected %i %s Circles in right image", int(kpr.size()), color);
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
    cv::threshold( src_image, threshold_output, EdgeThreshold, 255, cv::THRESH_BINARY );

    /// Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    // cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
    // cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    // cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, cv::Point(0, 0) );
    // cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

    return contours;
  }

  std::vector<std::vector<cv::Point> > getPolygons(std::vector<std::vector<cv::Point> > contours)
  {
    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > polygons( contours.size() );
    double eps = 0.05; // Parameter specifying the approximation accuracy. This is the maximum distance between the original curve and its approximation.
    for( size_t i = 0; i < contours.size(); i++ )
    {cv::approxPolyDP( cv::Mat(contours[i]), polygons[i], eps, true );}
    return polygons;
  } //end getPolygons

  void drawPolygons(std::vector<std::vector<cv::Point> > polygons, cv::Mat dest_image,cv::Scalar color )
  {
    for( size_t i = 0; i < polygons.size(); i++ )
    {cv::drawContours(dest_image, polygons, int(i), color , 1,8,std::vector<cv::Vec4i>(), 0, cv::Point());}
  } //end drawPolygons

  std::vector<cv::KeyPoint> getCircles(std::vector<std::vector<cv::Point> > polygons)
  {
    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f>center( polygons.size() );
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
        } //end for (int i=0; i<keypoints.size(); i++)
  } //end drawCircles

	std::vector<cv::KeyPoint> Blobbing(cv::Mat BlobD)
	{
	  /*--- Detect Blobs --- */
		cv::SimpleBlobDetector::Params params;
			params.minDistBetweenBlobs = 15;
			params.filterByColor = true; //find blobs by color
			params.blobColor = 255; // 255 = white
			params.filterByCircularity = false;
			params.filterByInertia = false;
			params.filterByConvexity = false;
			params.filterByArea = true; //find blobs between max/min
			params.minArea = 10; // 25 pixel min size for blob

		cv::SimpleBlobDetector blobDetector( params );
		blobDetector.create("SimpleBlob");
		std::vector<cv::KeyPoint> keypoints;
		blobDetector.detect(BlobD, keypoints);
	  return keypoints;
	} //end Blobbing

	void drawBlobs(std::vector<cv::KeyPoint> kpr, std::vector<cv::KeyPoint> kpl, cv::Scalar color)
	{
		for (unsigned int i=0; i<kpr.size(); i++)
			{
			cv::Point rCenter(cvRound(kpr[i].pt.x), cvRound(kpr[i].pt.y));
			int rRadius = cvRound(kpr[i].size);
			cv::circle(rBlob, rCenter, 1, color, -1, 8, 0);
			cv::circle(rBlob, rCenter, rRadius, color, 1, 8, 0);
			} //end for (int i=0; i<keypoints.size(); i++)

		for (unsigned int i=0; i<kpl.size(); i++)
			{
			cv::Point lCenter(cvRound(kpl[i].pt.x), cvRound(kpl[i].pt.y));
			int lRadius = cvRound(kpl[i].size);
			cv::circle(lBlob, lCenter, 1, color, -1, 8, 0);
			cv::circle(lBlob, lCenter, lRadius, color, 1, 8, 0);
			} //end for (int i=0; i<keypoints.size(); i++)
	}

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
    // z position
    Vecpix(2,0) = (Baseline*FocalLength) / disparity;
    //Non-Matthies left-y
    Vecpix(1,0) =  Vecpix(2,0) * (Pix[1] + Pix[3] ) / ( 2 * FocalLength );
    // x values are negative becuase of the way tirangulation is derived
    Vecpix(0,0) = -Vecpix(2,0) * (Pix[0] + Pix[2] ) / ( 2 * FocalLength );
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

  cv::Matx<double,1,2> jacobianOfAtan2(double a, double b)
  {
    double A = 1 / (a*a+b*b);
    cv::Matx12d px;
    px = A*cv::Matx12d(-b,a);
    return px;
  }

}; //end of class definition

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offlineStereo");
	offlineStereo oS;
  // while (oS.nh.ok())
  while (oS.b <= oS.numimages)
  {oS.incrementImages();}
  ROS_WARN("Last image (%i) done, closing.", oS.numimages);
	return 0;
}
