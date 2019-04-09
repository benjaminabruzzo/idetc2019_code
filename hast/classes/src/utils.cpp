#include "utils.hpp"

	cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
	{//ROS_INFO("uav cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)");
	// H = [R -R*t(1:3,1); 0 0 0 1];
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


	double toDegrees(double radians){return radians*57.29577951;} //180/Pi = 57.295779513082323;
	double toRadians(double degrees){return degrees*00.01745329;} //Pi/180 = 00.017453292519943;
