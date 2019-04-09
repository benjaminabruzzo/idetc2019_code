#include "apriltagclass.hpp"

apriltagclass::apriltagclass() // void for construction of KF class
{
	id = 0;

	isgoal = false;
	MeasPosition_cam = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	MeasPosition_uav = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	MeasPosition_gl = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	
	PredictedMeasurement_uav = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	PredictedMeasurement_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	PredictedPhi = 0;



	quaternion_xform = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

	measCount = 0;
	MeasYaw_cam = 0;
	MeasYaw_uav = 0;
	MeasYaw_gl = 0;
	EstYaw_gl = 0; //radians
	slamCount = 0;
	vk = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);



	// Rk_uav = 0.000000001 * (cv::Mat_<double>(4, 4) << 
	// 	0.7913, 0, 0, 0,
	// 	0, 0.8093, 0, 0,
	// 	0, 0, 0.5496, 0,
	// 	0, 0, 0, 4.6440);
	Rk_uav = 0.01 * (cv::Mat_<double>(4, 4) << 
		0.1, 0, 0, 0,
		0, 0.1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 0.1);

	I = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	nI = -1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
}


// cv::Mat getPosteriorEst() {return dKF.PosteriorEst;}
cv::Mat apriltagclass::getMeasPosition_uav() {return MeasPosition_uav;}
cv::Mat apriltagclass::getMeasPosition_gl() {return MeasPosition_gl;}

