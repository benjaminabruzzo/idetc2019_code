#include "hastSLAM.hpp"
hastSLAM::hastSLAM() // void for construction of KF class
{
	slamCount = 0;
	NumberOfTagsinFOV = 0;
	numberOfLandmarksOrdered = 0;
	// TagsInFOV = std::vector<uint>;
	// std::vector<int> vector1(length, 0);

	// Rk_gl = 0.001 * (cv::Mat_<double>(4, 4) << 
	// 		0.7913, 0, 0, 0,
	// 		0, 0.8093, 0, 0,
	// 		0, 0, 0.5496, 0,
	// 		0, 0, 0, 4.6440);
	// P is initialized for UAV only, must be grown as new data is added
	// P = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	I4 = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
	nI4 = -1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

	// init augmentable slam variables for uav only?
	augmentedCovariance = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

	// ROS_INFO("hastSLAM constructed: ");
}

void hastSLAM::initSLAM(double NumberOfTags)
{
	firstTimeForID.assign(NumberOfTags, 0);
	updateForID.assign(NumberOfTags, 0);
	TagsInFOV.assign(NumberOfTags, 0);

	ROS_INFO("slamMfile: %s", s_slamMfile.c_str());
	slamMfile = std::fopen (s_slamMfile.c_str(), "w");
	fprintf (slamMfile, "%% %s\n", s_slamMfile.c_str());
	fprintf (slamMfile, "%%clc; \n%%clear all;\n%%close all;\n\n");
}

void hastSLAM::incrementSLAM(cv::Mat H, cv::Mat Rk) {/* this actually happens in "void MarkerSLAM(double scantime)"*/}


void hastSLAM::augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj)
{
	// add new measurement of landmark to the augmented state (slam.est)
	augmentedState.push_back(newLandmarkPos.at<double>(0,0)); //x
	augmentedState.push_back(newLandmarkPos.at<double>(1,0)); //y
	augmentedState.push_back(newLandmarkPos.at<double>(2,0)); //z
	augmentedState.push_back(newLandmarkYaw); //yaw

	landmarksOrdered.push_back(newLandmarkID); //this is useful for tracking the number of landmarks
	++numberOfLandmarksOrdered; // this will be useful for determining size of augmented P matrix
	timeOflandmarksOrdered.push_back(ros::Time::now().toSec());

	// zero pad measurement matrix to maintain vector size
	augmentedStateMeas.push_back(0.0); // x
	augmentedStateMeas.push_back(0.0); // y
	augmentedStateMeas.push_back(0.0); // z
	augmentedStateMeas.push_back(0.0); // yaw

	// 1) create new rows
		// submatrix are the top 4 rows of the current P matrix, vconcat and hconcat don't follow zero index ideas
		cv::Mat submatrix = augmentedCovariance.colRange(0,4*numberOfLandmarksOrdered).rowRange(0,4); 

		cv::Mat newrowsA, newcolsA;
		submatrix.copyTo(newrowsA);
		// add top 4 rows to bottom of augmented matrix
		cv::vconcat(augmentedCovariance, newrowsA, augmentedCovariance);

	// 2) define new columns, then add Cm3 to bottom of new columns
		newcolsA = newrowsA.t();
		cv::Mat CMn = augmentedCovariance.colRange(0,4).rowRange(0,4)+100*Rj; 
		cv::vconcat(newcolsA, CMn, newcolsA);
	
	// 3) add new columns to right side of augmented matrix
		cv::hconcat(augmentedCovariance, newcolsA, augmentedCovariance);

}
