#ifndef HAST_SLAM_H
#define HAST_SLAM_H

#include "genheaders.hpp"

class hastSLAM
{
	private:

	public:
		std::FILE * slamMfile;
		std::string s_slamMfile;

		// cv::Mat Rk_gl; // Measurement covariance in global frame
		cv::Mat I4, nI4; // identity and negative negative matrix

		cv::Mat HPHt; // HPHt = H*P*H'
		cv::Mat Sk; // Innovation covariance, Sk = HPHt + Rk_slam
		cv::Mat PHt; // PHtr = P_slam*H' 
		cv::Mat Kk; // Kalman Gain, // K = PHtr * Sk
		cv::Mat vk; // difference between measured and estimated landmark poses
		cv::Mat correction; // correction to apply to augmentedState

		// Trying to do dynamic state and matrix growth
		std::vector<int> landmarksOrdered;  // Landmark ID in order of observation
		std::vector<double> timeOflandmarksOrdered;  // Landmark ID in order of observation
		uint numberOfLandmarksOrdered;
		std::vector<double> augmentedState; // robot plus landmarks : estimated positions
		std::vector<double> augmentedStateMeas; // robot plus landmarks : current measurements
		cv::Mat augmentedCovariance;

		uint slamCount;

		std::vector<int> firstTimeForID, updateForID;
		std::vector<uint> TagsInFOV;
		uint NumberOfTagsinFOV, registered;

		hastSLAM(); // void for construction of KF class

		void initSLAM(double NumberOfTags);

		void incrementSLAM(cv::Mat H, cv::Mat Rk);

		void augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj);
		
};

#endif