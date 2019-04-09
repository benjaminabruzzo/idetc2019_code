#ifndef HAST_APRILCLASS_H
#define HAST_APRILCLASS_H

#include "genheaders.hpp"

class apriltagclass
{
	private:
	public:
		/* Variables : make everything public for now */
		uint id; // serial # of marker
		uint order; // serialized occurrence of detection
		bool isgoal; // use this flag to remove april tag from obstacle setting in cost map
		uint measCount; // index of observations (for matlab data)
		uint slamCount; //index 
		bool markerFlag; // boolean flag for whether the marker is in the FOV
		cv::Mat MeasPosition_cam; // measured position of marker in camera frame
		cv::Mat MeasPosition_uav; // measured position of marker in uav frame
		cv::Mat MeasPosition_gl; // measured position of marker in global frame Hlo2gl
		cv::Mat MeasPosition_tf; // measured position of marker in global frame Hlo2gl
		cv::Mat EstPosition_uav; // estimated position in uav frame
		cv::Mat EstPosition_gl; // estimated position in global frame
		
		cv::Mat vk;
		cv::Mat PredictedMeasurement_uav; // predicted measurement of landmark from uav in uav frame
		cv::Mat PredictedMeasurement_gl; // predicted measurement of landmark from uav in global frame
		double PredictedPhi; //predicted angle of landmark in uav frame


		double MeasYaw_cam; // measured angle of marker relative to camera frame
		double MeasYaw_cam_rad; // measured angle of marker relative to camera frame in radians
		double MeasYaw_uav_rad; // measured angle of marker relative to uav frame in radians
		double MeasYaw_uav; // measured angle of marker relative to uav frame
		double MeasYaw_gl; // measured angle of marker relative to global frame
		double EstYaw_gl; // estimated yaw angle relative to global frame
		cv::Mat Position_gl;

		cv::Mat quaternion_xform; // tf quaternion
		cv::Mat Rk_uav; // Measurement covariance in uav frame
		cv::Mat Rk_gl; // Measurement covariance in global frame

		cv::Mat I, nI; //4x4 identity matrix

	// Functions

		apriltagclass(); // void for construction of KF class
		// cv::Mat getPosteriorEst() {return dKF.PosteriorEst;}
		cv::Mat getMeasPosition_uav();
		cv::Mat getMeasPosition_gl();

};


#endif


// cv::Mat R_uav;//Measurement covariance matrix in UAV frame
/* Error data from vicon lab, in uav frame:
	x: mean(x_diff) std(x_diff) 1000*var(x_diff)
	   -0.0287    0.0281    0.7913

	y: mean(y_diff) std(y_diff) 1000*var(y_diff)
	   -0.0001    0.0284    0.8093

	z: mean(z_diff) std(z_diff) 1000*var(z_diff)
	    0.0163    0.0234    0.5496

	yaw: mean(yaw_diff) std(yaw_diff) 1000*var(yaw_diff)
	    0.0358    0.0681    4.6440
*/
