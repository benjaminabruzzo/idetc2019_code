#ifndef HAST_DKF_H
#define HAST_DKF_H

#include "genheaders.hpp"

class DiscreteKF
{
	private:
	public:
	//Inputs
	cv::Mat H, zk; // kth measurement and measurement matrix
	cv::Mat PosteriorEst; // last estimate
	cv::Mat Dk; // disturbance?
	cv::Mat Fk, PosteriorCov, Qk, Rk, I;

	//Intermediates
	cv::Mat PriorEst, PriorCov;
	cv::Mat Sk, yk, Kk;
	cv::Mat Kkyk;

	DiscreteKF();

	void incrementKF();
};


#endif