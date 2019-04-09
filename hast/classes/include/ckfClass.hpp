#ifndef HAST_CKF_H
#define HAST_CKF_H

#include "genheaders.hpp"

class ckfClass
{
	private:
	public:
	// time
	double lastCKFtime;
	uint counter;
	//Inputs
	cv::Mat Mech, Aiding, PosteriorEst;
	cv::Mat Fk, PosteriorCov, Qdk, Qw, H, Rk, I;

	//Intermediates
	cv::Mat PriorEst, PriorCov;
	cv::Mat zk, Sk, Skinv, yk, Kk;
	cv::Mat Kkyk, IKkH;

	ckfClass();

	void incrementKF();

};

#endif