#include "DiscreteKF.hpp"

DiscreteKF::DiscreteKF(){}  // void for construction of KF class

void DiscreteKF::incrementKF()
{
	// ROS_WARN("ckfRecorder: incrementKF");
	// Prediction
	PriorEst = Fk * PosteriorEst + Dk;
	PriorCov = Fk * PosteriorCov * Fk.t() + Qk;

	// Innovation
	yk = zk - H * PriorEst;
	Sk = H * PriorCov * H.t() + Rk;
	Kk = PriorCov * H.t() * Sk.inv();

	//Estimate
	Kkyk = Kk * yk;
	PosteriorEst = PriorEst + Kk * yk;
	PosteriorCov = (I - Kk * H) * PriorCov;
}
