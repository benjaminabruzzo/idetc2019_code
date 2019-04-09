#include "ckfClass.hpp"

ckfClass::ckfClass()
{
	lastCKFtime = 0;
} // void for construction of KF class

void ckfClass::incrementKF()
{
	// % Prediction
	PriorEst = Fk * PosteriorEst;
	PriorCov = Fk * PosteriorCov * Fk.t() + Qdk;

// % Innovation
	zk = H * (Aiding - Mech);
	yk = zk - H * PriorEst;
	Sk = H * PriorCov * H.t() + Rk;
	Skinv = Sk.inv();
	Kk = PriorCov * H.t() * Skinv;

// % Estimate
	Kkyk = Kk * yk;
	PosteriorEst = PriorEst + Kk * yk;
	IKkH = (I - Kk * H);
	PosteriorCov = IKkH * PriorCov * IKkH.t() + Kk * Rk * Kk.t();
}
