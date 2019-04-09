#include "hastUAV.hpp"

hastUAV::hastUAV() // void for construction of class
{
	// ROS comms
	stateMsg_id = 0;
	pose_msg_seq = 0;

	// Possibly useful constants
	Pi = atan(1) * 4; // 3.14159...
	L2Norm = 4; // Frobenius norm for CV norm() function

	// Initial estimated states of uav
	EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	EstPositionHom_g = (cv::Mat_<double>(4, 1) << EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0),1);

	correction_ugv = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
	yawCorrection = 0;

	EstYaw_gl = 0;
	EstYawBias = 0;			
		cosyaw = cos(Pi * EstYaw_gl / 180);
		sinyaw = sin(Pi * EstYaw_gl / 180);
		Rgl2lo = (cv::Mat_<double>(3, 3) <<
					   cosyaw, sinyaw, 0,
					  -sinyaw, cosyaw, 0,
					   0, 0, 1);
		Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
				   cosyaw, sinyaw, 0, 0,
				  -sinyaw, cosyaw, 0, 0,
				   0, 0, 1, 0,
				   0, 0, 0, 1);

		H_cam2uav = (cv::Mat_<double>(4, 4) <<
					 0,-1, 0, 0.125,
					-1, 0, 0, 0,
				     0, 0,-1, 0,
				     0, 0, 0, 1);

		// Hgl2lo transforms a point in the global frame into the tb frame
		Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
		Hlo2gl = invertH(Hgl2lo);

		EstPositionHom_lo = Hgl2lo * EstPositionHom_g;
		EstPosition_lo = (cv::Mat_<double>(3, 1) << EstPositionHom_lo.at<double>(0,0), EstPositionHom_lo.at<double>(1,0), EstPositionHom_lo.at<double>(2,0));

	// Navdata variables
	flightState = 0;
	liftoffSwitch = false;
	positionFromTags = false;

	/*----- Altitude Conatiners*/
	echoAlt 	= 0;
	echoAltLast = 0;
	deltaAlt 	= 0;

	compassYaw = 0;
	compassYawLast = 0;
	compassYawDelta = 0;
	compassCounter = 0;
	yaw_drift = 0;
	yaw_drift_rate = 0;
	// yaw_drift_rate = 0.3;

	/*-----  Initialize Time Variables */
	navStamp = ros::Time::now();
	navTS = navStamp.toSec();
	navTSpublished 	= navStamp.toSec();
	navHeadSeq = 0;
	navDataCounter = 0;
	estCounter = 0;
	cmdcount = 0;

	//complementary KF class init for uav
	observedByStereo = false;
	ckf.Mech = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
	ckf.Aiding = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
	ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0);
	// 0.1 should be less elastic / overshooty
	// do not use 0.001 anymore !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ckf.Qw = 0.1*(cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
	ckf.Fk = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
	// 0.001 is what was used on 20170126, 20170210, 20170317:
	// do not use 0.001 anymore !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// ckf.Qw = 0.001*(cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
	// do not use 0.001 anymore !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ckf.H = (cv::Mat_<double>(4, 5) <<
									1,0,0,0,0,
									0,1,0,0,0,
									0,0,1,0,0, 
									0,0,0,1,1); // !!! 4x5 measurement matrix
	ckf.Rk = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);// !!! 4x4 stereo covariance matrix
	ckf.I = (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
}

void hastUAV::openmFile()
{
	ros::Duration(0.1).sleep();
	mFile = std::fopen (s_filename.c_str(), "w");
	ROS_INFO("ckfRecorder::uavClass: %s", s_filename.c_str());
	ros::Duration(0.2).sleep();

	fprintf (mFile, "uavRecorder.ckfinit.yaw_drift_rate = % -6.14f;\n", yaw_drift_rate);
	fprintf (mFile, "uavRecorder.ckfinit.Mech = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", ckf.Mech.at<double>(0,0), ckf.Mech.at<double>(1,0), ckf.Mech.at<double>(2,0), ckf.Mech.at<double>(3,0), ckf.Mech.at<double>(4,0));
	fprintf (mFile, "uavRecorder.ckfinit.Aiding = [% -6.14f % -6.14f % -6.14f % -6.14f % -6.14f];\n", ckf.Aiding.at<double>(0,0), ckf.Aiding.at<double>(1,0), ckf.Aiding.at<double>(2,0), ckf.Aiding.at<double>(3,0), ckf.Aiding.at<double>(4,0));

	fprintf (mFile, "uavRecorder.ckfinit.Qdk(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(0, 0), ckf.Qdk.at<double>(0, 1), ckf.Qdk.at<double>(0, 2), ckf.Qdk.at<double>(0, 3), ckf.Qdk.at<double>(0, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qdk(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(1, 0), ckf.Qdk.at<double>(1, 1), ckf.Qdk.at<double>(1, 2), ckf.Qdk.at<double>(1, 3), ckf.Qdk.at<double>(1, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qdk(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(2, 0), ckf.Qdk.at<double>(2, 1), ckf.Qdk.at<double>(2, 2), ckf.Qdk.at<double>(2, 3), ckf.Qdk.at<double>(2, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qdk(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(3, 0), ckf.Qdk.at<double>(3, 1), ckf.Qdk.at<double>(3, 2), ckf.Qdk.at<double>(3, 3), ckf.Qdk.at<double>(3, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qdk(5,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(4, 0), ckf.Qdk.at<double>(4, 1), ckf.Qdk.at<double>(4, 2), ckf.Qdk.at<double>(4, 3), ckf.Qdk.at<double>(4, 4));

	fprintf (mFile, "uavRecorder.ckfinit.Qw(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(0, 0), ckf.Qw.at<double>(0, 1), ckf.Qw.at<double>(0, 2), ckf.Qw.at<double>(0, 3), ckf.Qw.at<double>(0, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qw(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(1, 0), ckf.Qw.at<double>(1, 1), ckf.Qw.at<double>(1, 2), ckf.Qw.at<double>(1, 3), ckf.Qw.at<double>(1, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qw(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(2, 0), ckf.Qw.at<double>(2, 1), ckf.Qw.at<double>(2, 2), ckf.Qw.at<double>(2, 3), ckf.Qw.at<double>(2, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qw(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(3, 0), ckf.Qw.at<double>(3, 1), ckf.Qw.at<double>(3, 2), ckf.Qw.at<double>(3, 3), ckf.Qw.at<double>(3, 4));
	fprintf (mFile, "uavRecorder.ckfinit.Qw(5,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(4, 0), ckf.Qw.at<double>(4, 1), ckf.Qw.at<double>(4, 2), ckf.Qw.at<double>(4, 3), ckf.Qw.at<double>(4, 4));

	fprintf (mFile, "uavRecorder.ckfinit.Rk(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.Rk.at<double>(0, 0),ckf.Rk.at<double>(0, 1),ckf.Rk.at<double>(0, 2),ckf.Rk.at<double>(0, 3));
	fprintf (mFile, "uavRecorder.ckfinit.Rk(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.Rk.at<double>(1, 0),ckf.Rk.at<double>(1, 1),ckf.Rk.at<double>(1, 2),ckf.Rk.at<double>(1, 3));
	fprintf (mFile, "uavRecorder.ckfinit.Rk(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.Rk.at<double>(2, 0),ckf.Rk.at<double>(2, 1),ckf.Rk.at<double>(2, 2),ckf.Rk.at<double>(2, 3));
	fprintf (mFile, "uavRecorder.ckfinit.Rk(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.Rk.at<double>(3, 0),ckf.Rk.at<double>(3, 1),ckf.Rk.at<double>(3, 2),ckf.Rk.at<double>(3, 3));

	fprintf (mFile, "uavRecorder.ckfinit.H(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.H.at<double>(0, 0),ckf.H.at<double>(0, 1),ckf.H.at<double>(0, 2),ckf.H.at<double>(0, 3),ckf.H.at<double>(0, 4));
	fprintf (mFile, "uavRecorder.ckfinit.H(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.H.at<double>(1, 0),ckf.H.at<double>(1, 1),ckf.H.at<double>(1, 2),ckf.H.at<double>(1, 3),ckf.H.at<double>(1, 4));
	fprintf (mFile, "uavRecorder.ckfinit.H(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",ckf.H.at<double>(2, 0),ckf.H.at<double>(2, 1),ckf.H.at<double>(2, 2),ckf.H.at<double>(2, 3),ckf.H.at<double>(2, 4));
	fprintf (mFile, "uavRecorder.ckfinit.H(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n",ckf.H.at<double>(3, 0),ckf.H.at<double>(3, 1),ckf.H.at<double>(3, 2),ckf.H.at<double>(3, 3),ckf.H.at<double>(3, 4));

	fprintf (mFile, "uavRecorder.H_cam2uav(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",H_cam2uav.at<double>(0, 0),H_cam2uav.at<double>(0, 1),H_cam2uav.at<double>(0, 2),H_cam2uav.at<double>(0, 3),H_cam2uav.at<double>(0, 4));
	fprintf (mFile, "uavRecorder.H_cam2uav(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",H_cam2uav.at<double>(1, 0),H_cam2uav.at<double>(1, 1),H_cam2uav.at<double>(1, 2),H_cam2uav.at<double>(1, 3),H_cam2uav.at<double>(1, 4));
	fprintf (mFile, "uavRecorder.H_cam2uav(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n",H_cam2uav.at<double>(2, 0),H_cam2uav.at<double>(2, 1),H_cam2uav.at<double>(2, 2),H_cam2uav.at<double>(2, 3),H_cam2uav.at<double>(2, 4));
	fprintf (mFile, "uavRecorder.H_cam2uav(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n",H_cam2uav.at<double>(3, 0),H_cam2uav.at<double>(3, 1),H_cam2uav.at<double>(3, 2),H_cam2uav.at<double>(3, 3),H_cam2uav.at<double>(3, 4));

	ros::Duration(0.1).sleep();
}

void hastUAV::readCmdVel(const geometry_msgs::Twist::ConstPtr& vsub)
{//ROS_INFO("uav void readCmdVel(const geometry_msgs::Twist::ConstPtr& vsub)");
	velStamp = ros::Time::now().toSec();
	cmdTwist = (cv::Mat_<double>(4, 1) <<
								vsub -> linear.x,
								vsub -> linear.y,
								vsub -> linear.z,
								vsub -> angular.z);
	cmdcount++;
	fprintf (mFile,"uavRecorder.cmd.time(%d,1) = % -6.14f;\n", cmdcount, velStamp);
	fprintf (mFile,"uavRecorder.cmd.velocity(%d,:) = [% -6.14f,% -6.14f,% -6.14f,% -6.14f];\n\n", cmdcount,
		cmdTwist.at<double>(0, 0), cmdTwist.at<double>(0, 1), cmdTwist.at<double>(0, 2), cmdTwist.at<double>(0, 3));
}

void hastUAV::inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata)
{//ROS_INFO("uav void inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata)");
	++navDataCounter;
	// ROS_WARN("ckfRecorder: inertialUpdate");
	flightState = navdata->state;
	/*--------- KF - Constant Tilt Model------------- */
	navHeadSeq = navdata->header.seq;
	navStamp = navdata->header.stamp;
	navTSpublished = navStamp.toSec();

	/*--------- Has ? ------------- */
	if (navTSpublished != navTS)
	{	/*----- Inertial Update */
		// findCompassDrift(); // before liftoff, calculate the yaw drift rate
		
		// fprintf (mFile, "%% /*----- Inertial Update */;\n");
		/*----- Update Clock */
		navdt = navTSpublished - navTS;
		if (navdt>1000)// suppress huge velocity estimates if motion starts from zero
			{navdt=0.006;} // 0.006 is the mean dt of an early experiment
			ckf.Qdk = ckf.Fk * ckf.Qdk * ckf.Fk.t() + ckf.Qw*navdt;

		navTS = navTSpublished;

		/* ---- Read in message values ---- */
			/*----- Yaw values */
			compassYawLast = compassYaw;
			// compassYaw = navdata->rotZ;
			yaw_drift += navdt*yaw_drift_rate;
			compassYaw = navdata->rotZ;
			// 350?
			if ((compassYaw - compassYawLast) > 350) {compassYaw -= 360;}
			if ((compassYaw - compassYawLast) < -350) {compassYaw += 360;}

			EstYaw_gl = compassYaw + EstYawBias  - yaw_drift;  // use only when UAV is suspended

			cosyaw = cos(Pi * EstYaw_gl / 180);
			sinyaw = sin(Pi * EstYaw_gl / 180);

			Rgl2lo = (cv::Mat_<double>(3, 3) <<
						   cosyaw, sinyaw, 0,
						   -sinyaw, cosyaw, 0,
						   0, 0, 1);

			Rgl2lo4x4 = (cv::Mat_<double>(4, 4) <<
							   cosyaw, sinyaw, 0, 0,
							  -sinyaw, cosyaw, 0, 0,
							   0, 0, 1, 0,
							   0, 0, 0, 1);

			// Altitude update from sonar
			echoAltLast = echoAlt;
			echoAlt = 0.001 * double(navdata->altd);
			deltaAlt = echoAlt - echoAltLast;

			// Velocity update
			MeasuredVel_lo = (cv::Mat_<double>(3, 1) <<
								0.001 * (navdata->vx),
								0.001 * (navdata->vy),
								deltaAlt/0.05); // should this be deltaAlt/navdt?

			MeasuredVel_gl = Rgl2lo.t() * MeasuredVel_lo;
			EstPosition_gl += MeasuredVel_gl * navdt;

			MeasuredAcc_lo = (cv::Mat_<double>(3, 1) <<
								(navdata->ax),
								(navdata->ay),
								(navdata->az)); 
			MeasuredAcc_gl= Rgl2lo.t() * MeasuredAcc_lo;

			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

			posePublisher(navTS);
			if(!ckf_sim)
			{
				tfPublisher(navStamp);
				// fprintf (mFile, "uavRecorder.tf_pub.time(%d,:) = % -6.14f;\n", navDataCounter, navTS);
			}
			
		/*----- Write Data */

			// fprintf (mFile, "uavRecorder.est.time(%d,1) = % -6.14f;\n", ++estCounter, navTS);
			// fprintf (mFile, "uavRecorder.est.Yaw(%d,1) = % -6.14f;\n", estCounter, EstYaw_gl);
			// fprintf (mFile, "uavRecorder.est.YawBias(%d,1) = % -6.14f;\n", estCounter, EstYawBias);
			// fprintf (mFile, "uavRecorder.est.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", estCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));

			fprintf (mFile, "uavRecorder.navdata.time(%d,:) = % -6.14f;\n", navDataCounter, navTS);
			fprintf (mFile, "uavRecorder.navdata.V(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", navDataCounter, navdata->vx, navdata->vy, navdata->vz);
			fprintf (mFile, "uavRecorder.navdata.Alt(%d,:) = % -6.14f;\n", navDataCounter, echoAlt);
			fprintf (mFile, "uavRecorder.navdata.RPY(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", navDataCounter, navdata->rotY, navdata->rotX, navdata->rotZ);
			fprintf (mFile, "uavRecorder.navdata.navdt(%d,:) = % -6.14f;\n", navDataCounter, navdt);
			fprintf (mFile, "uavRecorder.navdata.HeadSeq(%d,:) = %d;\n", navDataCounter, navHeadSeq);
			fprintf (mFile, "uavRecorder.navdata.Battery(%d,:) = % -6.14f;\n", navDataCounter, navdata->batteryPercent);
			fprintf (mFile, "uavRecorder.navdata.DeltaAlt(%d,:) = % -6.14f;\n", navDataCounter, deltaAlt);
			fprintf (mFile, "uavRecorder.navdata.yaw_drift(%d,:) = % -6.14f;\n", navDataCounter, yaw_drift);
			fprintf (mFile, "uavRecorder.navdata.EstYaw_gl(%d,:) = % -6.14f;\n", navDataCounter, EstYaw_gl);
			fprintf (mFile, "uavRecorder.navdata.CompassYaw(%d,:) = % -6.14f;\n", navDataCounter, compassYaw);
			fprintf (mFile, "uavRecorder.navdata.MeasuredVel_gl(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", navDataCounter, MeasuredVel_gl.at<double>(0,0), MeasuredVel_gl.at<double>(1,0), MeasuredVel_gl.at<double>(2,0));
			fprintf (mFile, "uavRecorder.navdata.MeasuredVel_lo(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", navDataCounter, MeasuredVel_lo.at<double>(0,0), MeasuredVel_lo.at<double>(1,0), MeasuredVel_lo.at<double>(2,0));
			fprintf (mFile, "uavRecorder.navdata.ckf.Qdk(%d,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", navDataCounter, ckf.Qdk.at<double>(0,0), ckf.Qdk.at<double>(1,1), ckf.Qdk.at<double>(2,2), ckf.Qdk.at<double>(3,3), ckf.Qdk.at<double>(4,4));
	}
}

bool hastUAV::uavSwitchTags(hast::flipflop::Request &req, hast::flipflop::Response &res)
{ // softare control, switch between stereo for uav guidance and stereo for ugv odometry
	if(req.flip)
	{// True -> Guidance
		ROS_INFO("ckfRecorder: UAV is measuring tags");
		positionFromTags = true;
		res.flop = true;
	}
	if(!req.flip)
	{// False -> Odometry
		ROS_INFO("ckfRecorder: UAV is guiding from tags");
		positionFromTags = false;
		res.flop = false;
	}
	return true; //this needs to be true, otherwise the service doesn't send the response.
}

void hastUAV::tfPublisher(ros::Time pubstamp)
{//ROS_INFO("uav void tfPublisher(ros::Time pubstamp)");
	// ROS_INFO("uavRecorder: -------Pose_msg --------------------------");
	// ROS_INFO("uavRecorder: EstPosition_gl  = [%6.4f %6.4f %6.4f]  ", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
	// ROS_INFO("uavRecorder: [00 01 02] = [%6.4f %6.4f %6.4f]  ",  cosyaw, sinyaw, 0.0);
	// ROS_INFO("uavRecorder: [10 11 12] = [%6.4f %6.4f %6.4f]  ", -sinyaw, cosyaw, 0.0);
	// ROS_INFO("uavRecorder: [20 21 22] = [%6.4f %6.4f %6.4f]  ",  0.0,0.0,1.0);

	TF_gl.setOrigin( tf::Vector3(EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0)) );
	R_TF_gl.setValue(cosyaw, -sinyaw, 0,
					 sinyaw, cosyaw, 0,
					  0,0,1);
	R_TF_gl.getRotation(Q_TF_gl);
	TF_gl.setRotation(Q_TF_gl);
	TF_gl_pub.sendTransform(tf::StampedTransform(TF_gl, pubstamp, s_TF_gl_parent, s_TF_gl_child));
	poseHeaderPublisher(pubstamp);
	// ROS_WARN("oneCKF::tfPublisher");
	// ROS_INFO("  parent: %s, ", s_TF_gl_parent.c_str());
	// ROS_INFO("  child:  %s ", s_TF_gl_child.c_str());
	// ROS_INFO("  TF_gl.setOrigin : [%6.4f %6.4f %6.4f]", EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0));
}

void hastUAV::poseHeaderPublisher(ros::Time pubstamp)
{//ROS_INFO("UAV void posePublisher(double pubtime)");
// publish estimated pose for other nodes to use

	hast::posewithheader pose_msg;
	pose_msg.position.x = EstPosition_gl.at<double>(0, 0);
	pose_msg.position.y = EstPosition_gl.at<double>(1, 0);
	pose_msg.position.z = EstPosition_gl.at<double>(2, 0);
	pose_msg.orientation = tf::createQuaternionMsgFromYaw(Pi * EstYaw_gl / 180);

	/*----- Time stamp of Estimate */
	pose_msg.header.seq = ++pose_msg_seq;
	pose_msg.header.stamp = pubstamp;
	pose_msg.header.frame_id = "/map";

	pose_pub.publish(pose_msg);

}

void hastUAV::posePublisher(double pubtime)
{//ROS_INFO("UAV void posePublisher(double pubtime)");
// publish estimated pose for other nodes to use
	/*----- Position Estimate */
	state_msg.P.x = EstPosition_gl.at<double>(0, 0);
	state_msg.P.y = EstPosition_gl.at<double>(1, 0);
	state_msg.P.z = EstPosition_gl.at<double>(2, 0);

	/*----- Velocity Estimate */
	state_msg.V.x = MeasuredVel_gl.at<double>(0, 0);
	state_msg.V.y = MeasuredVel_gl.at<double>(1, 0);
	state_msg.V.z = MeasuredVel_gl.at<double>(2, 0);

	/*----- Acceleration Measured */
	state_msg.A.x = MeasuredAcc_gl.at<double>(0, 0);
	state_msg.A.y = MeasuredAcc_gl.at<double>(1, 0);
	state_msg.A.z = MeasuredAcc_gl.at<double>(2, 0);

	/*----- Pose Estimate */
	state_msg.R.row0.x = Rgl2lo.at<double>(0,0);
	state_msg.R.row0.y = Rgl2lo.at<double>(0,1);
	state_msg.R.row0.z = 0;
	state_msg.R.row1.x = Rgl2lo.at<double>(1,0);
	state_msg.R.row1.y = Rgl2lo.at<double>(1,1);
	state_msg.R.row1.z = 0;
	state_msg.R.row2.x = 0;
	state_msg.R.row2.y = 0;
	state_msg.R.row2.z = 1;

	/*----- Yaw Estimate */
	state_msg.yaw = EstYaw_gl;

	/*----- Time stamp of Estimate */
	state_msg.id = ++stateMsg_id;
	state_msg.stamp = pubtime;
	state_pub.publish(state_msg);

	fprintf (mFile, "uavRecorder.est.time(%d,1) = % -6.14f;\n", ++estCounter, pubtime);
	fprintf (mFile, "uavRecorder.est.Yaw(%d,1) = % -6.14f;\n", estCounter, EstYaw_gl);
	fprintf (mFile, "uavRecorder.est.YawBias(%d,1) = % -6.14f;\n", estCounter, EstYawBias);
	fprintf (mFile, "uavRecorder.est.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n\n", estCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
}

