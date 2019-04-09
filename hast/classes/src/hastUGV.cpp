#include "hastUGV.hpp"

hastUGV::hastUGV() 
{
	// ROS comms
		stateMsg_id = 0;

	// Possibly useful constants
		Pi = atan(1) * 4; // 3.14159...
		L2Norm = 4; // Frobenius norm for CV norm() function

	// Initial estimated states of ugv
		EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		EstYaw_gl = 0;
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

		// Hgl2lo transforms a point in the global frame into the tb frame
			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

		velStamp = 0;
		velStamplast = 0;
		velCounter = 0;
		estCounter = 0;

		correction_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		yawCorrection = 0;

		//complementary KF class init // make 5x5 for oneCKF
		ckf.Qw = 0.00001*(cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,0); // the last term is zero because the ugv doesnt contribute to the yaw bias
		ckf.Fk =  (cv::Mat_<double>(5, 5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
		ckf.Qdk = (cv::Mat_<double>(5, 5) << 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0); // the last term is zero because the ugv doesnt contribute to the yaw bias

	/*-----  Initialize Wheel Odometry */
		wheelTime = 0;
		lastWheelTime = 0;
		wheelcount = 0;
		wheelTimeStamp = ros::Time::now();

		wheelyaw_q = 0;

		// using odom twist message:
			wheel_yaw_rate = 0;
			wheel_twist_linear = (cv::Mat_<double>(3, 1) << 0, 0, 0);
			wheel_twist_angular = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		
		// reduce datawrite from vel commands
		vel_dt_aggregator = 0;
}

void hastUGV::openmFile()
{
	ros::Duration(0.1).sleep();
	mFile = std::fopen (s_filename.c_str(), "w");
	ROS_INFO("ckfRecorder::ugvClass: %s", s_filename.c_str());
	ros::Duration(0.2).sleep();

	fprintf (mFile, "ugvRecorder.ckfinit.Qw(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(0, 0), ckf.Qw.at<double>(0, 1), ckf.Qw.at<double>(0, 2), ckf.Qw.at<double>(0, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qw(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(1, 0), ckf.Qw.at<double>(1, 1), ckf.Qw.at<double>(1, 2), ckf.Qw.at<double>(1, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qw(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(2, 0), ckf.Qw.at<double>(2, 1), ckf.Qw.at<double>(2, 2), ckf.Qw.at<double>(2, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qw(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qw.at<double>(3, 0), ckf.Qw.at<double>(3, 1), ckf.Qw.at<double>(3, 2), ckf.Qw.at<double>(3, 3));

	fprintf (mFile, "ugvRecorder.ckfinit.Qdk(1,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(0, 0), ckf.Qdk.at<double>(0, 1), ckf.Qdk.at<double>(0, 2), ckf.Qdk.at<double>(0, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qdk(2,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(1, 0), ckf.Qdk.at<double>(1, 1), ckf.Qdk.at<double>(1, 2), ckf.Qdk.at<double>(1, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qdk(3,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", ckf.Qdk.at<double>(2, 0), ckf.Qdk.at<double>(2, 1), ckf.Qdk.at<double>(2, 2), ckf.Qdk.at<double>(2, 3));
	fprintf (mFile, "ugvRecorder.ckfinit.Qdk(4,:) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", ckf.Qdk.at<double>(3, 0), ckf.Qdk.at<double>(3, 1), ckf.Qdk.at<double>(3, 2), ckf.Qdk.at<double>(3, 3));
	ros::Duration(0.1).sleep();
}

void hastUGV::posePublisher(double timestamp)
{
	state_msg.stamp = timestamp;
	state_msg.id = ++stateMsg_id;
	state_msg.P.x = EstPosition_gl.at<double>(0, 0);
	state_msg.P.y = EstPosition_gl.at<double>(0, 1);
	state_msg.P.z = EstPosition_gl.at<double>(0, 2);
	state_msg.yaw = EstYaw_gl;
	state_pub.publish(state_msg);

	if(!ckf_sim)
	{
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(state_msg.P.x, state_msg.P.y, state_msg.P.z) );
		tf::Quaternion q;
		q.setRPY(0, 0, state_msg.yaw*Pi/180);
		transform.setRotation(q);
		// send pose as odometry measurement
		// br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp), "/kobuki/odom", "/kobuki/base_footprint"));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), s_wheel_odom_topic.c_str(), s_base_footprint_topic.c_str()));

	}

	fprintf (mFile, "ugvRecorder.est.time(%d,1)  = % -6.14f;\n", ++estCounter, timestamp);
	fprintf (mFile, "ugvRecorder.est.Position_gl(%u,:) = [% -6.14f % -6.14f % -6.14f];\n", estCounter, EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
	fprintf (mFile, "ugvRecorder.est.Yaw(%d,1)  = % -6.14f;\n\n", estCounter, EstYaw_gl); //degrees

	// ROS_INFO("ugvRecorder: sendTransform");	
}

cv::Mat hastUGV::wrapH(cv::Mat R_in, cv::Mat t_in)
{// H = [R -R*t(1:3,1); 0 0 0 1];
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

cv::Mat hastUGV::invertH(cv::Mat H)
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

void hastUGV::WheelOdometry(const nav_msgs::Odometry::ConstPtr& osub)
{
	
	// ROS_WARN("WheelOdometry Called");
	wheelTimeStamp = osub->header.stamp;
	wheelTime = wheelTimeStamp.toSec();

	if (lastWheelTime != wheelTime)
	{//new data
		// ROS_INFO("ugvRecorder: wheel update");
			wheel_dt = wheelTime - lastWheelTime;
			if (wheel_dt>0.5)// suppress huge velocity estimates if motion starts from zero
				{wheel_dt=0.05;} // 0.1 is the mean dt of experiments on 20170922

			lastWheelTime = wheelTime;

		// update new yaw measurement
			wheel_twist_linear = (cv::Mat_<double>(3, 1) << osub->twist.twist.linear.x, osub->twist.twist.linear.y, osub->twist.twist.linear.z);
			wheel_twist_angular = (cv::Mat_<double>(3, 1) << osub->twist.twist.angular.x, osub->twist.twist.angular.y, osub->twist.twist.angular.z);
			wheel_yaw_rate = osub->twist.twist.angular.z; // negative for some reason

			double x = osub -> pose.pose.orientation.x;
			double y = osub -> pose.pose.orientation.y;
			double z = osub -> pose.pose.orientation.z;
			double w = osub -> pose.pose.orientation.w;
			
			wheeldeltayaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)) - wheelyaw_q;
			wheelyaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)); // radians
			// wheeldeltayaw_q = 2*acos(osub -> pose.pose.orientation.w) - wheelyaw_q;
			// wheelyaw_q = 2*acos(osub -> pose.pose.orientation.w); // radians

		//update estimate of ugv yaw
			// EstYaw_gl += wheel_yaw_rate*wheel_dt*180/Pi;  // rad/sec --> deg/sec?
			EstYaw_gl += wheeldeltayaw_q*180/Pi;  // rad/sec --> deg/sec?
			cosyaw = cos(Pi * EstYaw_gl / 180);
			sinyaw = sin(Pi * EstYaw_gl / 180);
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
							 cosyaw, sinyaw, 0,
							-sinyaw, cosyaw, 0,
										0,0,1);

		// Velocity update
			MeasuredVel_lo = (cv::Mat_<double>(3, 1) << osub->twist.twist.linear.x, 0, 0); // forward/backward velocity only
			MeasuredVel_gl = Rgl2lo.t() * MeasuredVel_lo;
			EstPosition_gl += MeasuredVel_gl * wheel_dt;


		// grow Qdk only when ugv is moving
			ckf.Qdk = ckf.Fk * ckf.Qdk * ckf.Fk.t() + ckf.Qw*wheel_dt;

			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

			posePublisher(wheelTime);

		fprintf (mFile, "ugvRecorder.wheel.time(%d,1) = % -6.14f;\n", ++wheelcount, wheelTime);
		fprintf (mFile, "ugvRecorder.wheel.dt(%d,1)   = % -6.14f;\n",   wheelcount, wheel_dt);

		// odom only records
		fprintf (mFile, "ugvRecorder.wheel.P_odom(%d,:)  = [% -6.14f, % -6.14f, % -6.14f];\n", wheelcount,osub -> pose.pose.position.x,osub -> pose.pose.position.y,osub -> pose.pose.position.z);
		fprintf (mFile, "ugvRecorder.wheel.quat(%d,:)    = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", wheelcount, osub -> pose.pose.orientation.x,osub -> pose.pose.orientation.y,osub -> pose.pose.orientation.z,osub -> pose.pose.orientation.w);
		fprintf (mFile, "ugvRecorder.wheel.yaw_q(%d,1)   =  % -6.14f;\n", wheelcount, wheelyaw_q); //radians?
		fprintf (mFile, "ugvRecorder.wheel.yawrate(%d,1) =  % -6.14f;\n", wheelcount, wheel_yaw_rate); //radians?
		
		// ckf records
		fprintf (mFile, "ugvRecorder.wheel.twist.linear_xyz(%d,:)  = [% -6.14f, % -6.14f, % -6.14f];\n", wheelcount, osub->twist.twist.linear.x,osub->twist.twist.linear.y,osub->twist.twist.linear.z);
		fprintf (mFile, "ugvRecorder.wheel.twist.angular_xyz(%d,:) = [% -6.14f, % -6.14f, % -6.14f];\n", wheelcount, osub->twist.twist.angular.x,osub->twist.twist.angular.y,osub->twist.twist.angular.z);
		fprintf (mFile, "ugvRecorder.wheel.EstYaw_gl(%d,:) = % -6.14f;\n\n", wheelcount, EstYaw_gl);

		fprintf (mFile, "ugvRecorder.wheel.Qdk(1,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", wheelcount, ckf.Qdk.at<double>(0, 0), ckf.Qdk.at<double>(0, 1), ckf.Qdk.at<double>(0, 2), ckf.Qdk.at<double>(0, 3));
		fprintf (mFile, "ugvRecorder.wheel.Qdk(2,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", wheelcount, ckf.Qdk.at<double>(1, 0), ckf.Qdk.at<double>(1, 1), ckf.Qdk.at<double>(1, 2), ckf.Qdk.at<double>(1, 3));
		fprintf (mFile, "ugvRecorder.wheel.Qdk(3,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n", wheelcount, ckf.Qdk.at<double>(2, 0), ckf.Qdk.at<double>(2, 1), ckf.Qdk.at<double>(2, 2), ckf.Qdk.at<double>(2, 3));
		fprintf (mFile, "ugvRecorder.wheel.Qdk(4,:,%d) = [% -6.14f, % -6.14f, % -6.14f, % -6.14f];\n\n", wheelcount, ckf.Qdk.at<double>(3, 0), ckf.Qdk.at<double>(3, 1), ckf.Qdk.at<double>(3, 2), ckf.Qdk.at<double>(3, 3));
	}
}



