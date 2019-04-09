#include "genheaders.hpp"

class ugvAutopilot
{
	private:
		double Pi;

		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;

		ros::Publisher ugvCmd_pub;
			geometry_msgs::Twist ugvCmd_msg;

		ros::Publisher DriveState_pub;
			hast::flag DriveState_msg;

		ros::Subscriber ugvState_sub, HastShutDown_sub;
			double StateTS, LastStateTS;

			/*----- Client Services */
		ros::ServiceServer ugvDrive_ser, ugvPilot_ser;
		std::string s_drivestate_topic, s_ugvCmd_topic, s_drive_service, s_pilot_service;


		/*---------  PID Variables ------------- */
			cv::Mat goalP_gl, estP_gl, errorP_gl;
			double yawFacing, yawError, yawEst, yawAtGoal;
			double yawKp, yawKi, yawKd, yawRamp;
			double posKp, posKi, posKd, posRamp;
			double yawSpeedLimit, posSpeedLimit;
			double time_cutoff;

			double epsPos, epsYaw; // epsilon is the close-enough threshold to stop guiding ugv and turn to goal yaw angle
			double errXY; // the x+y error in ugv location to check against eps
			
			int cmdCount, estCount, guideCount;


		/*---------  File Recorder variables ------------- */
		std::string s_filename, s_run, s_dotm, s_root, s_handle, s_date, s_user;
		std::FILE * pFile;

	public:
	ugvAutopilot()
	{
		Pi = atan(1) * 4; // 3.14159...
		/*---------  File Recorder Initilizer ------------- */
			s_handle = "ugvAutopilot_";
			if (n.getParam("/hast/user", s_user)) {} else {s_user = "turtlebot";}
			s_root = "/home/" + s_user + "/ros/data/";
			s_dotm = ".m";
			s_run = "000";
			if(n.getParam("/hast/run", s_run)){} else {s_run = "000";}
			if(n.getParam("/hast/date", s_date)){} else {s_date = "";}
			s_filename = s_root + s_date + "/" + s_run + "/" + s_handle + s_run + s_dotm;
			// mac ROS_INFO("experiment: %s", s_filename.c_str());
			pFile = std::fopen (s_filename.c_str(),"w");
			fprintf (pFile,"%% %s",s_filename.c_str());
			fprintf (pFile,"\n%%clc; \n%%clear all;\n%%close all;\n\n");


		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		HastShutDown_sub= n.subscribe("/hast/shutdown", 10, &ugvAutopilot::nodeShutDown, this);
		ugvState_sub 	= n.subscribe("/hast/ugv/state", 1, &ugvAutopilot::ReadEstimatedState, this);

		ros::param::get("~drivestate_topic", s_drivestate_topic);
		ros::param::get("~ugvCmd_topic", s_ugvCmd_topic);
		DriveState_pub 	= n.advertise<hast::flag>(s_drivestate_topic, 60);
		ugvCmd_pub		= n.advertise<geometry_msgs::Twist>(s_ugvCmd_topic, 10);

		/*-----  Servers and Clients */
		ros::param::get("~drive_service", s_drive_service);
			ugvDrive_ser	= n.advertiseService(s_drive_service, &ugvAutopilot::DriveRequest, this);
		ros::param::get("~pilot_service", s_pilot_service);
			ugvPilot_ser	= n.advertiseService(s_pilot_service, &ugvAutopilot::pilotUGV, this);
		
		fprintf (pFile,"%% s_drivestate_topic: 	%s \n",s_drivestate_topic.c_str());
		fprintf (pFile,"%% s_ugvCmd_topic: 		%s \n",s_ugvCmd_topic.c_str());
		fprintf (pFile,"%% s_drive_service: 	%s \n",s_drive_service.c_str());
		fprintf (pFile,"%% s_pilot_service: 	%s \n\n",s_pilot_service.c_str());
		
		estP_gl 	= (cv::Mat_<double>(3,1) << 0,0,0);
		goalP_gl 	= (cv::Mat_<double>(3,1) << 0,0,0);
		errorP_gl 	= (cv::Mat_<double>(3,1) << 0,0,0);
		
		yawEst		= 0;
		yawError	= 0;
		yawFacing 	= 0;
		yawAtGoal	= 0;

		// epsPos = 0.1 ; // meters

		if(ros::param::get("~posKp",posKp)){} else {posKp = 0.01;} // proportional gain on position control
		if(ros::param::get("~posKi",posKi)){} else {posKi = 0.00;} // 
		if(ros::param::get("~posKd",posKd)){} else {posKd = 0.00;} // 
		if(ros::param::get("~epsPos",epsPos)){} else {epsPos = 0.1;} // threshold for stopping motion
		if(ros::param::get("~posRamp",posRamp)){} else {posRamp = 0.05;} // ramp rate on velocity controller

		if(ros::param::get("~yawKp",yawKp)){} else {yawKp = 0.01;} // proportional gain on yaw control
		if(ros::param::get("~yawKi",yawKi)){} else {yawKp = 0.00;} // 
		if(ros::param::get("~yawKd",yawKd)){} else {yawKp = 0.00;} // 
		if(ros::param::get("~epsYaw",epsYaw)){} else {epsYaw = 0.1;} // threshold for stopping motion
		if(ros::param::get("~yawRamp",yawRamp)){} else {yawRamp = 0.05;} // ramp rate on velocity controller
		
		if(ros::param::get("~time_cutoff",time_cutoff)){} else {time_cutoff = 20.0;} // 
		if(ros::param::get("~posSpeedLimit",posSpeedLimit)){} else {posSpeedLimit = 0.25;} // 
		if(ros::param::get("~yawSpeedLimit",yawSpeedLimit)){} else {yawSpeedLimit = 0.25;} // 
		
		fprintf (pFile,"ugvAutopilot.init.posKp   = %6.14f;\n", posKp);
		fprintf (pFile,"ugvAutopilot.init.posKi   = %6.14f;\n", posKi);
		fprintf (pFile,"ugvAutopilot.init.posKd   = %6.14f;\n", posKd);
		fprintf (pFile,"ugvAutopilot.init.epsPos  = %6.14f;\n", epsPos);
		fprintf (pFile,"ugvAutopilot.init.posRamp = %6.14f;\n", posRamp);

		fprintf (pFile,"ugvAutopilot.init.yawKp   = %6.14f;\n", yawKp);
		fprintf (pFile,"ugvAutopilot.init.yawKi   = %6.14f;\n", yawKi);
		fprintf (pFile,"ugvAutopilot.init.yawKd   = %6.14f;\n", yawKd);
		fprintf (pFile,"ugvAutopilot.init.epsYaw  = %6.14f;\n", epsYaw);
		fprintf (pFile,"ugvAutopilot.init.yawRamp = %6.14f;\n", yawRamp);

		fprintf (pFile,"ugvAutopilot.init.time_cutoff     = %6.14f;\n", time_cutoff);
		fprintf (pFile,"ugvAutopilot.init.posSpeedLimit   = %6.14f;\n", posSpeedLimit);
		fprintf (pFile,"ugvAutopilot.init.yawSpeedLimit   = %6.14f;\n", yawSpeedLimit);


		cmdCount = 0;
		estCount = 0;
		guideCount = 0;

		ugvCmd_msg.linear.x = 0.0;
		ugvCmd_msg.linear.y = 0.0;
		ugvCmd_msg.linear.z = 0.0;
		ugvCmd_msg.angular.z = 0.0;
		ugvCmd_msg.angular.x = 0.0;
		ugvCmd_msg.angular.y = 0.0;

		DriveState_msg.flag = false; // Bool false = stationary, true = driving
		initpub();

		ROS_INFO("UGV ugvAutopilot Constructed");
	}

/*--------- Autonomous Drive functions ------------- */
	bool pilotUGV(hast::ugvautopilot::Request &req, hast::ugvautopilot::Response &res)
	{
		if (req.drive)
		{
 			ROS_INFO("/*--------- Guide UGV ------------- */");
			DriveState_msg.flag = true; // Bool false = stationary, true = driving
			DriveState_pub.publish(DriveState_msg);

			goalP_gl  = (cv::Mat_<double>(3,1) << req.P.x,req.P.y,req.P.z);
			yawAtGoal =  req.Yaw;

			if (req.turnonly)
			{
				TurnOnly(); // turn to desired orientation
			} else {
				Face(); // turn to face goal location
				Move(); // drive towards goal location
			}
			

			rampToZero();

			res.stopped = true;
			DriveState_msg.flag = false; // Bool false = stationary, true = driving
			DriveState_pub.publish(DriveState_msg);
 			return true;
		}
		else
		{
			return false;
		}
	}
	void Face()
	{
		ROS_INFO("ugvAutopilot:: Turn to face destination");
		double timestop, timestart, tnow;
		timestart = ros::Time::now().toSec();
		timestop = 20.0; // seconds to drive before cutting off motion

		//turn to face goal location
		errorP_gl 	= goalP_gl - estP_gl;
		yawFacing = atan2(errorP_gl.at<double>(1,0), errorP_gl.at<double>(0,0)) * 180 / Pi; //facing angle in degrees in global frame
		yawError  = yawEst - yawFacing;
		errXY = abs(errorP_gl.at<double>(0,0)) + abs(errorP_gl.at<double>(1,0));
		// ROS_INFO ("ugvAutopilot::yawFacing : %6.10f;", yawFacing);
		// ROS_INFO ("ugvAutopilot::25*epsYaw : %6.10f;", 25*epsYaw);
		// ROS_INFO ("ugvAutopilot::yawError : %6.10f;", yawError);
		while ((25*epsYaw < abs(yawError)) && (ros::Time::now().toSec() - timestart < timestop)) // eps = 0,1 ---> turn until ugv is within 2.5 degrees of the correct facing direction
		{
			//yawError is both the direction and magnitude that the UGV needs to turn
			errorP_gl 	= goalP_gl - estP_gl;
			errXY = abs(errorP_gl.at<double>(0,0)) + abs(errorP_gl.at<double>(1,0));
			yawFacing = atan2(errorP_gl.at<double>(1,0), errorP_gl.at<double>(0,0)) * 180 / Pi; //facing angle in degrees in global frame
			yawError  = yawEst - yawFacing;

			// ROS_INFO ("ugvAutopilot:: [ros::Time::now().toSec() - timestart :: timestop] [%6.10f %6.10f];", (ros::Time::now().toSec() - timestart), timestop);
			// ROS_INFO ("ugvAutopilot:: [yawEst yawError] : [%6.10f %6.10f];", yawEst, yawError);
			// ROS_INFO ("ugvAutopilot::errorP_gl [X,Y,Z] : [%6.10f %6.10f %6.10f];", errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), errorP_gl.at<double>(2,0));
			fprintf (pFile,"ugvAutopilot.guide.time(%d,1)   = %6.14f;\n", ++guideCount, ros::Time::now().toSec());
				fprintf (pFile,"ugvAutopilot.guide.yawEst(%d,1)   = %6.14f;\n", guideCount, yawEst);
				fprintf (pFile,"ugvAutopilot.guide.yawFacing(%d,1)= %6.14f;\n", guideCount, yawFacing);
				fprintf (pFile,"ugvAutopilot.guide.yawError(%d,1) = %6.14f;\n", guideCount, yawError);
				fprintf (pFile,"ugvAutopilot.guide.yawAtGoal(%d,1) = %6.14f;\n", guideCount, yawAtGoal);
				fprintf (pFile,"ugvAutopilot.guide.estP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, estP_gl.at<double>(0,0), estP_gl.at<double>(1,0), estP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, goalP_gl.at<double>(0,0), goalP_gl.at<double>(1,0), goalP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errorP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), errorP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errXY(%d,1)   = %6.14f;\n", guideCount, errXY);

			rampToVelocity(-yawKp * yawError, 0);
			// ros::Duration(0.025).sleep();
		}
		ROS_INFO("ugvAutopilot::Stopping turn ");
		// rampToZero();
	}

	void Move()
	{
		// ROS_INFO("ugvAutopilot:: move to destination");
		// ROS_INFO("ugvAutopilot:: global goal Position : yawFacing [%6.10f %6.10f : %6.10f];", goalP_gl.at<double>(0,0), goalP_gl.at<double>(1,0), yawFacing);
		// ROS_INFO("ugvAutopilot:: global estimated Position : yawFacing [%6.10f %6.10f : %6.10f];", estP_gl.at<double>(0,0), estP_gl.at<double>(1,0), yawEst);
		// ROS_INFO("ugvAutopilot:: global error Position : yawError [%6.10f %6.10f : %6.10f];", errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), yawError);

		double timestop, timestart, tnow;

		//turn to face goal location
		errorP_gl 	= goalP_gl - estP_gl;
		yawFacing = atan2(errorP_gl.at<double>(1,0), errorP_gl.at<double>(0,0)) * 180 / Pi; //facing angle in degrees in global frame
		yawError  = yawEst - yawFacing;
		errXY =pow( errorP_gl.at<double>(0,0)*errorP_gl.at<double>(0,0) + errorP_gl.at<double>(1,0)*errorP_gl.at<double>(1,0), 0.5);

		//drive towards location
		timestart = ros::Time::now().toSec();
		timestop = time_cutoff; // seconds to drive before cutting off motion

		// ROS_INFO("ugvAutopilot:: [ros::Time::now().toSec() - timestart :: timestop] [%6.10f %6.10f];", (ros::Time::now().toSec() - timestart), timestop);
		// ROS_INFO("ugvAutopilot::estP_gl [x y] : [%6.10f %6.10f];", estP_gl.at<double>(0,0), estP_gl.at<double>(1,0));
		// ROS_INFO("ugvAutopilot::[errXY epsPos]: [%6.10f %6.10f];", errXY, epsPos);
		
		while ((epsPos < errXY) && (ros::Time::now().toSec() - timestart < timestop)) // eps = 0,1 ---> turn until ugv is within 2.5 degrees of the correct facing direction
		{
			errorP_gl = goalP_gl - estP_gl;
			errXY =pow( errorP_gl.at<double>(0,0)*errorP_gl.at<double>(0,0) + errorP_gl.at<double>(1,0)*errorP_gl.at<double>(1,0), 0.5);
			yawFacing = atan2(errorP_gl.at<double>(1,0), errorP_gl.at<double>(0,0)) * 180 / Pi; //facing angle in degrees in global frame
			yawError  = yawEst - yawFacing;

			//yawError is both the direction and magnitude that the UGV needs to turn
			ROS_INFO ("ugvAutopilot:: [ros::Time::now().toSec() - timestart :: timestop] [%6.10f %6.10f];", (ros::Time::now().toSec() - timestart), timestop);
			ROS_INFO ("ugvAutopilot::estP_gl [x y] : [%6.10f %6.10f];", estP_gl.at<double>(0,0), estP_gl.at<double>(1,0));
			ROS_INFO ("ugvAutopilot::[errXY epsPos]: [%6.10f %6.10f];", errXY, epsPos);


			fprintf (pFile,"ugvAutopilot.guide.time(%d,1)   = %6.14f;\n", ++guideCount, ros::Time::now().toSec());
				fprintf (pFile,"ugvAutopilot.guide.yawEst(%d,1)   = %6.14f;\n", guideCount, yawEst);
				fprintf (pFile,"ugvAutopilot.guide.yawFacing(%d,1)= %6.14f;\n", guideCount, yawFacing);
				fprintf (pFile,"ugvAutopilot.guide.yawError(%d,1) = %6.14f;\n", guideCount, yawError);
				fprintf (pFile,"ugvAutopilot.guide.yawAtGoal(%d,1) = %6.14f;\n", guideCount, yawAtGoal);
				fprintf (pFile,"ugvAutopilot.guide.estP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, estP_gl.at<double>(0,0), estP_gl.at<double>(1,0), estP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, goalP_gl.at<double>(0,0), goalP_gl.at<double>(1,0), goalP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errorP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), errorP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errXY(%d,1)   = %6.14f;\n", guideCount, errXY);

			rampToVelocity(-yawKp * yawError, posKp * errXY);			
			// ros::Duration(0.025).sleep();
		}

		}

	void TurnOnly()
	{
		ROS_INFO("ugvAutopilot:: Turn to face desired angle at destination");
		double timestop, timestart, tnow;
		timestart = ros::Time::now().toSec();
		timestop = 20.0; // seconds to drive before cutting off motion

		//turn to face goal location
		yawError  = yawEst - yawAtGoal;
		while ((25*epsYaw < abs(yawError)) && (ros::Time::now().toSec() - timestart < timestop)) // eps = 0,1 ---> turn until ugv is within 2.5 degrees of the correct facing direction
		{
			yawError  = yawEst - yawAtGoal;

			// ROS_INFO ("ugvAutopilot:: [ros::Time::now().toSec() - timestart :: timestop] [%6.10f %6.10f];", (ros::Time::now().toSec() - timestart), timestop);
			// ROS_INFO ("ugvAutopilot:: [yawEst yawError] : [%6.10f %6.10f];", yawEst, yawError);
			// ROS_INFO ("ugvAutopilot::errorP_gl [X,Y,Z] : [%6.10f %6.10f %6.10f];", errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), errorP_gl.at<double>(2,0));
			fprintf (pFile,"ugvAutopilot.guide.time(%d,1)   = %6.14f;\n", ++guideCount, ros::Time::now().toSec());
				fprintf (pFile,"ugvAutopilot.guide.yawEst(%d,1)   = %6.14f;\n", guideCount, yawEst);
				fprintf (pFile,"ugvAutopilot.guide.yawFacing(%d,1)= %6.14f;\n", guideCount, yawFacing);
				fprintf (pFile,"ugvAutopilot.guide.yawError(%d,1) = %6.14f;\n", guideCount, yawError);
				fprintf (pFile,"ugvAutopilot.guide.yawAtGoal(%d,1) = %6.14f;\n", guideCount, yawAtGoal);
				fprintf (pFile,"ugvAutopilot.guide.estP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, estP_gl.at<double>(0,0), estP_gl.at<double>(1,0), estP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, goalP_gl.at<double>(0,0), goalP_gl.at<double>(1,0), goalP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errorP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", guideCount, errorP_gl.at<double>(0,0), errorP_gl.at<double>(1,0), errorP_gl.at<double>(2,0));
				fprintf (pFile,"ugvAutopilot.guide.errXY(%d,1)   = %6.14f;\n", guideCount, errXY);

			rampToVelocity(-yawKp * yawError, 0);
			// ros::Duration(0.025).sleep();
		}
		ROS_INFO("ugvAutopilot::Stopping turn ");
		// rampToZero();
	}

/*--------- Drive functions ------------- */
	bool DriveRequest(hast::ugvdrive::Request &req, hast::ugvdrive::Response &res)
	{
		ROS_INFO("/*--------- Drive Turtlebot ?------------- */");
		if (req.drive)
		{
 			ROS_INFO("/*--------- Drive Turtlebot !------------- */");
			DriveState_msg.flag = true; // Bool false = stationary, true = driving
			DriveState_pub.publish(DriveState_msg);
			// ROS_INFO("Drive mode = %s", stereostate ? "Guidance" : "Odometry" );
			ROS_INFO("Drive(req.vel, req.yaw, req.time);");
			ROS_INFO("Drive(%6.4f, %6.4f, %6.4f);",req.vel, req.yaw, req.time);
 			Drive(req.vel, req.yaw, req.time);
 			res.stopped = true;
			DriveState_msg.flag = false; // Bool false = stationary, true = driving
			DriveState_pub.publish(DriveState_msg);
 			return true;
		}
		else
		{
			return false;
		}
	}

	void Drive(double Xdot, double Phidot, double dwellSeconds)
	{
		ROS_INFO("Driving");
		double drivestart = ros::Time::now().toSec();
		double driveclock = ros::Time::now().toSec();
		ugvCmd_msg.linear.x = Xdot;
		ugvCmd_msg.angular.z = Phidot;
		publishCmd();

		while ((driveclock - drivestart )< dwellSeconds)
		{
			driveclock = ros::Time::now().toSec();
			// ugvCmd_pub.publish(ugvCmd_msg);
			DriveState_pub.publish(DriveState_msg);
			publishCmd();
		}

		double drivestop = ros::Time::now().toSec();
		ROS_INFO("Total drive time: %6.4f [s]", (drivestop - drivestart));
		ROS_INFO("Drive Speed : %6.4f [m/s]", Xdot);
		ROS_INFO("Estimated Distance : %6.4f [m], %6.4f [in]", Xdot*(drivestop - drivestart), 39.3701*Xdot*(drivestop - drivestart));
		ROS_INFO("Stopping");
		ugvCmd_msg.linear.x = 0.0;
		ugvCmd_msg.angular.z = 0.0;
		for(int b = 0; b < 3; ++b)
		{
			// ugvCmd_pub.publish(ugvCmd_msg);
			publishCmd();
		}

	}

/*--------- Bookkeeping functions ------------- */
	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{if(ShutDown->flag){ros::shutdown();}}

	void rampToVelocity(double phiGain, double xGain)
	{
		ROS_INFO ("ugvAutopilot::[xGain msg.linear.x] : [%6.4f %6.4f];", xGain, ugvCmd_msg.linear.x);
		ROS_INFO ("ugvAutopilot::[phiGain msg.angular.z] : [%6.4f %6.4f];", phiGain, ugvCmd_msg.angular.z);
		double angularRateError = phiGain - ugvCmd_msg.angular.z;
		double linearRateError = xGain - ugvCmd_msg.linear.x;

		if (abs(angularRateError) < 0.025)
		{ // then small correction
			ugvCmd_msg.angular.z += angularRateError;
		} else { //correction is too large
			if (angularRateError > 0)
			{// then turn positive direction
				ugvCmd_msg.angular.z += 0.025;
			} else { //turn the other way
				ugvCmd_msg.angular.z -= 0.025;
			}
		} //end if (abs() < 0.05)

		if (abs(linearRateError) < 0.05)
		{ // then small correction
			ugvCmd_msg.linear.x += linearRateError;
		} else { //correction is too large
			if (linearRateError > 0)
			{// then turn positive direction
				ugvCmd_msg.linear.x += 0.05;
			} else { //turn the other way
				ugvCmd_msg.linear.x -= 0.05;
			}
		} //end if (abs() < 0.05)

		// ROS_INFO ("ugvAutopilot::ugvCmd_msg.linear.x : %6.10f;", ugvCmd_msg.linear.x);
		// ROS_INFO ("ugvAutopilot::ugvCmd_msg.angular.z : %6.10f;", ugvCmd_msg.angular.z);

		fprintf (pFile,"ugvAutopilot.guide.xGain(%d,1)   = %6.14f;\n", guideCount, xGain);
		fprintf (pFile,"ugvAutopilot.guide.phiGain(%d,1)   = %6.14f;\n", guideCount, phiGain);
		fprintf (pFile,"ugvAutopilot.guide.linearRateError(%d,1)   = %6.14f;\n", guideCount, xGain);
		fprintf (pFile,"ugvAutopilot.guide.angularRateError(%d,1)   = %6.14f;\n", guideCount, phiGain);
		
		publishCmd();
		DriveState_pub.publish(DriveState_msg);
	}

	void rampToZero()
	{
		while ((ugvCmd_msg.angular.z != 0) || (ugvCmd_msg.angular.z != 0)) // if either is not zero, keep ramping
		{
			double angularRateError = 0 - ugvCmd_msg.angular.z;
			if (abs(angularRateError) < 0.05)
			{ // then small correction
				ugvCmd_msg.angular.z += angularRateError;
			} else { //correction is too large
				if (angularRateError > 0)
				{// then turn positive direction
					ugvCmd_msg.angular.z += 0.05;
				} else { //turn the other way
					ugvCmd_msg.angular.z -= 0.05;
				}
			} //end if (abs() < 0.05)

			double linearRateError = 0 - ugvCmd_msg.linear.x;
			if (abs(linearRateError) < 0.05)
			{ // then small correction
				ugvCmd_msg.linear.x += linearRateError;
			} else { //correction is too large
				if (linearRateError > 0)
				{// then turn positive direction
					ugvCmd_msg.linear.x += 0.05;
				} else { //turn the other way
					ugvCmd_msg.linear.x -= 0.05;
				}
			} //end if (abs() < 0.05)

			
			publishCmd();
			DriveState_pub.publish(DriveState_msg);
		}
	}

	void initpub()
	{
		ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
		// ugvCmd_pub.publish(ugvCmd_msg);
		publishCmd();
		DriveState_pub.publish(DriveState_msg);
		ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
	}

	void publishCmd()
	{
		++cmdCount;
		if (ugvCmd_msg.linear.x >  posSpeedLimit) {ugvCmd_msg.linear.x =  posSpeedLimit;}
		if (ugvCmd_msg.linear.x < -posSpeedLimit) {ugvCmd_msg.linear.x = -posSpeedLimit;}

		if (ugvCmd_msg.angular.z >  yawSpeedLimit) {ugvCmd_msg.angular.z =  yawSpeedLimit;}
		if (ugvCmd_msg.angular.z < -yawSpeedLimit) {ugvCmd_msg.angular.z = -yawSpeedLimit;}

		fprintf (pFile,"ugvAutopilot.cmd.time(%d,1) = %6.14f;\n", cmdCount, ros::Time::now().toSec());
		fprintf (pFile,"ugvAutopilot.cmd.linearRate(%d,1) = %6.10f;\n", cmdCount, ugvCmd_msg.linear.x);
		fprintf (pFile,"ugvAutopilot.cmd.angularRate(%d,1) = %6.10f;\n\n", cmdCount, ugvCmd_msg.angular.z);
		ugvCmd_pub.publish(ugvCmd_msg);
		// ros::Duration(0.05).sleep();
		ros::spinOnce();
		ros::Duration(0.025).sleep();

	}

	void ReadEstimatedState(const hast::ugvstate::ConstPtr& ugvState_msg)
	{
		StateTS = ugvState_msg -> stamp;
		if ((StateTS!=LastStateTS))
		{ /*----- yes */
			++estCount;
			LastStateTS = StateTS;

			estP_gl = (cv::Mat_<double>(3,1) << ugvState_msg->P.x, ugvState_msg->P.y, ugvState_msg->P.z);
			yawEst = ugvState_msg->yaw;

			fprintf (pFile,"ugvAutopilot.est.time(%d,1) = %6.14f;\n", estCount, ros::Time::now().toSec());
			fprintf (pFile,"ugvAutopilot.est.linearRate(%d,1) = %6.10f;\n", estCount, ugvCmd_msg.linear.x);
			fprintf (pFile,"ugvAutopilot.est.angularRate(%d,1) = %6.10f;\n\n", estCount, ugvCmd_msg.angular.z);

		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ugvDrive");
	ugvAutopilot tC;
	ros::spin();
	return 0;
}
