#include "genheaders.hpp"	

#include <actionlib/client/simple_action_client.h>
#include <hast/ugvgoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr uavWaypoint_ptr (new pcl::PointCloud<pcl::PointXYZ>);

class uavClass
{
	private:
	public:

		// ROS comms
		// ros::NodeHandle uav_n;

		/* -------- Clock Times */
		ros::Time navStamp;
		uint navHeadSeq;
		double navTS, navTSpublished, navdt; //, KFdt;

		double Pi;
		int L2Norm;

		/* ugv position variables*/
		cv::Mat EstPosition_gl;
		double EstYaw_gl, cosyaw, sinyaw;
		cv::Mat Rgl2lo, Hlo2gl, Hgl2lo;

		
		ros::ServiceClient slamswitch_cli;
			hast::slamswitch slamswitch_call;
			hast::posewithheader pose_msg;
			uint pose_msg_seq;

		ros::Subscriber state_sub;
		ros::ServiceClient control_cli, picket_cli;
			hast::uavcontrolstate control_call;
			hast::uavpicket picket_call;
			cv::Mat picket_vector;
		ros::ServiceClient navState_cli;
			hast::uavnavstate navState_call;
			uint navStateUint;
		ros::Publisher Land_pub, TakeOff_pub, Reset_pub, imuRecal_pub, flatTrim_pub;
			std_msgs::Empty null;

		double DesiredYaw, flytime;
		cv::Mat Waypoint0;
		cv::Mat WaypointA, WaypointB, WaypointC, WaypointD, WaypointE, WaypointF, WaypointG, WaypointH, WaypointI;
		double WaypointYaw0, WaypointYawA, WaypointYawB, WaypointYawC, WaypointYawD, WaypointYawE, WaypointYawF, WaypointYawG, WaypointYawH, WaypointYawI;

		double liftoff_time;

		ros::Publisher uavWaypoint_pub;
			sensor_msgs::PointCloud2 uavWaypoint_msg;

	// uav class init
		uavClass() // void for construction of class
		{
			// Possibly useful constants
			pose_msg_seq = 0;
			Pi = atan(1) * 4; // 3.14159...
			L2Norm = 4; // Frobenius norm for CV norm() function

			navStateUint = 0;

			// Initial estimated states of ugv
			EstPosition_gl = (cv::Mat_<double>(3, 1) << 0, 0, 0);
			EstYaw_gl = 0;
			cosyaw = cos(Pi * EstYaw_gl / 180);
			sinyaw = sin(Pi * EstYaw_gl / 180);
			Rgl2lo = (cv::Mat_<double>(3, 3) <<
								cosyaw, sinyaw, 0,
								-sinyaw, cosyaw, 0,
								0, 0, 1);
			DesiredYaw = 0;
			flytime = 5;
			liftoff_time = 0;

			WaypointA = (cv::Mat_<double>(3,1) << 3.15, 0.80, 1.50); WaypointYawA = 0.00;
			WaypointB = (cv::Mat_<double>(3,1) << 2.25, 0.70, 1.40); WaypointYawB = 0.00;
			WaypointC = (cv::Mat_<double>(3,1) << 1.60, 0.60, 1.30); WaypointYawC = 0.00;
			WaypointD = (cv::Mat_<double>(3,1) << 1.60, 0.00, 1.30); WaypointYawD = 0.00;
			WaypointE = (cv::Mat_<double>(3,1) << 3.80, 0.00, 1.50); WaypointYawE = 0.00;
			WaypointF = (cv::Mat_<double>(3,1) << 3.15,-0.80, 1.50); WaypointYawF = 0.00;
			WaypointG = (cv::Mat_<double>(3,1) << 2.25,-0.70, 1.40); WaypointYawB = 0.00;
			WaypointH = (cv::Mat_<double>(3,1) << 1.60,-0.60, 1.30); WaypointYawG = 0.00;
			
			// WaypointH = (cv::Mat_<double>(3,1) << 1.50, 0.30, 1.00); WaypointYawH = 0.00;
			// WaypointI = (cv::Mat_<double>(3,1) << 1.50, 0.30, 1.00); WaypointYawI = 0.00;

 // 0.75*180/Pi; -0.75*180/Pi;

			// waypoint 0?
			Waypoint0 = (cv::Mat_<double>(3,1) << 2.50, 0.00, 1.50); WaypointYaw0 = 0.75*180/Pi;
		}

		void readState(const hast::uavstate::ConstPtr& uavState_msg)
		{
			EstPosition_gl = (cv::Mat_<double>(3,1) << uavState_msg->P.x, uavState_msg->P.y, uavState_msg->P.z);
			EstYaw_gl = uavState_msg->yaw;
		}

		void hoverFor(double span)
		{
			double tstart = ros::Time::now().toSec();
			double elapsed = 0;
			while(elapsed < span)
			{
				// mac ROS_INFO("experiment: Drone Hovering for (%6.4f)", (span-elapsed));
				hover();
				ros::Duration(0.25).sleep();
				elapsed = (ros::Time::now().toSec() - tstart);
			}
		}

		void hover()
		{
			ros::spinOnce();
			// // mac ROS_INFO("experiment: Drone Flight Service Called: Hover");
			control_call.request.flip = false;
			control_call.request.directtilt = false; // true: use tilt.x.y.z for control false = use PV control
			control_call.request.dP_g.x = 0;
			control_call.request.dP_g.y = 0;
			control_call.request.dP_g.z = 0;
			control_call.request.dYaw = 0;
			control_call.request.YawRate = 0;
			control_call.request.tilt.x = 0;
			control_call.request.tilt.y = 0;
			control_call.request.tilt.z = 0;
			control_cli.call(control_call);
		}

		void cmdtilt(double upRate, double roll, double pitch, double yawrate)
		{
			ros::spinOnce();
			// // mac ROS_INFO("experiment: Drone Flight Service Called: Hover");
			control_call.request.flip = true;
			control_call.request.directtilt = true; // true: use tilt.x.y.z for control false = use PV control
			control_call.request.dP_g.x = 0;
			control_call.request.dP_g.y = 0;
			control_call.request.dP_g.z = 0;
			control_call.request.dYaw = 0;
			control_call.request.YawRate = yawrate;
			control_call.request.tilt.x = pitch;
			control_call.request.tilt.y = roll;
			control_call.request.tilt.z = upRate;
			control_cli.call(control_call);
		}

		void increaseAltitude(double upRate)
		{
			ros::spinOnce();
			// // mac ROS_INFO("experiment: Drone Flight Service Called: Hover");
			control_call.request.flip = false;
			control_call.request.directtilt = true; // true: use tilt.x.y.z for control false = use PV control
			control_call.request.dP_g.x = 0;
			control_call.request.dP_g.y = 0;
			control_call.request.dP_g.z = 0;
			control_call.request.dYaw = 0;
			control_call.request.YawRate = 0;
			control_call.request.tilt.x = 0;
			control_call.request.tilt.y = 0;
			control_call.request.tilt.z = upRate;
			control_cli.call(control_call);
		}

		void callState()
		{
			navState_cli.call(navState_call);
			navStateUint = navState_call.response.state;
			ros::Duration(1).sleep(); // sleep for 'x' second(s).
		}

	/* UAV utility functions*/
		void liftoff()
		{
			/*--- Zero commanded tilts before liftoff */
			for(int b = 0; b < 2; ++b)
			{
				reCal();
				hover();
				ros::spinOnce();
			}
			/*--------- Drone Takeoff ------------- */
			callState();
			if(navStateUint==2)
			{
				TakeOff_pub.publish(null);
				ros::Duration(1.5).sleep(); // sleep for 'x' second(s).
				liftoff_time = ros::Time::now().toSec();
			}
			else{retryTakeoff();}

			callState();
			if (navStateUint==0){retryTakeoff();}

			for(int b = 0; b < 5; ++b)
			{
				ros::Duration(0.15).sleep();
				ros::spinOnce();
			}
			hoverFor(2);
		}

		void retryTakeoff()
		{
			do
			{
				Reset_pub.publish(null);
				// mac ROS_INFO("experiment: Drone Resetting..");
				ros::Duration(2).sleep(); // sleep for 'x' second(s).
				TakeOff_pub.publish(null);
		        callState();
		        callState();
		        callState();
		        callState();
		        callState();
		        liftoff_time = ros::Time::now().toSec();
			}while(navStateUint==0);
		}

		void setPicket(double setX, double setY, double setZ, double setYaw)
		{
			ROS_WARN("experiment: uav.setPicket(%6.2f %6.2f %6.2f %6.2f)", setX, setY, setZ, setYaw);
			picket_vector = (cv::Mat_<double>(3,1) << setX, setY, setZ);
			picket_call.request.flip = true; // true: use PV control instead of dumb hover
			picket_call.request.xyz.x = setX;
			picket_call.request.xyz.y = setY;
			picket_call.request.xyz.z = setZ;
			picket_call.request.yaw = setYaw;
			picket_cli.call(picket_call);

			publishUAVwaypoint(setX, setY, setZ);
		}

		void setDesiredPosition(double NewX, double NewY, double NewZ, double NewYaw)
		{
			// update desired positions in controller
			control_call.request.flip = true; // true: use PV control instead of dumb hover
			control_call.request.directtilt = false; // true: use tilt.x.y.z for control false = use PV control
			control_call.request.dP_g.x = NewX;
			control_call.request.dP_g.y = NewY;
			control_call.request.dP_g.z = NewZ;
			control_call.request.dYaw = NewYaw;
			control_cli.call(control_call);

			publishUAVwaypoint(NewX, NewY, NewZ);
		}

		void publishUAVwaypoint(double NewX, double NewY, double NewZ)
		{
			uavWaypoint_ptr->points.clear();

			pcl::PointXYZ pcl_XYZ; // pcl point for center of obstacle
				pcl_XYZ.x = NewX;
				pcl_XYZ.y = NewY;
				pcl_XYZ.z = NewZ;
			
			uavWaypoint_ptr->points.push_back(pcl_XYZ);
			pcl::toROSMsg(*uavWaypoint_ptr, uavWaypoint_msg);
			uavWaypoint_msg.header.frame_id = "/map";
			uavWaypoint_pub.publish(uavWaypoint_msg);
		}

		void reCal()
		{
			ROS_INFO("reCal(): Publishing Flat-Trim Command...");
				flatTrim_pub.publish(null);
				ros::Duration(0.25).sleep(); // sleep for 'x' second(s).

			ROS_INFO("reCal(): Publishing IMU reCal Command...");
				imuRecal_pub.publish(null);
				ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
			
			ROS_INFO("reCal(): ...done");
		}
};

class ugvClass
{
	private:
		// ROS comms
		// ros::NodeHandle ugv_n;

		// Constants which might be useful in the future
		double Pi;
		int L2Norm;

	public:
		/* ugv position variables*/
		cv::Mat EstPosition_gl;
		double EstYaw_gl, cosyaw, sinyaw;
		cv::Mat Rgl2lo, Hlo2gl, Hgl2lo;

		ros::Subscriber pose_sub;
		uint stateMsg_id, stateMsg_idLast; // pose msg counter
		double msgStamp, msgStampLast;
		uint posecount;

		/* ugv client/Services variables*/
		move_base_msgs::MoveBaseGoal goal; // for move_base goal setting
		
		hast::flipflop StereoOdom_call;

		ros::ServiceClient StereoOdom_cli, UpdateOdom_cli;
			bool stereostate;

		ros::Subscriber DriveState_sub;
			bool drivestate;


		ros::ServiceClient Jockey_cli, Pilot_cli;
			hast::ugvdrive Jockey_call;
			hast::ugvautopilot Pilot_call;

		// ros::Publisher Cmd_pub;
		// 	geometry_msgs::Twist Cmd_msg;

		double linearTime, linearRate;
		double angularTime, angularRate, arcTime;
		double TurnAngle, TurnRate, TurnLeft, TurnRight;
		double wait;

		cv::Mat goalP_gl;
		double yawAtGoal_gl;


		ugvClass() // void for construction of class
		{
			// ROS comms
			stateMsg_id = 0;
			stateMsg_idLast = 0;
			posecount = 0;
			wait = 0.2;

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

			// Hgl2lo transforms a point in the global frame into the tb frame
			Hgl2lo = wrapH(Rgl2lo, EstPosition_gl);
			Hlo2gl = invertH(Hgl2lo);

			// Default Parameters
			drivestate = false;
			linearTime = 1.5;
			linearRate = 0.25;
			TurnAngle = 0.7;
			TurnRate = 0.5;
			TurnLeft = 1;
			TurnRight = -1;
			stereostate = false;
			angularTime = 1.75;
			angularRate = 0.175;

			//intermediate step: simple pid controller for ugv

			//define the goal for the ugv motion using move_base
			
			goal.target_pose.header.frame_id = "/base_footprint";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = 1.0;
			goal.target_pose.pose.orientation.w = 1.0;
		}

		cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in)
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

		cv::Mat invertH(cv::Mat H)
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

		void updatePose()
		{
			// mac ROS_INFO("ugvClass: updatePose");
			do
			{
				stateMsg_idLast = stateMsg_id;
				ros::spinOnce();
			}
			while(stateMsg_idLast!=stateMsg_id);
			// mac ROS_INFO("experiment: UGV Pose updated.");
			// mac ROS_INFO("experiment: EstPosition_gl: [%6.4f %6.4f %6.4f]", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
		}


		void updateDriveState(const hast::flag::ConstPtr& DriveState)
		{
			drivestate = DriveState -> flag;
		}

		void readState(const hast::ugvstate::ConstPtr& ugvState_msg)
		{
			msgStamp = ugvState_msg-> stamp;

			if (msgStamp!=msgStampLast)
			{
				++posecount;
				msgStampLast = ugvState_msg-> stamp;
				Rgl2lo = (cv::Mat_<double>(3,3) <<
					ugvState_msg -> R.row0.x, ugvState_msg -> R.row0.y, ugvState_msg -> R.row0.z,
					ugvState_msg -> R.row1.x, ugvState_msg -> R.row1.y, ugvState_msg -> R.row1.z,
					ugvState_msg -> R.row2.x, ugvState_msg -> R.row2.y, ugvState_msg -> R.row2.z);
				EstPosition_gl = (cv::Mat_<double>(3, 1) <<
					ugvState_msg -> P.x,
					ugvState_msg -> P.y,
					ugvState_msg -> P.z);
				EstYaw_gl = ugvState_msg -> yaw;
				stateMsg_id = ugvState_msg -> id;
			}
		}



		void flip2Odom()//true = linear
		{
			// mac ROS_INFO("Setting Stereo to Odometry Mode ... ");
			StereoOdom_call.request.flip = false; // Set Stereo to odometry
			// StereoOdom_call.request.mode = mode;
			StereoOdom_cli.call(StereoOdom_call);
			ros::spinOnce();
			stereostate = StereoOdom_call.response.flop;
			// ROS_INFO("stereostate = %s", stereostate ? "Guidance" : "Odometry" );
		}

		void flop2Guide()
		{
			// mac ROS_INFO("Resetting Stereo to Guidance Mode ... ");
			StereoOdom_call.request.flip = true; // Set Stereo to guidance
			// StereoOdom_call.request.mode = true; //true = linear
			StereoOdom_cli.call(StereoOdom_call);
			ros::spinOnce();
			stereostate = StereoOdom_call.response.flop;
			// mac ROS_INFO("stereostate = %s", stereostate ? "Guidance" : "Odometry" );
		}

		void ArcOnce()
		{	
			ROS_INFO("experiment: ugv.ArcOnce [linearRate, angularRate, arcTime] = [%6.4f %6.4f %6.4f]", linearRate, angularRate, arcTime);
			// flip2Odom();
			// ros::Duration(wait).sleep();

			//Drive ugv forward
			// if (!stereostate) // False -> Odometry
			// {// Stereo Switched modes, drive ugv
				Jockey_call.request.drive = true;
				Jockey_call.request.vel  = linearRate; // m/s
				Jockey_call.request.yaw  = angularRate; // also turn
				Jockey_call.request.time = arcTime; // ~Seconds

				Jockey_cli.call(Jockey_call);
					// Wait for the tb to stop driving
					do{ros::spinOnce();}while(drivestate);
			// }
			// ros::Duration(wait).sleep();
			// flop2Guide();
			updatePose();
		}

		void ForwardOnce()
		{	
			ROS_INFO("experiment: ugv.ForwardOnce [linearRate, linearTime] = [%6.4f %6.4f]", linearRate, linearTime);
			// flip2Odom();
			// ros::Duration(wait).sleep();

			//Drive ugv forward
			// if (!stereostate) // False -> Odometry
			// {// Stereo Switched modes, drive ugv
				Jockey_call.request.drive = true;
				Jockey_call.request.vel  = linearRate; // m/s
				Jockey_call.request.yaw  = 0.0; // don't intentionally turn
				Jockey_call.request.time = linearTime; // ~Seconds

				Jockey_cli.call(Jockey_call);
					// Wait for the tb to stop driving
					do{ros::spinOnce();}while(drivestate);
			// }
			// ros::Duration(wait).sleep();
			// flop2Guide();
			updatePose();
		}

		void pilotTo(cv::Mat goalP, double yawAtGoal, bool turnonly)
		{	
			// bool drive
			// vector3 P	# desired global position of ugv
			// float64 Yaw	# desired global yaw of uav
			// bool turnonly # turn to yaw at goal without driving
			// ---
			// bool stopped

			ROS_INFO ("ugvPilot to [X,Y,Z, Yaw] : [%6.10f %6.10f %6.10f %6.10f];\n", goalP.at<double>(0,0), goalP.at<double>(1,0), goalP.at<double>(2,0), yawAtGoal);

				Pilot_call.request.drive 	= true;
				Pilot_call.request.P.x  	= goalP.at<double>(0,0);
				Pilot_call.request.P.y  	= goalP.at<double>(1,0);
				Pilot_call.request.P.z  	= 0.0; // ugv can't fly
				Pilot_call.request.Yaw  	= yawAtGoal; 
				Pilot_call.request.turnonly	= turnonly; //turn to yaw at goal without driving

				Pilot_cli.call(Pilot_call);
					// Wait for the tb to stop driving
					do{ros::spinOnce();}while(drivestate);
			// }
			// ros::Duration(wait).sleep();
			// flop2Guide();
			updatePose();
		}

		void TurnOnce()
		{
			ROS_INFO("experiment: ugv.TurnOnce [angularRate, angularTime] = [%6.4f %6.4f]", angularRate, angularTime);
			// flip2Odom(); // false means angular

			//Drive ugv forward
			// if (!stereostate) // False -> Odometry
			// {// Stereo Switched modes, drive ugv
				Jockey_call.request.drive = true;
				Jockey_call.request.vel  = 0.0; // m/s
				Jockey_call.request.yaw  = angularRate; // rad/sec?
				Jockey_call.request.time = angularTime; // ~Seconds

				Jockey_cli.call(Jockey_call);
					// Wait for the tb to stop driving
					do{ros::spinOnce();}while(drivestate);
			// }
			// ros::Duration(wait).sleep();
			// flop2Guide();
			updatePose();
		}
};

class hastExperiment
{
	private:
		/*---------  File Recorder ------------- */
		std::string s_filename, s_trial, s_dotm, s_root, s_handle, s_date, s_user;
		std::FILE * pFile;
		std::string s_ugv_basefootprint, s_drivestate_topic, s_drive_service;
		std::string s_ugvCmd_topic, s_pilot_service;
		std::string s_local_plan_topic, s_global_plan_topic, s_move_base_result_topic;
		double Pi;
		int L2Norm;

		/*--------- Flight Containers ------------- */
		bool FlightFlag;
		uint FlyToCount;
		double FlyToYaw, FlyToYawLast;
			cv::Mat FlyTo_g;

		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;

		/*----- Hast Channels */
 		bool ON, OFF;
			// Shutdown hast nodes

		tf::TransformListener listener;

		uavClass uav; // create uav from class
		ugvClass ugv; // create ugv from class

		ros::Publisher UGVposePublisher;
			std::string s_UGVgoal_topic, s_UGV1ClearCostmap_topic, s_ugvn_ckfstate_topic, s_ugvn_OdomSwitch_topic;
			geometry_msgs::Pose2D UGVgoalPose_msg; //pose message for uav goal location
		ros::ServiceClient GoalTagLocation_cli; // client to request goal tag location
			hast::ugvgoal goal_call;
			geometry_msgs::PointStamped goalPoint_ckf, goalPoint_map;
		ros::ServiceClient clearCostmaps_cli; // client to request goal tag location
			hast::null nullcall;

		// ros::ServiceClient clearCostmaps_cli; // client to request goal tag location
		// 	hast::null nullcall;


		nav_msgs::Path global_plan, local_plan;
			ros::Subscriber global_plan_sub, local_plan_sub;
			double yaw_at_end_radians, yaw_at_end_degrees; //wheelyaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)); // radians
			std::vector<geometry_msgs::PoseStamped> globalPoses, localPoses; //marker array as detected by image
			int path_msg_count;

		/*----- experiment types */
		bool picket_exp, arc_exp, pull_exp, abcde_exp, ugvcomplex_exp, action_exp, local_exp, cmdtilt;
		bool ugvfore_exp, ugvAutopilot, iflift, aprillog, step, slamtest;
		int phase;

		double start_wait;

	public:
		ros::Subscriber KillHast_sub, ActionStatus_sub, uav_pose_sub;
		
		ros::Publisher HastShutDown_pub;
			hast::flag ShutDown;


	hastExperiment()
	{
		/*---------  File Recorder Initilizer ------------- */
			s_handle = "experiment_";
			if(n.getParam("/hast/user", s_user))  {ROS_INFO("/hast/user: %s", s_user.c_str());} else {s_user = "benjamin";}
			if(n.getParam("/hast/date", s_date))  {ROS_INFO("/hast/date: %s", s_date.c_str());} else {s_date = "20181211";}
			if(n.getParam("/hast/trial", s_trial)){ROS_INFO("/hast/trial: %s", s_trial.c_str());} else {s_trial = "001";}
			
			s_root = "/home/" + s_user + "/ros/data/";
			s_dotm = ".m";
			s_filename = s_root + s_date + "/" + s_trial + "/" + s_handle + s_trial + s_dotm;
			ROS_INFO("experiment: %s", s_filename.c_str());
			
			pFile = std::fopen (s_filename.c_str(),"w");
			fprintf (pFile,"%% %s",s_filename.c_str());
			fprintf (pFile,"\n%%clc; \n%%clear all;\n%%close all;\n\n");

		/*--------- Initialize ROS Communication ------------- */
		/*-----  Publishers and Subscribers */
			// rostopic pub -1 /ardrone/flattrim std_msgs/Empty
			// rostopic pub -1 /ardrone/imu_recalib std_msgs/Empty
		/*----- UAV Channels */
			uav.imuRecal_pub	= n.advertise<std_msgs::Empty>("/ardrone/imu_recalib", 1000);
			uav.flatTrim_pub	= n.advertise<std_msgs::Empty>("/ardrone/flattrim", 1000);
			uav.Land_pub    	= n.advertise<std_msgs::Empty>("/ardrone/land", 10);
			uav.TakeOff_pub 	= n.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
			uav.Reset_pub 		= n.advertise<std_msgs::Empty>("/ardrone/reset", 10);
			uav.uavWaypoint_pub = n.advertise<sensor_msgs::PointCloud2>("/hast/uav/uavWaypointCloud", 1);

			uav.state_sub 		= n.subscribe("/hast/uav/state", 1, &uavClass::readState, &uav);
			uav.navState_cli 	= n.serviceClient<hast::uavnavstate>("/hast/service/uav/state");
			uav.control_cli		= n.serviceClient<hast::uavcontrolstate>("/hast/service/uav/ControlIsOn", true);
			uav.picket_cli		= n.serviceClient<hast::uavpicket>("/hast/service/uav/picketControl", true);
			uav.slamswitch_cli = n.serviceClient<hast::slamswitch>("/hast/service/uav/slamswitch", true);

			ros::param::get("~UGVgoal_topic",s_UGVgoal_topic); // frame for uav detecting april tags
			ros::param::get("~UGV1ClearCostmap_topic",s_UGV1ClearCostmap_topic); // frame for uav detecting april tags
			UGVposePublisher	= n.advertise<geometry_msgs::Pose2D>(s_UGVgoal_topic.c_str(), 10);
			GoalTagLocation_cli = n.serviceClient<hast::ugvgoal>("/hast/service/ugv/goal", true);
			// clearCostmaps_cli = n.serviceClient<hast::null>("/move_base/clear_costmaps", true);
			clearCostmaps_cli = n.serviceClient<hast::null>(s_UGV1ClearCostmap_topic, true);

			if(ros::param::get("~flytime",uav.flytime)){} else {uav.flytime = 10;} //time (seconds) for uav to make and keep a waypoint

		/*----- UGV Channels */
			ros::param::get("~drivestate_topic", s_drivestate_topic);
			ros::param::get("~move_base_result_topic", s_move_base_result_topic);
			// ros::param::get("~ugvCmd_topic", s_ugvCmd_topic);
			ros::param::get("~drive_service", s_drive_service);
			ros::param::get("~pilot_service", s_pilot_service);
			ros::param::get("~local_plan_topic", s_local_plan_topic);
			ros::param::get("~global_plan_topic", s_global_plan_topic);
			ros::param::get("~ugvn_ckfstate_topic", s_ugvn_ckfstate_topic);
			ros::param::get("~ugvn_OdomSwitch_topic", s_ugvn_OdomSwitch_topic);

			fprintf (pFile,"%% s_drivestate_topic: 	%s \n",s_drivestate_topic.c_str());
			// fprintf (pFile,"%% s_ugvCmd_topic: 		%s \n",s_ugvCmd_topic.c_str());
			fprintf (pFile,"%% s_drive_service: 	%s \n",s_drive_service.c_str());
			fprintf (pFile,"%% s_pilot_service: 	%s \n\n",s_pilot_service.c_str());

			ugv.DriveState_sub	= n.subscribe(s_drivestate_topic,	60, &ugvClass::updateDriveState, &ugv);
			ugv.pose_sub 		= n.subscribe(s_ugvn_ckfstate_topic, 	60, &ugvClass::readState, &ugv);
			ugv.Jockey_cli 		= n.serviceClient<hast::ugvdrive>(s_drive_service, true);
			ugv.Pilot_cli 		= n.serviceClient<hast::ugvautopilot>(s_pilot_service, true);
			ugv.StereoOdom_cli  = n.serviceClient<hast::flipflop>(s_ugvn_OdomSwitch_topic, true);

			// ugv.Cmd_pub			= n.advertise<geometry_msgs::Twist>("/hast/ugv/cmd_vel", 10);

			ROS_INFO("s_pilot_service: %s", s_drive_service.c_str());
			ROS_INFO("s_drive_service: %s", s_pilot_service.c_str());

			if(ros::param::get("~ugv_angularTime",ugv.angularTime)){} 	else {ugv.angularTime = 1.75;}
			if(ros::param::get("~ugv_angularRate",ugv.angularRate)){} 	else {ugv.angularRate = 0.175;}
			if(ros::param::get("~ugv_linearTime", ugv.linearTime)){} 	else {ugv.linearTime = 3.25;}
			if(ros::param::get("~ugv_linearRate", ugv.linearRate)){} 	else {ugv.linearRate = 0.2;}
			if(ros::param::get("~ugv_arcTime", ugv.arcTime)){} 			else {ugv.arcTime = 1.5;}


			ros::param::get("~ugv_goal_target_pose_header_frame_id", s_ugv_basefootprint);
			ugv.goal.target_pose.header.frame_id = s_ugv_basefootprint;

		/*----- Experiment Channels */
			HastShutDown_pub 	= n.advertise<hast::flag>("/hast/shutdown", 10);
			KillHast_sub	 	= n.subscribe("/hast/shutdown",1, &hastExperiment::killhast, this);
			ActionStatus_sub 	= n.subscribe(s_move_base_result_topic,	60, &hastExperiment::updateMoveBaseResult, this);

			ON = true;
			OFF = false;
			ShutDown.flag = ON;

		/*--------- Initialize Flight Variables ------------- */
		// uav.EstPosition_gl = (cv::Mat_<double>(3,1) << 2.5,0,0);
			FlightFlag = true;
			FlyToCount = 0;
			FlyToYaw = 0;
			FlyToYawLast = 0;

		/*--------- experiment parameters ------------- */
			if(ros::param::get("~phase",phase)){} 							else {phase = 0;}
			if(ros::param::get("~start_wait",start_wait)){} 		else {start_wait = 13;}

			if(ros::param::get("~abcde_exp",abcde_exp)){} 			else {abcde_exp = false;}
			if(ros::param::get("~action_exp",action_exp)){} 		else {action_exp = false;}
			if(ros::param::get("~aprillog",aprillog)){} 				else {aprillog = false;}
			if(ros::param::get("~arc_exp",arc_exp)){} 					else {arc_exp = false;}
			if(ros::param::get("~cmdtilt",cmdtilt)){} 					else {cmdtilt = false;}
			if(ros::param::get("~iflift",iflift)){} 						else {iflift = false;}
			if(ros::param::get("~local",local_exp)){} 					else {local_exp = false;}
			if(ros::param::get("~picket_exp",picket_exp)){} 		else {picket_exp = false;}
			if(ros::param::get("~pull_exp",pull_exp)){} 				else {pull_exp = false;}
			if(ros::param::get("~step",step)){} 								else {step = false;}
			if(ros::param::get("~slamtest",slamtest)){} 				else {slamtest = false;}
			if(ros::param::get("~ugvfore_exp",ugvfore_exp)){} 	else {ugvfore_exp = false;}
			if(ros::param::get("~ugvAutopilot",ugvAutopilot)){} else {ugvAutopilot = false;}
			if(ros::param::get("~ugvcomplex",ugvcomplex_exp)){} else {ugvcomplex_exp = false;}
			
			// spinsleep(15);
	}

	// move_base_msgs/MoveBaseActionResult
	void updateMoveBaseResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& resultMSG)
	{
		if ( resultMSG->status.status == 3)	
		{
			endExperiment();
		}
	}


	void updateUGVstate()
	{
		ugv.updatePose();
		// fprintf (pFile,"experiment.ugv.msgtime(%d,1) = %6.4f;\n", ugv.posecount, ugv.msgStamp);
		// fprintf (pFile,"experiment.ugv.time(%d,1) = %6.4f;\n", ugv.posecount, ros::Time::now().toSec());
		// fprintf (pFile,"experiment.ugv.msg_id(%d,1) = %u;\n", ugv.posecount, ugv.stateMsg_id);
		// fprintf (pFile,"experiment.ugv.EstPosition_gl(%d,:)  = [%6.10f, %6.10f, %6.10f];\n", ugv.posecount, ugv.EstPosition_gl.at<double>(0,0), ugv.EstPosition_gl.at<double>(1,0), ugv.EstPosition_gl.at<double>(2,0));
		// fprintf (pFile,"experiment.ugv.EstYaw_gl(%d,:)   = %6.10f;\n", ugv.posecount, ugv.EstYaw_gl);
	}

	/*####################        Start Experiment       #################### */
	void RunExperiment()
	{
		/*---------------------- Pre-Flight Preparations ---------------------- */
		spinsleep(start_wait);
		// ugv.angularTime = 1.75;
		// ugv.angularRate = 1; // .5 is too low to overcome friction
		// ugv.linearTime = 3.25;
		// ugv.linearRate = .2;
		// uav.flytime = 4;

		fprintf (pFile,"experiment.ugv.linearTime = %6.10f;\n", ugv.linearTime);
		fprintf (pFile,"experiment.ugv.linearRate = %6.10f;\n", ugv.linearRate);
		fprintf (pFile,"experiment.ugv.angularTime = %6.10f;\n", ugv.angularTime);
		fprintf (pFile,"experiment.ugv.angularRate = %6.10f;\n", ugv.angularRate);
		updateUGVstate();

		/*----------------------  Begin Experiment ---------------------- */
		ROS_INFO("experiment: ! ! ! Starting Experiment ! ! !");
		ROS_INFO("abcde_exp mode = %s", abcde_exp ? "true" : "false" );
		ROS_INFO("cmdtilt mode   = %s", cmdtilt ? "true" : "false" );
		ROS_INFO("slamtest mode   = %s", slamtest ? "true" : "false" );
		ROS_INFO("Phase  = %i", phase );
		fprintf (pFile,"experiment.start_time = %6.10f;\n", ros::Time::now().toSec());




		if (phase == 1) {
			ROS_WARN("phase1(); ...");
			phase1();
		} else if (phase == 2) {
			ROS_WARN("phase2(); ...");
			phase2();
		} else {
			ROS_WARN("phase0; ...");
			if (arc_exp) {experimentUGVarc();} // UAV suspended, pull cord for twist
			if (pull_exp) {experimentPullcord();} // UAV suspended, pull cord for twist
			if (cmdtilt) {experiment_cmdtilt();} // UAV suspended, pull cord for twist
			if (local_exp)
			{
				iflift=true;
				//uav takes off
				if (iflift)
				{
					ROS_WARN("uav.liftoff(); ...");
					uav.liftoff();
				} else {
					ROS_WARN("NO uav.liftoff(); ...");
				}
				//uav explores the space and builds the map
				UAVscanArena();
				// try guiding the ugv
				localPlanPicket();
			}
			if (action_exp)
			{
				iflift=true;
				goal_call.request.tagID = 7;
				// GoalTagLocation_cli.call(goal_call);
				clearCostmaps_cli.call(nullcall);

				//uav takes off
				if (iflift)
				{
					ROS_WARN("uav.liftoff(); ...");
					uav.liftoff();
				} else {
					ROS_WARN("NO uav.liftoff(); ...");
				}
				//uav explores the space and builds the map
				UAVscanArena();
				// try guiding the ugv
				experimentAction();
			} 
			if (abcde_exp) {experimentABCDE();} // UAV liftoff, cycle through five way-points
			if (ugvfore_exp) {experimentUGVforward();} 
			if (picket_exp) {experimentUGVpicket();} 
			if (aprillog) {experimentAprilLogger();} 
			if (slamtest) {experimentSlamTest();} 
			if (ugvcomplex_exp)
			{
				picket_exp=true;
				iflift=true;
				experimentUGVcomplex();
			}

		} 

		/*----------------------  End Experiment  ---------------------- */
		endExperiment();
	}

/****************************************************
*** Proposal Phases
****** Phase 1
* 3 landmarks
* UGV is stationary
* UAV lifts off, explores area, then lands at pre-defined location
****** Phase 2
* 3 landmarks
* UAV takes off, locates landmark, hovers over landmark
* UGV turns away from uav, drives a short distance, and then looks back
* UAV and UGV should update their poses when the UGV re-locates UAV
****** Phase 3
* 
****** Phase 4
**********************************************/

	void phase2()
	{/* ***** Phase 2
		* 3 landmarks
		* UAV takes off, locates landmark, hovers over landmark
		* UGV turns away from uav, drives a short distance, and then looks back
		* UAV and UGV should update their poses when the UGV re-locates UAV
		*/

		iflift=true;
		goal_call.request.tagID = 7;
		// GoalTagLocation_cli.call(goal_call);
		clearCostmaps_cli.call(nullcall);

		//uav takes off
		if (iflift)
		{
			ROS_WARN("uav.liftoff(); ...");
			uav.liftoff();
		} else {
			ROS_WARN("NO uav.liftoff(); ...");
		}
		//uav explores the space and builds the map

		cv::Mat waypoint;
		double waypointyaw;
		double scan_flytime = 10;
		uavHigher(0.5, 1.5);
		UAVstepTo( (cv::Mat_<double>(3,1) << 3.10, 0.00, 1.40), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << 2.00, 0.00, 1.20), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << 1.40, 0.00, 1.00), 0.00, scan_flytime);

		goal_call.request.tagID = 7;
		GoalTagLocation_cli.call(goal_call);
		clearCostmaps_cli.call(nullcall);
		spinsleep(2);

		if (goal_call.response.goalInMap) 
		{
			ROS_INFO("uav found goal tag, setting location as goal for uav");
			goalPoint_ckf = goal_call.response.goalPoint;
			ROS_INFO("goalPoint_ckf = [% -6.4f % -6.4f % -6.4f]", goalPoint_ckf.point.x, goalPoint_ckf.point.y, goalPoint_ckf.point.z);
			uav.setDesiredPosition(goalPoint_ckf.point.x, goalPoint_ckf.point.y, 1.00, 0.00);

			UGVgoalPose_msg.x = 0.0; UGVgoalPose_msg.y = 1.0; UGVgoalPose_msg.theta = 0.0;
			UGVposePublisher.publish(UGVgoalPose_msg); clearCostmaps_cli.call(nullcall); spinsleep(1);
			UGVposePublisher.publish(UGVgoalPose_msg); clearCostmaps_cli.call(nullcall); spinsleep(1);
		}
		// UAV hovers in place
		// ugv needs to drive to a different position and then look back at UAV
		ros::spin();
		// spinsleep(25); // sleep for 'x' second(s).
		uav.Land_pub.publish(uav.null);

	}


	void phase1()
	{/* ***** Phase 1
		* 3 landmarks
		* UGV is stationary
		* UAV lifts off, explores area, then lands at pre-defined location
		*/

		iflift=true;
		goal_call.request.tagID = 7;
		// GoalTagLocation_cli.call(goal_call);
		clearCostmaps_cli.call(nullcall);

		//uav takes off
		if (iflift)
		{
			ROS_WARN("uav.liftoff(); ...");
			uav.liftoff();
		} else {
			ROS_WARN("NO uav.liftoff(); ...");
		}
		//uav explores the space and builds the map

		cv::Mat waypoint;
		double waypointyaw;
		double scan_flytime = 10;
		uavHigher(0.5, 1.5);
		UAVstepTo( (cv::Mat_<double>(3,1) << 3.10, 0.00, 1.40), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << 2.00, 0.00, 1.20), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << 1.40, 0.00, 1.00), 0.00, scan_flytime);

		goal_call.request.tagID = 7;
		GoalTagLocation_cli.call(goal_call);
		clearCostmaps_cli.call(nullcall);
		spinsleep(2);

		if (goal_call.response.goalInMap) 
		{
			ROS_INFO("uav found goal tag, setting location as goal for uav");
			goalPoint_ckf = goal_call.response.goalPoint;
			ROS_INFO("goalPoint_ckf = [% -6.4f % -6.4f % -6.4f]", goalPoint_ckf.point.x, goalPoint_ckf.point.y, goalPoint_ckf.point.z);
		}
		spinsleep(1); // sleep for 'x' second(s).
		UAVstepTo( (cv::Mat_<double>(3,1) << goalPoint_ckf.point.x, goalPoint_ckf.point.y, 1.00), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << goalPoint_ckf.point.x, goalPoint_ckf.point.y, 0.50), 0.00, scan_flytime);
		UAVstepTo( (cv::Mat_<double>(3,1) << goalPoint_ckf.point.x, goalPoint_ckf.point.y, 0.25), 0.00, scan_flytime);
		spinsleep(1); // sleep for 'x' second(s).
		uav.Land_pub.publish(uav.null);
		spinsleep(5); // sleep for 'x' second(s).

	}

	void UAVscanArena()
	{
		cv::Mat waypoint;
		double waypointyaw;
		// double scan_flytime = 10; block experiment scan time
		double scan_flytime = 10;

		// ~~~~~~~~~~~~~~~~~~ use these for actual goal experiment
		uavHigher(0.5, 1.5);

		if(step)
		{
			// WaypointA = (cv::Mat_<double>(3,1) << 3.15, 0.80, 1.50); WaypointYawA = 0.00;
			// WaypointB = (cv::Mat_<double>(3,1) << 2.25, 0.70, 1.40); WaypointYawB = 0.00;
			// WaypointC = (cv::Mat_<double>(3,1) << 1.60, 0.60, 1.30); WaypointYawC = 0.00;
			// WaypointD = (cv::Mat_<double>(3,1) << 1.60, 0.00, 1.30); WaypointYawD = 0.00;
			// WaypointE = (cv::Mat_<double>(3,1) << 3.80, 0.00, 1.50); WaypointYawE = 0.00;
			// WaypointF = (cv::Mat_<double>(3,1) << 3.15,-0.80, 1.50); WaypointYawF = 0.00;
			// WaypointG = (cv::Mat_<double>(3,1) << 2.25,-0.70, 1.40); WaypointYawB = 0.00;
			// WaypointH = (cv::Mat_<double>(3,1) << 1.60,-0.60, 1.30); WaypointYawG = 0.00;
	

			// lawnmower path:
			// UAVstepTo( (cv::Mat_<double>(3,1) << 3.15, 0.60, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 2.35, 0.60, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 1.60, 0.60, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 1.60, 0.00, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 2.35, 0.00, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 3.35, 0.00, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 3.25,-0.70, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 2.35,-0.70, 1.50), 0.00, scan_flytime);
			// UAVstepTo( (cv::Mat_<double>(3,1) << 1.60,-0.70, 1.50), 0.00, scan_flytime);

			// line path
			UAVstepTo( (cv::Mat_<double>(3,1) << 3.1, 0.00, 1.40), 0.00, scan_flytime);
			UAVstepTo( (cv::Mat_<double>(3,1) << 2.0, 0.00, 1.20), 0.00, scan_flytime);
			UAVstepTo( (cv::Mat_<double>(3,1) << 1.40, 0.00, 1.00), 0.00, scan_flytime);
			



			// UAVstepTo( uav.WaypointB, 0.00, scan_flytime);
			// UAVstepTo( uav.WaypointC, 0.00, scan_flytime);
			// UAVstepTo( uav.WaypointD, 0.00, scan_flytime);
			// // UAVstepTo( uav.WaypointE, 0.00, scan_flytime);
			// UAVstepTo( uav.WaypointF, 0.00, scan_flytime);
			// UAVstepTo( uav.WaypointG, 0.00, scan_flytime);
			// UAVstepTo( uav.WaypointH, 0.00, scan_flytime);

			// waypoint = (cv::Mat_<double>(3,1) << 2.50, -0.2, 1.5); waypointyaw = 0.00;
			// UAVstepTo( waypoint, waypointyaw, scan_flytime); 
			// waypoint = (cv::Mat_<double>(3,1) << 2.00,  0.2, 1.5); waypointyaw = 0.00;
			// UAVstepTo( waypoint, waypointyaw, scan_flytime); 
			// waypoint = (cv::Mat_<double>(3,1) << 1.75,  0.0, 1.5); waypointyaw = 0.00;
			// UAVstepTo( waypoint, waypointyaw, scan_flytime); 

		} else {
			waypoint = (cv::Mat_<double>(3,1) << 2.50, -0.2, 1.5); waypointyaw = 0.00;
			linearTrajTo(uav.EstPosition_gl, waypoint, uav.EstYaw_gl, waypointyaw, scan_flytime);
			waypoint = (cv::Mat_<double>(3,1) << 2.00, -0.2, 1.5); waypointyaw = 0.00;
			linearTrajTo(uav.EstPosition_gl, waypoint, uav.EstYaw_gl, waypointyaw, scan_flytime);
		}

		//uav comes back and pickets for ugv
		uav.setPicket(4.0, 0, 1.80, 0); // use this to set picket location of uav
		spinsleep(5);
	}

	void localPlanPicket()
	{
		local_plan_sub  = n.subscribe(s_local_plan_topic, 	1, &hastExperiment::updateLocalPath, this);
		path_msg_count = 0;
		// ugv.global_plan_sub = n.subscribe(s_global_plan_topic, 	1, &hastExperiment::updateGlobalPath, this);

		experimentAction();
	}

	void updateLocalPath(const nav_msgs::Path::ConstPtr& path_msg)
	{
		localPoses.clear();
		// localPoses.push_back(path_msg->poses);
		localPoses = path_msg->poses;
		uint numposes = localPoses.size();
		// ROS_INFO ("localPoses.size() : %u ", numposes);

		double x_end = localPoses[numposes-1].pose.position.x;
		double y_end = localPoses[numposes-1].pose.position.y;
		double z_end = localPoses[numposes-1].pose.position.z;

		double qx = localPoses[numposes-1].pose.orientation.x;
		double qy = localPoses[numposes-1].pose.orientation.y;
		double qz = localPoses[numposes-1].pose.orientation.z;
		double qw = localPoses[numposes-1].pose.orientation.w;

		double yaw_end = atan2(2*(qx*qy-qz*qw),1-2*(qy*qy+qz*qz)); // radians

		cv::Mat R_end, p_end, picket_end;


		p_end = (cv::Mat_<double>(3, 1) << 
			localPoses[numposes-1].pose.position.x,
			localPoses[numposes-1].pose.position.y,
			localPoses[numposes-1].pose.position.z);

		R_end = (cv::Mat_<double>(3, 3) <<
			 cos(yaw_end),  sin(yaw_end), 0,
			-sin(yaw_end),  cos(yaw_end), 0,
			 0,0,1);

		picket_end = (R_end * uav.picket_vector) + p_end;

		path_msg_count +=1;
		fprintf (pFile,"experiment.plan.time(%i,1) = %6.10f;\n", path_msg_count, ros::Time::now().toSec());
		fprintf (pFile,"experiment.plan.path_end(%i,:) = [%6.10f %6.10f %6.10f %6.10f ];\n", 
			path_msg_count, x_end, y_end,z_end, yaw_end);
		fprintf (pFile,"experiment.plan.uav_waypoint(%i,:) = [%6.10f %6.10f %6.10f %6.10f];\n", 
			path_msg_count, 
			picket_end.at<double>(0,0),
			picket_end.at<double>(1,0),
			picket_end.at<double>(2,0),
			yaw_end);

		uav.setDesiredPosition(
			picket_end.at<double>(0,0),
			picket_end.at<double>(1,0),
			picket_end.at<double>(2,0),
			yaw_end);




		// ROS_INFO ("localPoses end [X,Y,Z, Yaw] : % -3.3f % -3.3f % -3.3f % -3.3f", x_end, y_end,z_end, yaw_end);
		// ROS_INFO ("uav picket_end [X,Y,Z, Yaw] : % -3.3f % -3.3f % -3.3f % -3.3f\n", 
		// 	picket_end.at<double>(0,0),
		// 	picket_end.at<double>(1,0),
		// 	picket_end.at<double>(2,0),
		// 	yaw_end);

		// double yaw_at_end_radians, yaw_at_end_degrees; //wheelyaw_q = -atan2(2*(x*y-z*w),1-2*(y*y+z*z)); // radians

			//   - 
			//     header: 
			//       seq: 0
			//       stamp: 
			//         secs: 14
			//         nsecs: 626000000
			//       frame_id: "hast/kobuki/odom"
			//     pose: 
			//       position: 
			//         x: 1.63220670074
			//         y: 0.344350837171
			//         z: 0.0
			//       orientation: 
			//         x: 0.0
			//         y: 0.0
			//         z: 0.317152161803
			//         w: 0.948374665553
			// ---
	}

	// void updateGlobalPath(const nav_msgs::Path::ConstPtr& path_msg)
	// {
	// 	globalPoses.clear();
	// 	// globalPoses.push_back(path_msg->poses);
	// }

	void experimentSlamTest()
	{
		/* 
			UAV NOT suspended from tripod
			liftoff, then cycle through five way-points
		*/
		
		// uav_pose_sub 	= n.subscribe("/hast/uav/pose", 1, &hastExperiment::uavPoseSub , this);

		fprintf (pFile,"experiment.type.slamtest = %s;\n", slamtest ? "true" : "false" );


		ROS_WARN("uav.liftoff(); ...");
		uav.liftoff();
		uav.flytime = 5;

		UAVstepTo((cv::Mat_<double>(3,1) << 1.75, 0.00, 0.85), 0.00, 3*uav.flytime); 

		slamSwitch();

		UAVstepTo((cv::Mat_<double>(3,1) << 1.25, -0.20, 0.85), 0.00, uav.flytime); 
		UAVstepTo((cv::Mat_<double>(3,1) << 1.75, -0.20, 0.85), 0.00, uav.flytime); 
		UAVstepTo((cv::Mat_<double>(3,1) << 2.25, -0.20, 0.85), 0.00, uav.flytime); 
		UAVstepTo((cv::Mat_<double>(3,1) << 1.75, -0.20, 0.85), 0.00, uav.flytime); 


	}

	void slamSwitch()
	{
		uav.pose_msg.position.x = uav.EstPosition_gl.at<double>(0, 0);
		uav.pose_msg.position.y = uav.EstPosition_gl.at<double>(1, 0);
		uav.pose_msg.position.z = uav.EstPosition_gl.at<double>(2, 0);
		uav.pose_msg.orientation = tf::createQuaternionMsgFromYaw(uav.Pi * uav.EstYaw_gl / 180);
		uav.pose_msg.header.seq = ++uav.pose_msg_seq;
		uav.pose_msg.header.stamp = ros::Time::now();
		uav.pose_msg.header.frame_id = "/map";


		uav.slamswitch_call.request.flip = true; // Set to activate slam
		uav.slamswitch_call.request.pose = uav.pose_msg;

		ROS_WARN("slamswitch(); ...");
		uav.slamswitch_cli.call(uav.slamswitch_call);
	}


	void experimentAction()
	{
		goal_call.request.tagID = 7;
		GoalTagLocation_cli.call(goal_call);
		clearCostmaps_cli.call(nullcall);
		spinsleep(2);

		if (goal_call.response.goalInMap) 
		{
			ROS_INFO("uav found goal tag, setting location as goal for ugv");
			goalPoint_ckf = goal_call.response.goalPoint;
				std::string target_frame = "/map";
				listener.transformPoint(target_frame, goalPoint_ckf, goalPoint_map);
				UGVgoalPose_msg.x = goalPoint_map.point.x;
				UGVgoalPose_msg.y = goalPoint_map.point.y;
			// UGVgoalPose_msg.x = goal_call.response.goalP.x;
			// UGVgoalPose_msg.y = goal_call.response.goalP.y;
			UGVgoalPose_msg.theta = goal_call.response.goalYaw; // set the ugv goal yaw to match tag yaw, for some reason
			UGVposePublisher.publish(UGVgoalPose_msg);
			clearCostmaps_cli.call(nullcall);
			spinsleep(1);
			UGVposePublisher.publish(UGVgoalPose_msg);
			clearCostmaps_cli.call(nullcall);
			spinsleep(1);
		}
		ros::spin();
	}

	void experiment_cmdtilt() 
	{
		/* 
			UAV NOT suspended from tripod
			liftoff, then cycle through five way-points
		*/
		ROS_WARN("uav.liftoff(); ...");
		uav.liftoff();
		uav.flytime = 15;
		spinsleep(1);

		double uprate, roll, pitch, yawrate, cmd_sleep_time;
		uprate = 0;
		yawrate = 0;
		roll = 0;
		cmd_sleep_time = 1;

		cmd_sleep(pitch=0.00, roll=0, uprate=0, yawrate=0, cmd_sleep_time=1.0);
		cmd_sleep(pitch=0.075, roll=0, uprate=0, yawrate=0, cmd_sleep_time=3.0);
		cmd_sleep(pitch= 0.00, roll=0, uprate=0, yawrate=0, cmd_sleep_time=3.0);
		// double sleepStartTime = ros::Time::now().toSec();
		// while((ros::Time::now().toSec() - sleepStartTime) < 4)
		// {
		// 	pitch = 0.0001;
		// 		uav.cmdtilt(uprate, roll, pitch, yawrate);
		// 		spin_sleep(0.05);
		// 	pitch =-0.0001;
		// 		uav.cmdtilt(uprate, roll, pitch, yawrate);
		// 		spin_sleep(0.05);
		// }
		
		cmd_sleep(pitch=-0.075, roll=0, uprate=0, yawrate=0, cmd_sleep_time=3.0);
		cmd_sleep(pitch= 0.00, roll=0, uprate=0, yawrate=0, cmd_sleep_time=1.0);

	}

	void cmd_sleep(double pitch, double roll, double uprate, double yawrate, double sleep_time) 
	{
		ROS_WARN("uav.cmdtilt(uprate:=% -2.2f, roll:=% -2.2f, pitch:=% -2.2f, yawrate:=% -2.2f);",uprate, roll, pitch, yawrate);
		double sleepStartTime = ros::Time::now().toSec();
		while((ros::Time::now().toSec() - sleepStartTime) < sleep_time)
		{
				uav.cmdtilt(uprate, roll, pitch, yawrate);
				spin_sleep(0.05);
		}

	}

	void experimentABCDE() 
	{
		/* 
			UAV NOT suspended from tripod
			liftoff, then cycle through five way-points
		*/
		ROS_INFO("abcde_exp mode = %s", abcde_exp ? "true" : "false" );
		fprintf (pFile,"experiment.type.abcde = %s;\n", abcde_exp ? "true" : "false" );
		ROS_WARN("uav.liftoff(); ...");
		uav.liftoff();
		uav.flytime = 5;
		// spinsleep(1);

		if(step)
		{
			// lawnmower path:
			uav.flytime = 8;
			// UAVstepTo( uav.WaypointA, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointB, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointC, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointD, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointE, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointF, 0.00, uav.flytime);
			// UAVstepTo( uav.WaypointG, 0.00, uav.flytime);

			// UAVstepTo( uav.WaypointA, uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointA, 0, uav.flytime); UAVstepTo( uav.WaypointA,-uav.WaypointYaw0, uav.flytime);
			// UAVstepTo( uav.WaypointB,-uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointB, 0, uav.flytime); UAVstepTo( uav.WaypointB, uav.WaypointYaw0, uav.flytime);
			// UAVstepTo( uav.WaypointC, uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointC, 0, uav.flytime); UAVstepTo( uav.WaypointC,-uav.WaypointYaw0, uav.flytime); 
			// UAVstepTo( uav.WaypointD,-uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointD, 0, uav.flytime); UAVstepTo( uav.WaypointD, uav.WaypointYaw0, uav.flytime); 
			// UAVstepTo( uav.WaypointE, uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointE, 0, uav.flytime); UAVstepTo( uav.WaypointE,-uav.WaypointYaw0, uav.flytime); 
			// UAVstepTo( uav.WaypointF,-uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointF, 0, uav.flytime); UAVstepTo( uav.WaypointF, uav.WaypointYaw0, uav.flytime); 
			// UAVstepTo( uav.WaypointG, uav.WaypointYaw0, uav.flytime); UAVstepTo( uav.WaypointG, 0, uav.flytime); UAVstepTo( uav.WaypointG,-uav.WaypointYaw0, uav.flytime); 

			// Close range way points

			UAVstepTo((cv::Mat_<double>(3,1) << 3.00, 0.00, 1.00), 0.00, uav.flytime); 
			UAVstepTo((cv::Mat_<double>(3,1) << 1.50, 0.00, 0.70), 0.00, uav.flytime); 
			UAVstepTo((cv::Mat_<double>(3,1) << 1.00, 0.00, 0.70), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 0.80, 0.00, 0.60), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 0.60, 0.00, 0.60), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 0.80, 0.00, 0.60), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 1.00, 0.00, 0.70), 0.00, uav.flytime); 
			UAVstepTo((cv::Mat_<double>(3,1) << 2.00, 0.00, 0.80), 0.00, uav.flytime); 
			UAVstepTo((cv::Mat_<double>(3,1) << 3.00, 0.00, 1.00), 0.00, uav.flytime); 

			// Far range way points
			// UAVstepTo((cv::Mat_<double>(3,1) << 3.75, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 4.25, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 4.75, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 5.25, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 5.75, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 5.25, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 4.75, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 4.25, 0.00, 1.80), 0.00, uav.flytime); 
			// UAVstepTo((cv::Mat_<double>(3,1) << 3.75, 0.00, 1.80), 0.00, uav.flytime); 

			// UAVstepTo( uav.WaypointE, uav.WaypointYawE, uav.flytime);
		} else {
			linearTrajTo(uav.EstPosition_gl, uav.WaypointA, uav.EstYaw_gl, uav.WaypointYawA, uav.flytime); // to deal with liftoff transients
			// linearTrajTo(uav.EstPosition_gl, uav.WaypointA, uav.EstYaw_gl, uav.WaypointYawA, uav.flytime); // to deal with liftoff transients

			linearTrajTo(uav.EstPosition_gl, uav.WaypointB, uav.EstYaw_gl, uav.WaypointYawB, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, uav.WaypointC, uav.EstYaw_gl, uav.WaypointYawC, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, uav.WaypointD, uav.EstYaw_gl, uav.WaypointYawD, uav.flytime);
			// linearTrajTo(uav.EstPosition_gl, uav.WaypointE, uav.EstYaw_gl, uav.WaypointYawE, uav.flytime);
		}
	}

	void experimentUGVforward()
	{
		/* 
			Using the old code to 
			do a simple forward motion of ugv
		*/
		ROS_INFO("ugvfore_exp mode = %s", ugvfore_exp ? "true" : "false" );
		fprintf (pFile,"experiment.type.ugvfore = %s;\n", ugvfore_exp ? "true" : "false" );
		// linearTrajTo(uav.EstPosition_gl, uav.WaypointA, uav.EstYaw_gl, uav.WaypointYawA, uav.flytime); // to deal with liftoff transients
		linearTrajTo(uav.EstPosition_gl, uav.WaypointA, uav.EstYaw_gl, uav.WaypointYawA, 3); // to deal with liftoff transients
		ugv.ForwardOnce();
		spinsleep(3);
		ugv.ArcOnce(); 
		ugv.angularRate = -ugv.angularRate;
		ugv.ArcOnce(); 
	}

	void experimentUGVpicket()
	{
		/* 
			set the uav to a fixed offset from ugv, then move forward.
		*/
		int picketcounter; picketcounter = 0;
		ROS_INFO("picket_exp mode = %s", picket_exp ? "true" : "false" );
		fprintf (pFile,"experiment.type.picket = %s;\n", picket_exp ? "true" : "false" );
		uav.setPicket(1.375, 0, 0.88, 0); // use this to set picket location of uav
		// uav.setPicket(1.88, 0, 1.05, 0); default picket location for icra2019 experiments
		
		if (iflift)
		{
			ROS_WARN("uav.liftoff(); ...");
			uav.liftoff();
		} else {
			ROS_WARN("NO uav.liftoff(); ...");
		}

		uav.setPicket(1.88, 0, 1.05, 0); // use this to set picket location of uav
		spinsleep(8);

		ROS_INFO("ugvAutopilot mode = %s", ugvAutopilot ? "true" : "false" );

		fprintf (pFile,"experiment.picket.time(%d,1) = %6.14f;\n", ++picketcounter, ros::Time::now().toSec());
		if (ugvAutopilot) 
		{
			ugv.goalP_gl = (cv::Mat_<double>(3, 1) << 1,0.5,0); // meters?
			ugv.yawAtGoal_gl = -30; // degrees?
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ROS_INFO ("ugvPilot to [X,Y,Z] : [%6.10f %6.10f %6.10f];\n", ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); 
		} else {
			fprintf (pFile,"experiment.picket.linearRate(%d,1) = %6.10f;\n", picketcounter, ugv.linearRate);
			fprintf (pFile,"experiment.picket.angularRate(%d,1) = %6.10f;\n", picketcounter, ugv.angularRate);
			ugv.ForwardOnce();	
		}
	}

	void experimentUGVcomplex()
	{
		/* 
			set the uav to a fixed offset from ugv, then move forward.
		*/
		int picketcounter; picketcounter = 0;
		fprintf (pFile,"experiment.type.picket = %s;\n", picket_exp ? "true" : "false" );
		uav.setPicket(1.625, 0, 1.25, 0); // use this to set picket location of uav
		
		if (iflift)
		{
			ROS_WARN("uav.liftoff(); ...");
			uav.liftoff();
		} else {
			ROS_WARN("NO uav.liftoff(); ...");
		}

		uav.setPicket(1.88, 0, 1.05, 0); // use this to set picket location of uav
		spinsleep(8);

		ROS_INFO("ugvAutopilot mode = %s", ugvAutopilot ? "true" : "false" );

		fprintf (pFile,"experiment.picket.time(%d,1) = %6.14f;\n", ++picketcounter, ros::Time::now().toSec());

		// ZigZag
		double x1 = 1.0;	double y1 = 1.0;	double yaw1 = 0;
		double x2 = 2.0;	double y2 = 0.0; 	double yaw2 = 0;
		double x3 = 3.0; 	double y3 = 1.0; 	double yaw3 = 0;
		double x4 = 4.0;	double y4 = 0.0; 	double yaw4 = 0;
		
		// Octagon
		// double x1 = 1;		double y1 = 0;		double yaw1 = 0;
		// double x2 = 1.7071;	double y2 = 0.7071; double yaw2 = 0;
		// double x3 = 1.7071; double y3 = 1.7071; double yaw3 = 0;
		// double x4 = 1;		double y4 = 2.4142; double yaw4 = 0;


		/* first maneuver */
		ugv.goalP_gl = (cv::Mat_<double>(3, 1) << x1,y1,0); // meters, not relative
		ugv.yawAtGoal_gl = yaw1; // degrees, not relative
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); //false is for turn only

		/* second maneuver */
		ugv.goalP_gl = (cv::Mat_<double>(3, 1) << x2,y2,0); // meters, not relative
		ugv.yawAtGoal_gl = yaw2; // degrees, not relative
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); 

		/* second maneuver */
		ugv.goalP_gl = (cv::Mat_<double>(3, 1) << x3,y3,0); // meters, not relative
		ugv.yawAtGoal_gl = yaw3; // degrees, not relative
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); 

		/* second maneuver */
		ugv.goalP_gl = (cv::Mat_<double>(3, 1) << x4,y4,0); // meters, not relative
		ugv.yawAtGoal_gl = yaw4; // degrees, not relative
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); 

		/* last maneuver */
		double xend = ugv.EstPosition_gl.at<double>(0,0) + 0.1*cos(ugv.EstYaw_gl*3.14/180);		
		double yend = ugv.EstPosition_gl.at<double>(1,0) + 0.1*sin(ugv.EstYaw_gl*3.14/180);		
		ugv.goalP_gl = (cv::Mat_<double>(3, 1) << xend,yend,0); // meters, not relative
		ugv.yawAtGoal_gl = yaw4; // degrees, not relative
			fprintf (pFile,"experiment.pilot.goalP_gl(%d,:) = [%6.10f %6.10f %6.10f];\n", picketcounter, ugv.goalP_gl.at<double>(0,0), ugv.goalP_gl.at<double>(1,0), ugv.goalP_gl.at<double>(2,0));
			fprintf (pFile,"experiment.pilot.yawAtGoal_gl(%d,1) = %6.10f;\n", picketcounter, ugv.yawAtGoal_gl);
			ugv.pilotTo(ugv.goalP_gl, ugv.yawAtGoal_gl, false); 
	}

	void experimentUGVarc()
	{
		/* 
			Using the old code to 
			do a simple forward motion of ugv
		*/
		ROS_INFO("arc mode = %s", arc_exp ? "true" : "false" );
		fprintf (pFile,"experiment.type.ugvfore = %s;\n", arc_exp ? "true" : "false" );
		
		linearTrajTo(uav.EstPosition_gl, uav.WaypointA, uav.EstYaw_gl, uav.WaypointYawA, 3); // to deal with liftoff transients

		ugv.ArcOnce(); 
		ugv.angularRate = -ugv.angularRate;
		ugv.ArcOnce(); 
	}

	void experimentAprilLogger()
	{
		/* 
			UAV is suspended from tripod, 
			user pulls cord to make uav twist
		*/
		ROS_INFO("aprillog mode = %s", aprillog ? "true" : "false" );
		ROS_WARN("Move april tage around FOV");
		fprintf (pFile,"experiment.type.april = %s;\n", aprillog ? "true" : "false" );
		spinsleep(60);
	}

	void experimentPullcord()
	{
		/* 
			UAV is suspended from tripod, 
			user pulls cord to make uav twist
		*/
		ROS_INFO("pull_exp mode = %s", pull_exp ? "true" : "false" );
		ROS_WARN("PULL CORD!!!");
		fprintf (pFile,"experiment.type.pull = %s;\n", pull_exp ? "true" : "false" );
		spinsleep(15);
	}

	void endExperiment() 
	{
		/*  Sends landing command and shutdown messages */
		ROS_WARN("experiment: landing...\n");
			ros::Duration(2).sleep(); // sleep for 'x' second(s).
			uav.Land_pub.publish(uav.null);
		/*---------------------- End Experiment /*---------------------- */
		ros::Duration(0.5).sleep();
		// mac ROS_INFO("experiment: Flight Ended");
		ros::Duration(2).sleep(); // sleep for 'x' second(s).
		HastShutDown_pub.publish(ShutDown);
		ros::Duration(2).sleep(); // sleep for 'x' second(s).
		ShutDown.flag = OFF;
		HastShutDown_pub.publish(ShutDown);
		ros::Duration(1).sleep(); // sleep for 'x' second(s).
		// mac ROS_INFO("experiment: Node Closing...");
		ros::shutdown();
	}

	void uavHigher(double upRate, double newAltitude)
	{
		double eps = 0.25;
		while (abs(uav.EstPosition_gl.at<double>(2,0) - newAltitude) > eps)
		{
			uav.increaseAltitude(upRate);
			ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
		}
	}

	void UAVstepTo(cv::Mat ToPos, double ToYaw, double timeSpan)
	{
		ROS_WARN("UAVstepTo: ToPos = [%6.4f %6.4f %6.4f] [%6.4f], FlyTime = [%6.4f]", ToPos.at<double>(0,0), ToPos.at<double>(1,0), ToPos.at<double>(2,0), ToYaw, timeSpan);
		// mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
		// mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);

		double TrajStartTime = ros::Time::now().toSec();
		double NewX, NewY, NewZ;
		// ROS_WARN("experiment: delta = NewPos - LastPos = [%6.4f %6.4f %6.4f]",delta.at<double>(0,0), delta.at<double>(1,0), delta.at<double>(2,0) );

		while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
		{
		   // run trajectory path
			//update states & calculate desired states
			ros::spinOnce();
			NewX = ToPos.at<double>(0,0);
			NewY = ToPos.at<double>(1,0);
			NewZ = ToPos.at<double>(2,0);
			++FlyToCount;
			// fprintf (pFile,"experiment.uav.FlyTime(%d)  = %f;\n", FlyToCount, ros::Time::now().toSec());
			// fprintf (pFile,"experiment.uav.DesiredState_gl(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, NewX, NewY, NewZ, ToYaw);
			// fprintf (pFile,"experiment.uav.Waypoint(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
			// fprintf (pFile,"experiment.uav.CurrentState_gl(%d,:) = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
			uav.setDesiredPosition(NewX, NewY, NewZ, ToYaw);
			// // mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
			// // mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);
			ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
		}
	}

	void linearTrajTo(cv::Mat OriginalPos, cv::Mat ToPos, double OriginalYaw, double ToYaw, double timeSpan)
	{
		ROS_WARN("linearTrajTo: ToPos = [%6.4f %6.4f %6.4f] [%6.4f], FlyTime = [%6.4f]", ToPos.at<double>(0,0), ToPos.at<double>(1,0), ToPos.at<double>(2,0), ToYaw, timeSpan);
		// mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
		// mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);

		double TrajStartTime = ros::Time::now().toSec();
		cv::Mat mdelta = (ToPos - OriginalPos)/timeSpan;
		double mYaw = (ToYaw - OriginalYaw)/timeSpan;
		double NewX, NewY, NewZ, NewYaw;
		// ROS_WARN("experiment: delta = NewPos - LastPos = [%6.4f %6.4f %6.4f]",delta.at<double>(0,0), delta.at<double>(1,0), delta.at<double>(2,0) );

		while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
		{
		   // run trajectory path
			//update states & calculate desired states
			ros::spinOnce();
			double deltaTime = ros::Time::now().toSec() - TrajStartTime;
			NewX = OriginalPos.at<double>(0,0) + mdelta.at<double>(0,0) * deltaTime;
			NewY = OriginalPos.at<double>(1,0) + mdelta.at<double>(1,0) * deltaTime;
			// NewZ = OriginalPos.at<double>(2,0) + mdelta.at<double>(2,0) * deltaTime;
			NewZ = ToPos.at<double>(2,0);
			NewYaw = OriginalYaw + mYaw * deltaTime;
			fprintf (pFile,"experiment.uav.FlyTime(%d)  = %f;\n", ++FlyToCount, deltaTime+TrajStartTime);
			fprintf (pFile,"experiment.uav.DesiredState_gl(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, NewX, NewY, NewZ, NewYaw);
			fprintf (pFile,"experiment.uav.Waypoint(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
			fprintf (pFile,"experiment.uav.CurrentState_gl(%d,:) = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
			uav.setDesiredPosition(NewX, NewY, NewZ, NewYaw);
			// // mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
			// // mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);
			ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
		}
	}

	void pronavTo(cv::Mat ToPos, double ToYaw, double timeSpan)
	{
		// mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
		// mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);

		double TrajStartTime = ros::Time::now().toSec();
		double NewX, NewY, NewZ, NewYaw;

		while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
		{
		   // run trajectory path
			//update states & calculate desired states
			ros::spinOnce();
			double deltaTime = ros::Time::now().toSec() - TrajStartTime;
			NewX = ToPos.at<double>(0,0);
			NewY = ToPos.at<double>(1,0);
			NewZ = ToPos.at<double>(2,0);
			NewYaw = ToYaw;
			fprintf (pFile,"experiment.uav.FlyTime(%d)  = %f;\n", ++FlyToCount, deltaTime+TrajStartTime);
			fprintf (pFile,"experiment.uav.DesiredState_gl(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, NewX, NewY, NewZ, NewYaw);
			fprintf (pFile,"experiment.uav.Waypoint(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
			fprintf (pFile,"experiment.uav.CurrentState_gl(%d,:) = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
			uav.setDesiredPosition(NewX, NewY, NewZ, NewYaw);
			// // mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
			// // mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);
			ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
		}
	}

	void hybridnavTo(cv::Mat OriginalPos, cv::Mat ToPos, double OriginalYaw, double ToYaw, double timeSpan)
	{
		// mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
		// mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);

		double TrajStartTime = ros::Time::now().toSec();
		cv::Mat mdelta = (ToPos - OriginalPos)/timeSpan;
		double mYaw = (ToYaw - OriginalYaw)/timeSpan;
		double NewX, NewY, NewZ, NewYaw;
		// ROS_WARN("experiment: delta = NewPos - LastPos = [%6.4f %6.4f %6.4f]",delta.at<double>(0,0), delta.at<double>(1,0), delta.at<double>(2,0) );

		while((ros::Time::now().toSec() - TrajStartTime) < timeSpan)
		{
		   // run trajectory path
			//update states & calculate desired states
			ros::spinOnce();
			double deltaTime = ros::Time::now().toSec() - TrajStartTime;
			NewX = OriginalPos.at<double>(0,0) + mdelta.at<double>(0,0) * deltaTime;
			NewY = ToPos.at<double>(1,0);
			// NewZ = OriginalPos.at<double>(2,0) + mdelta.at<double>(2,0) * deltaTime;
			NewZ = ToPos.at<double>(2,0);
			NewYaw = ToYaw;
			fprintf (pFile,"experiment.uav.FlyTime(%d)  = %f;\n", ++FlyToCount, deltaTime+TrajStartTime);
			fprintf (pFile,"experiment.uav.DesiredState_gl(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, NewX, NewY, NewZ, NewYaw);
			fprintf (pFile,"experiment.uav.Waypoint(%d,:)  = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, ToPos.at<double>(0,0), ToPos.at<double>(0,1), ToPos.at<double>(0,2), ToYaw);
			fprintf (pFile,"experiment.uav.CurrentState_gl(%d,:) = [%6.4f, %6.4f, %6.4f, %6.4f];\n", FlyToCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0),uav.EstPosition_gl.at<double>(2,0), uav.EstYaw_gl);
			uav.setDesiredPosition(NewX, NewY, NewZ, NewYaw);
			// // mac ROS_INFO("experiment: uav.DesiredPos_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.Waypoint0.at<double>(0,0), uav.Waypoint0.at<double>(1,0), uav.Waypoint0.at<double>(2,0), 0.0);
			// // mac ROS_INFO("experiment: uav.EstPosition_gl: [%6.4f %6.4f %6.4f : %6.4f]", uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0),uav.EstYaw_gl);
			ros::Duration(0.25).sleep(); // sleep for 'x' second(s).
		}
	}

	/* ROS level utility functions */
	void killhast(const hast::flag::ConstPtr& shutdownflag)
	{ // node cleanup for end of experiment
		if(shutdownflag->flag)
		{
			// mac ROS_INFO("experiment: Node going down hard!");
			ros::Duration(0.5).sleep();
			uav.Land_pub.publish(uav.null);
			fprintf (pFile,"experiment.stop_time = %6.10f;\n", ros::Time::now().toSec());
			// mac ROS_INFO("experiment: Flight Ended");
			ros::Duration(2).sleep(); // sleep for 'x' second(s).
			ros::shutdown();
		}
	}

	void spinsleep(int dwell_seconds)
	{
		for(int b = 0; b < dwell_seconds; ++b)
		{
			ros::spinOnce();
			ros::Duration(1).sleep(); // sleep for 'x' second(s).
		}
	}

	void spin_sleep(double sleep_time)
	{
		double sleepStartTime = ros::Time::now().toSec();

		while((ros::Time::now().toSec() - sleepStartTime) < sleep_time)
		{
			ros::spinOnce();
		}

	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hastExperiment");
	// ros::Duration(2).sleep(); // sleep for 'x' second(s).
	hastExperiment hE;
	ros::Duration(1).sleep(); // sleep for 'x' second(s).
	hE.RunExperiment();
	return 0;
}


