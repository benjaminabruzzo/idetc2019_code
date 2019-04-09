#include "genheaders.hpp"
#include "utils.hpp"
#include "fileops.hpp"
#include "apriltagclass.hpp"
#include "ckfClass.hpp"
#include "DiscreteKF.hpp"
#include "hastSLAM.hpp"
#include "hastUAV.hpp"
#include "hastUGV.hpp"
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << std::setw(2) << std::setfill('0')  << n ;
		return stm.str() ;
	}
}// s_imString = s_root + s_date + "/" + s_run + "/original/left_image_" + patch::to_string(calledCount) + ".png";



class uavSLAM
{
	private:
		/*---------  Assorted Constants ------------- */
			int L2Norm;
		/*---------  File Recorder ------------- */
			std::string s_trial, s_date, s_user;

		/*--------- ROS Communication Containers ------------- */
			ros::NodeHandle n;
			tf::TransformListener listener;

			// April Tag detections
			std::string s_tagDetections;
			ros::Subscriber TagDetections_sub;

			//Slam On/Off switch service
			std::string s_slam_onoff_switch_ser;
			ros::ServiceServer slam_onoff_switch_ser;
			bool slamSwitch;

			// Shutdown listener
			ros::Subscriber shutdown_sub;
			std::string s_shutdown_topic;

			// Uav inertial measurement
			ros::Subscriber navData_sub;
			std::string s_navData_topic;

			struct slamData
	    {
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
				std::vector<double> augmentedMeasurementVector; // current measurements of landmarks
				std::vector<double> predictedMeasurementVector; // predicted measurements of landmarks
				cv::Mat augmentedCovariance;

				cv::Mat uavR, uavt, uavHgl2lo;
				double uavCos, uavSin;

				std::vector<int> firstTimeForID;
				std::vector<int> updateForID;
				std::vector<uint> TagsInFOV;
				uint NumberOfTagsinFOV, registered, slamCount;    
				int downimagecountup; // number of images to ignore before starting slam
			};

			struct uavData
			{
				cv::Mat EstPosition_gl, EstPositionHom_gl, EstPosition_uav;
				cv::Mat odom_gl;
				double EstPhi_gl, cosEstPhi_gl, sinEstPhi_gl;
				cv::Mat MeasuredVel_lo;
				cv::Mat Rgl2lo, Hgl2lo, Hlo2gl;

				double deltaPhi, odomPhi;
				double compassYaw, compassYawLast;
				double cos_compassYaw, cos_compassYawLast;
				double sin_compassYaw, sin_compassYawLast;
				
				cv::Mat Qdk, Qw, Fk;

				cv::Mat RdeltaPhi_gl2lo;
				double cosRdeltaPhi, sinRdeltaPhi;

				// tf broadcaster variables
				tf::Transform TF;
				tf::Matrix3x3 R_TF;
				tf::Quaternion Q_TF;
			};

	public:

		/*---------  Landmark Definitions ------------- */
			std::vector<apriltags_ros::AprilTagDetection> tagArray; //marker array as detected by image
				uint tagArraySize; // number of tags in array
				ros::Time tagStamp;
				double tagTSpublished;

			std::vector<apriltagclass> april;  // class for storing individual tag information
				int tag_max_id, tag_counter;
				std::string s_markerid;
				int printlength;
				char markerChannel[20];
				cv::Vec<double,3> markerObs;

		/*---------  Navdata/uav variables ------------- */
			uint navDataCounter, flightState, navHeadSeq;
			ros::Time navStamp;
			double navTS, navTSpublished, navdt;
			double echoAlt, echoAltLast, deltaAlt;
			
			// tf variables
			tf::TransformBroadcaster uav_TF_pub;
			std::string s_ref_TF_frame, s_uav_TF_frame;

		/*--------- Class variables ------------- */
			fileops mfile;
			slamData slam;
			uavData uav;

			double augCovScale, uavQdkScale, uavQwScale, RkScale;

			ros::Publisher slamState_pub;
				hast::flag slam_state;




	// Class Functions
		uavSLAM()
		{
			// File params
				if(ros::param::get("~user",s_user)){}   else {s_user = "";}  //ROS_INFO("uavSLAM:: user [%s]", s_user.c_str());
				if(ros::param::get("~date",s_date)){}   else {s_date = "";}  //ROS_INFO("uavSLAM:: date [%s]", s_date.c_str());
				if(ros::param::get("~trial",s_trial)){} else {s_trial = "";} //ROS_INFO("uavSLAM:: trial [%s]", s_trial.c_str());
				ros::Duration(0.5).sleep();

				mfile.init_fileops("/home/" + s_user + "/ros/data/" + s_date + "/" + s_trial + "/uavSLAM_"  + s_trial + ".m");

			/*---------  Config ROS ------------- */
				slamState_pub 	= n.advertise<hast::flag>("/hast/slam_state", 10);
				slam_state.flag = false;
				slamState_pub.publish(slam_state);

			// Shutdown listener
				if(ros::param::get("~shutdown_topic",s_shutdown_topic)){} else {s_shutdown_topic = "/hast/shutdown";}
				shutdown_sub 	= n.subscribe(s_shutdown_topic,	1, &uavSLAM::nodeShutDown, this);
				mfile.writeString("%% shutdown_topic: " + s_shutdown_topic + "\n" );

			// Subscribe to tag detections
				if(ros::param::get("~tag_detection_topic",s_tagDetections)){} else {s_tagDetections = "tag_detections";}
				TagDetections_sub 	= n.subscribe(s_tagDetections, 1, &uavSLAM::tagDetections , this);
				mfile.writeString("%% tag_detection_topic: " + s_tagDetections + "\n" );

			// Subscribe to uav navdata
				if(ros::param::get("~uav_navdata_topic",s_navData_topic)){} else {s_navData_topic = "/ardrone/navdata";}
				navData_sub 	= n.subscribe(s_navData_topic, 1, &uavSLAM::inertialUpdate , this);
				mfile.writeString("%% uav_navdata_topic: " + s_navData_topic + "\n" );

			// Switch to turn slam on/off
				if(ros::param::get("~uav_slam_switch_topic",s_slam_onoff_switch_ser)){} else {s_slam_onoff_switch_ser = "/hast/uav/slam_switch";}
				slam_onoff_switch_ser    = n.advertiseService(s_slam_onoff_switch_ser, &uavSLAM::slamOnOffSwitch, this);
					mfile.writeString("%% uav_slam_switch_topic: " + s_slam_onoff_switch_ser + "\n" );
					slamSwitch = false;

				if(ros::param::get("~augCovScale",augCovScale)){}   else {augCovScale = 0.001;}  
				if(ros::param::get("~uavQdkScale",uavQdkScale)){}   else {uavQdkScale = 0.001;}  
				if(ros::param::get("~uavQwScale",uavQwScale)){}   else {uavQwScale = 0.001;}  
				if(ros::param::get("~RkScale",RkScale)){}   else {RkScale = 0.001;}  

			/*---------  Init UAV ------------- */
				uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.MeasuredVel_lo = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.odom_gl = (cv::Mat_<double>(3, 1) << 0,0,0);
				uav.odomPhi = 0;
				// uav.EstPositionHom_gl = (cv::Mat_<double>(4, 1) << 0,0,0,1);
				uav.EstPhi_gl = 0;
				uav.compassYaw = 0; uav.compassYawLast = 0; uav.deltaPhi = 0;
				uav.cos_compassYaw = 0; uav.cos_compassYawLast = 0;
				uav.sin_compassYaw = 0; uav.sin_compassYawLast = 0; 
					uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
					uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
					uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
					uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
					uav.Hlo2gl = invertH(uav.Hgl2lo);

				navTS = 0; navdt = 10000; navTSpublished = 0; 
				echoAlt = 0; echoAltLast = 0; deltaAlt = 0;
				navDataCounter = 0;

				uav.Qw  = uavQwScale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				uav.Fk  = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				uav.Qdk = uavQdkScale * (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);


				// frames for publishing TF to Rviz
				if(ros::param::get("~uav_TF_frame",s_uav_TF_frame)){} else {s_uav_TF_frame = "/hast/uav/base_frame";}
				if(ros::param::get("~ref_TF_frame",s_ref_TF_frame)){} else {s_ref_TF_frame = "/map";}
				mfile.writeString("%% uav_TF_frame: " + s_uav_TF_frame + "\n" );
				mfile.writeString("%% ref_TF_frame: " + s_ref_TF_frame + "\n" );

			/*---------  Init slam ------------- */

			// create vector of landmarks
				if(ros::param::get("~max_number_of_tags",tag_max_id)){}   else {tag_max_id = 15;}	
				mfile.writeString("%% max_number_of_tags: " + patch::to_string(tag_max_id) + "\n" );			
				april.reserve( tag_max_id );
				for(uint i = 0; i != tag_max_id; i++) //init april tags
				{
					apriltagclass obj;
					obj.id = i;
					if (i==7){obj.isgoal = true;} else {obj.isgoal=false;} // define tag #7 as the goal landmark
					obj.Rk_uav =  RkScale * (cv::Mat_<double>(4, 4) << 
															1,  0,  0, 0,
															0,  1,  0, 0,
															0,  0, 1, 0,
															0,  0, 0, 1); // measurement covariance of landmarks
					april.push_back(obj);
				}
				tag_counter = 0;

				slam.firstTimeForID.assign(tag_max_id, 0);
				slam.updateForID.assign(tag_max_id, 0);
				slam.TagsInFOV.assign(tag_max_id, 0);

				slam.slamCount = 1;
				slam.downimagecountup = 0; // number of images to ignore before starting slam
				slam.NumberOfTagsinFOV = 0;
				slam.numberOfLandmarksOrdered = 0;
				slam.I4 = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				slam.nI4 = -1*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
				// the augmented covariance will only grow from this size
				
				slam.augmentedCovariance = augCovScale*(cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

				slam.augmentedState.push_back(0); //ugv x
				slam.augmentedState.push_back(0); //ugv y
				slam.augmentedState.push_back(0); //ugv z
				slam.augmentedState.push_back(0); //ugv yaw

			mfile.writeString("%% slam.augmentedState at init \n" );
			mfile.cellVecDoubles(slam.augmentedState, "uavSLAM.augmentedState_k", slam.slamCount);
			mfile.writeString("%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" );
		}

		void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags) 
		{
			// run update every time there is a new tag detection:
			
			tagArray = tags->detections;
			tagArraySize = tagArray.size();
			slam.NumberOfTagsinFOV = 0;
			slam.TagsInFOV.assign(tag_max_id, 0);

			if(slamSwitch)
			{
				if (tagArray.size()>0)
				{
					tag_counter++;
					fprintf(mfile.filename, "%% --------------- tagDetections --------------- %% \n");
					fprintf(mfile.filename,"uavSLAM.april.tag_time(%u,:) = %6.4f;\n", tag_counter, ros::Time::now().toSec());
					fprintf(mfile.filename,"uavSLAM.april.tagArraySize(%u,:) = %i;\n", tag_counter, tagArraySize);

				}//end if (tagArray.size()>0)


				int id;
				for(int k = 0; k != tagArraySize; k++) 
				{
					// set marker IDs
					id = tagArray[k].id;
					printlength = sprintf(markerChannel, "/april_marker_%u", id);
					s_markerid = "tag_" + patch::to_string(id);

					// marker time stamp
					tagStamp = tagArray[k].pose.header.stamp;
					tagTSpublished = tagStamp.toSec();

					// Camera frame measurement of tags
					april[id].MeasPosition_cam = (cv::Mat_<double>(4, 1) << tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z, 1);

					// Games using TF
					double qx,qy,qz,qw, qroll, qpitch;
					tf::StampedTransform StampedXform_uav2cam, StampedXform_april2map;

					tf::Quaternion q_cam2tag(tagArray[k].pose.pose.orientation.x,  tagArray[k].pose.pose.orientation.y, tagArray[k].pose.pose.orientation.z, tagArray[k].pose.pose.orientation.w);
					tf::Vector3 t_cam2tag(tagArray[k].pose.pose.position.x, tagArray[k].pose.pose.position.y, tagArray[k].pose.pose.position.z);
					tf::Transform Xform_cam2tag(q_cam2tag, t_cam2tag);
					tf::Matrix3x3(Xform_cam2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_cam_rad);
					// april[id].MeasYaw_cam = toDegrees(april[id].MeasYaw_cam_rad);
					april[id].MeasYaw_cam = april[id].MeasYaw_cam_rad; //stay in radians where possible

					try {listener.lookupTransform("/hast/uav/ardrone_base_link", "/hast/uav/base_bottomcam", ros::Time(0), StampedXform_uav2cam);}
						catch (tf::TransformException ex) {ROS_INFO("ckfRecorder: /hast/uav/base_bottomcam is missing");} 

					tf::Quaternion q_uav2cam(StampedXform_uav2cam.getRotation().x(), StampedXform_uav2cam.getRotation().y(), StampedXform_uav2cam.getRotation().z(), StampedXform_uav2cam.getRotation().w()); 
					tf::Vector3 t_uav2cam(StampedXform_uav2cam.getOrigin().x(), StampedXform_uav2cam.getOrigin().y(), StampedXform_uav2cam.getOrigin().z());
					tf::Transform Xform_uav2cam(q_uav2cam, t_uav2cam);
					tf::Transform Xform_uav2tag = Xform_uav2cam*Xform_cam2tag;

					qx = Xform_uav2tag.getRotation().x(); 
					qy = Xform_uav2tag.getRotation().y(); 
					qz = Xform_uav2tag.getRotation().z(); 
					qw = Xform_uav2tag.getRotation().w(); 
					tf::Matrix3x3(Xform_uav2tag.getRotation()).getRPY(qroll, qpitch, april[id].MeasYaw_uav_rad);

					april[id].measCount++;
					april[id].MeasPosition_uav = (cv::Mat_<double>(4, 1) << Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z(), 1);
					// april[id].MeasYaw_uav = toDegrees(april[id].MeasYaw_uav_rad);
					april[id].MeasYaw_uav = april[id].MeasYaw_uav_rad; //stay in radians where possible
					// fprintf(mfile.filename,"uavSLAM.april.%s.MeasPosition_uav(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[id].measCount, Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z());
					// fprintf(mfile.filename,"uavSLAM.april.%s.MeasYaw_uav(%u,:) = %6.4f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_uav);

					// april[id].MeasPosition_gl = uav.Hlo2gl*(cv::Mat_<double>(4, 1) << Xform_uav2tag.getOrigin().x(), Xform_uav2tag.getOrigin().y(), Xform_uav2tag.getOrigin().z(), 1);
					// april[id].MeasYaw_gl = wrapDegrees(april[id].MeasYaw_uav + uav.EstPhi_gl);
					// fprintf(mfile.filename,"uavSLAM.april.%s.MeasPosition_gl(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[id].measCount, april[id].MeasPosition_gl.at<double>(0,0), april[id].MeasPosition_gl.at<double>(1,0), april[id].MeasPosition_gl.at<double>(2,0));
					// fprintf(mfile.filename,"uavSLAM.april.%s.MeasYaw_gl(%u,:) = %6.4f;\n", s_markerid.c_str(), april[id].measCount, april[id].MeasYaw_gl);

					slam.TagsInFOV[slam.NumberOfTagsinFOV] = april[id].id;
					//update the number count of tags in FOV
					slam.NumberOfTagsinFOV++;
				} // end for(int k = 0; k != tagArraySize; k++) 
		
				// ROS_INFO("Number of tags in FOV %u", slam.NumberOfTagsinFOV);
				if (slam.NumberOfTagsinFOV > 0)
				{ // only run slam if there is a new measurement of tags & the UAVv has been observed at least once
					MarkerSLAM(tagTSpublished);
					// update the map every time there is at least one tag in the FOV
					// publishAprilObstacles();
				} //end if (slam.NumberOfTagsinFOV > 0)

			} //end if(slamSwitch)
		} // end void tagDetections(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tags) 

		void MarkerSLAM(double scantime)
		{		//reset vectors
			slam.firstTimeForID.assign(tag_max_id, 0);
			slam.updateForID.assign(tag_max_id, 0);
			int updatingTags = 0; // number of tags that are updating measurements

			std::vector<int> updatingIDs;
			int newTags = 0; // number of first time observations
			uint ID; // id of april tag under consideration
			// ROS_INFO("Number of tags in FOV %u", slam.NumberOfTagsinFOV);
			// First, check which markers are being observed for the first time and which are updating measurements
			for(uint i = 0; i != slam.NumberOfTagsinFOV; i++) 
			{
				ID = april[slam.TagsInFOV[i]].id;
				// ROS_INFO("tag %i in FOV: %u", i, ID);
				// ROS_INFO("tag ID %u measCount: %u", ID, april[slam.TagsInFOV[i]].measCount);
				// ROS_INFO("MarkerSLAM::april[slam.TagsInFOV[%i]].measCount = %u", slam.TagsInFOV[i], april[slam.TagsInFOV[i]].measCount);
				if (april[slam.TagsInFOV[i]].measCount == (1+slam.downimagecountup)) // first, check to see if this is the first estimate of the marker:
				{
					// ROS_INFO("MarkerSLAM::slam.firstTimeForID[%i] = %i;", newTags, ID);
					slam.firstTimeForID[newTags]=ID;
					newTags++;
					// ROS_INFO("MarkerSLAM::newTags [%u]", newTags);
				} else {
					if (april[slam.TagsInFOV[i]].measCount < (1+slam.downimagecountup))
					{
						// Do nothing
					} else { // measuremnt >= (1+slam.downimagecountup), start updating
						slam.updateForID[updatingTags] = ID;
						updatingIDs.push_back(ID);
						updatingTags++;
						// ROS_INFO("MarkerSLAM::updatingTags [%u]", updatingTags);
					}
				}
			} // end for(uint i = 0; i != slam.NumberOfTagsinFOV; i++) 

			mfile.cellVecUints(slam.TagsInFOV,"uavSLAM.april.TagsInFOV", slam.slamCount);
			mfile.cellVecInts(slam.firstTimeForID,"uavSLAM.april.firstTimeForID", slam.slamCount);
			mfile.cellVecInts(slam.updateForID,"uavSLAM.april.updateForID", slam.slamCount);
			mfile.cellVecInts(slam.landmarksOrdered,"uavSLAM.april.landmarksOrdered", slam.slamCount);

			// ROS_INFO("Number of tags first time %u", newTags);
			// ROS_INFO("Number of tags updating   %u", updatingTags);

			/* ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~ ~~~~~~~~~*/ 
			// now that I know how many markers are new and which are repeat measurements, ...
			// I can update the measurement matrices, but only if there are any second+ measurements (otherwise skip this section)
			if (updatingTags>0) //&& (1)) // 1 vs 0 just lets be turn on and off this block
			{// now that we know there is an update for at least one april tag:
				fprintf(mfile.filename, "%% --------------- updatingTags --------------- %% \n");
				slam.slamCount++;
				fprintf(mfile.filename,"uavSLAM.time(%u,:) = % -6.14f;\n", slam.slamCount, scantime);
				fprintf(mfile.filename,"uavSLAM.newTags(%u,:) = %i;\n", slam.slamCount, newTags);
				fprintf(mfile.filename,"uavSLAM.updatingTags(%u,:) = %i;\n", slam.slamCount, updatingTags);
				
				slam.uavCos = cos(slam.augmentedState[3]);
				slam.uavSin = sin(slam.augmentedState[3]);
				slam.uavR  = (cv::Mat_<double>(3, 3) <<
						    slam.uavCos, slam.uavSin, 0,
						   -slam.uavSin, slam.uavCos, 0,
						    0, 0, 1);
				slam.uavt  = (cv::Mat_<double>(3, 1) << slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2]);
				slam.uavHgl2lo = wrapH(slam.uavR, slam.uavt);
				mfile.writeCvMatDoubles(slam.uavHgl2lo, "uavSLAM.slam.uavHgl2lo", slam.slamCount);

				// Both of these should be zero from initial augmentation, or should be zeroed at the end of an iteration
				// slam.augmentedMeasurementVector == 0
				// slam.predictedMeasurementVector == 0

				// ROS_INFO("MarkerSLAM::updatingTags if (%i>0) ", updatingTags);
				cv::Mat SLAM_H; // measurement matrix based on observed april tags
				// H should only have j*4 rows (j = number of tags observed, each with four states)
				// H must have as many columns as 4*(total number of markers in dictionary) plus 4 for the UAV
				// I think H only needs as many columns as previously observed tags...
				// SLAM_H = cv::Mat::zeros(updatingTags*4, (1+slam.numberOfLandmarksOrdered)*4, CV_64F);  //1+ accounts for UAv
				SLAM_H = cv::Mat::zeros(slam.numberOfLandmarksOrdered*4, (1+slam.numberOfLandmarksOrdered)*4, CV_64F);  //1+ accounts for UAv

				//slam.numberOfLandmarksOrdered does not need to be decremented because of the UAV states
				cv::Mat SLAM_Rk; // measurement covariance matrix for each marker
				SLAM_Rk = cv::Mat::zeros(slam.numberOfLandmarksOrdered*4, slam.numberOfLandmarksOrdered*4, CV_64F);
				// ROS_INFO("SLAM_Rk = cv::Mat::zeros(updatingTags*4, updatingTags*4, CV_64F);");
				// ROS_INFO("SLAM_Rk = cv::Mat::zeros(%i, %i, CV_64F);", updatingTags*4, updatingTags*4);
					// Rk_slam = [
					//     Rk_uav Z;...
					//     Z Rk_uav]

				//Update covariance of UAV from motion:
				cv::Mat aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4);  // uav current covariance
				// mfile.writeCvMatDoubles(aux_C, "uavSLAM.aux_C_0", slam.slamCount);
				// mfile.cellCvMatDoubles(slam.augmentedCovariance, "uavSLAM.augmentedCovariance_0", slam.slamCount);

				cv::Mat uav_C = uav.Qdk;
				// mfile.writeCvMatDoubles(uav_C, "uavSLAM.uav.Qdk", slam.slamCount);
				aux_C += uav_C;  //this should add uav.Qdk to slam.augmentedCovariance.colRange(0,4).rowRange(0,4)
				aux_C.release();
				uav_C.release();

				// Now that H and Rk are defined, we can populate them
				// ROS_INFO("for(uint i = 0; i != updatingTags; i++)");
				fprintf(mfile.filename,"%%slam.uav.EstPhi_gl = %6.4f;\n", uav.EstPhi_gl);
				for(uint i = 0; i != updatingTags; i++)
				{
					// ROS_INFO("i = %i", i);
					// ROS_INFO("uint ID = april[slam.updateForID[%i]].id = %u;", i, ID);
					// ROS_INFO("uint Ord = april[slam.updateForID[%i]].order = %u;", i, Ord);
					uint ID = april[slam.updateForID[i]].id; // the numeric ID of the observed tag
					uint Ord = april[slam.updateForID[i]].order; // serial order of observation (zero indexed?)

					// the first four columns of H are always block diagonals of I:
					cv::Mat aux_H_UAV = SLAM_H.colRange(0,4).rowRange((Ord-1)*4,(Ord)*4);
					slam.I4.copyTo(aux_H_UAV); // this should write data to SLAM_H				
					aux_H_UAV.release();//release pointer

					// the negative eye(4) needs to be shifted to the right by the eye(4) of the UAV plus the correct position in the dictionary
					cv::Mat aux_H_TAG = SLAM_H.colRange(4*Ord,4*(Ord+1)).rowRange(4*(Ord-1), Ord*4); 
					slam.nI4.copyTo(aux_H_TAG); // this should write data to SLAM_H
					aux_H_TAG.release(); //release pointer

					// Rk is constant for all tag measurements, but must be rotated to the global/map frame eventually
					cv::Mat aux_Rk = SLAM_Rk.colRange((i)*4,(i+1)*4).rowRange((i)*4,(i+1)*4);
					
					// this should copy Rk to the correct sub-block
					april[slam.updateForID[i]].Rk_uav.copyTo(aux_Rk); 
						// mfile.writeCvMatDoubles(aux_Rk, "uavSLAM.tag_" + patch::to_string(ID) + ".Rk", slam.slamCount);
						aux_Rk.release();

					slam.augmentedMeasurementVector[4*(Ord-1)+0] = april[ID].MeasPosition_uav.at<double>(0,0);
					slam.augmentedMeasurementVector[4*(Ord-1)+1] = april[ID].MeasPosition_uav.at<double>(1,0);
					slam.augmentedMeasurementVector[4*(Ord-1)+2] = april[ID].MeasPosition_uav.at<double>(2,0);
					slam.augmentedMeasurementVector[4*(Ord-1)+3] = april[ID].MeasYaw_uav; // in randians
					// fprintf(mfile.filename,"%%slam.augmentedMeasurementVector[%i:%i] = april[%i].[%6.4f %6.4f %6.4f %6.4f]\n", 4*(Ord-1)+0, 4*(Ord-1)+3, ID, slam.augmentedMeasurementVector[4*(Ord-1)+0], slam.augmentedMeasurementVector[4*(Ord-1)+1], slam.augmentedMeasurementVector[4*(Ord-1)+2], slam.augmentedMeasurementVector[4*(Ord-1)+3]);

					// need to convert previous estimated global location of april tag to the uav frame 

					april[ID].PredictedMeasurement_uav = slam.uavHgl2lo * april[ID].EstPosition_gl;
					april[ID].PredictedPhi = april[ID].EstYaw_gl - slam.augmentedState[3];

					slam.predictedMeasurementVector[4*(Ord-1)+0] = april[ID].PredictedMeasurement_uav.at<double>(0,0);
					slam.predictedMeasurementVector[4*(Ord-1)+1] = april[ID].PredictedMeasurement_uav.at<double>(1,0);
					slam.predictedMeasurementVector[4*(Ord-1)+2] = april[ID].PredictedMeasurement_uav.at<double>(2,0);
					slam.predictedMeasurementVector[4*(Ord-1)+3] = april[ID].PredictedPhi;
					// fprintf(mfile.filename,"%%slam.predictedMeasurementVector[%i:%i] = april[%i].[%6.4f %6.4f %6.4f %6.4f]\n", 4*(Ord-1)+0, 4*(Ord-1)+3, ID, slam.predictedMeasurementVector[4*(Ord-1)+0], slam.predictedMeasurementVector[4*(Ord-1)+1], slam.predictedMeasurementVector[4*(Ord-1)+2], slam.predictedMeasurementVector[4*(Ord-1)+3]);
					april[ID].vk = -1*(cv::Mat_<double>(4, 1) << 
						slam.augmentedMeasurementVector[4*(Ord-1)+0] - slam.predictedMeasurementVector[4*(Ord-1)+0], 
						slam.augmentedMeasurementVector[4*(Ord-1)+1] - slam.predictedMeasurementVector[4*(Ord-1)+1], 
						slam.augmentedMeasurementVector[4*(Ord-1)+2] - slam.predictedMeasurementVector[4*(Ord-1)+2], 
						slam.augmentedMeasurementVector[4*(Ord-1)+3] - slam.predictedMeasurementVector[4*(Ord-1)+3]);


					s_markerid = "tag_" + patch::to_string(ID);
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.time(%u,:) = % -6.14f;\n",  s_markerid.c_str(), ++april[ID].slamCount, scantime);
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.MeasPosition_uav(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, 
																										april[ID].MeasPosition_uav.at<double>(0,0), 
																										april[ID].MeasPosition_uav.at<double>(1,0), 
																										april[ID].MeasPosition_uav.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.EstPosition_gl(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, 
																										april[ID].EstPosition_gl.at<double>(0,0), 
																										april[ID].EstPosition_gl.at<double>(1,0), 
																										april[ID].EstPosition_gl.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.PredictedMeasurement_uav(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, 
																										april[ID].PredictedMeasurement_uav.at<double>(0,0), 
																										april[ID].PredictedMeasurement_uav.at<double>(1,0), 
																										april[ID].PredictedMeasurement_uav.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.vk(%u,:) = [%6.4f, %6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, 
																										april[ID].vk.at<double>(0,0), 
																										april[ID].vk.at<double>(1,0), 
																										april[ID].vk.at<double>(2,0), 
																										april[ID].vk.at<double>(3,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.MeasYaw_uav(%u,:) = %6.4f;\n", s_markerid.c_str(), april[ID].slamCount, april[ID].MeasYaw_uav);
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.PredictedPhi(%u,:) = %6.4f;\n", s_markerid.c_str(), april[ID].slamCount, april[ID].PredictedPhi);
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.EstPhi_gl(%u,1) = %6.4f;\n", s_markerid.c_str(), april[ID].slamCount, april[ID].EstYaw_gl);
					mfile.writeCvMatDoubles(slam.uavHgl2lo, "uavSLAM.april." + s_markerid + ".slam.Hgl2lo", april[ID].slamCount);



				}  // end for(uint i = 0; i != updatingTags; i++)
				// mfile.cellVecDoubles(slam.augmentedMeasurementVector, "uavSLAM.augmentedMeasurementVector", slam.slamCount);
				// mfile.cellVecDoubles(slam.predictedMeasurementVector, "uavSLAM.predictedMeasurementVector", slam.slamCount);

				//now Rk and H are defined, and P was grown automatically when the markers were measured the first time (see below)
				// increment slam estimate of re-observed markers and UAV
				// This uses the WHOLE augmented covariance because the unobserved rows of SLAM_H are all zeros
				// mfile.cellCvMatDoubles_multline(SLAM_H, "uavSLAM.H", slam.slamCount);

				// fprintf(mfile.filename,"%% slam.HPHt = SLAM_H * slam.augmentedCovariance * SLAM_H.t(); [%i x %i]*[%i x %i]*[%i x %i]\n", SLAM_H.rows, SLAM_H.cols, slam.augmentedCovariance.rows, slam.augmentedCovariance.cols, SLAM_H.cols, SLAM_H.rows);

				slam.HPHt = SLAM_H * slam.augmentedCovariance * SLAM_H.t();
				// mfile.cellCvMatDoubles_multline(slam.HPHt, "uavSLAM.HPHt", slam.slamCount);
				// mfile.cellCvMatDoubles_multline(SLAM_Rk, "uavSLAM.Rk", slam.slamCount);
				// fprintf(mfile.filename,"%% slam.Sk = slam.HPHt + SLAM_Rk; [%i x %i] + [%i x %i]\n", slam.HPHt.rows, slam.HPHt.cols, SLAM_Rk.rows, SLAM_Rk.cols);
				slam.Sk = slam.HPHt + SLAM_Rk;
				// mfile.cellCvMatDoubles_multline(slam.Sk, "uavSLAM.Sk", slam.slamCount);
				// mfile.cellCvMatDoubles_multline(slam.Sk.inv(), "uavSLAM.Sk_inv", slam.slamCount);
				// fprintf(mfile.filename,"%% slam.PHt = slam.augmentedCovariance*SLAM_H.t(); [%i x %i] * [%i x %i]\n", slam.augmentedCovariance.rows, slam.augmentedCovariance.cols, SLAM_H.cols, SLAM_H.rows);
				slam.PHt = slam.augmentedCovariance*SLAM_H.t(); // PHtr = P_slam*H' 
				// mfile.cellCvMatDoubles_multline(slam.PHt, "uavSLAM.PHt", slam.slamCount);
				// fprintf(mfile.filename,"%% slam.Kk = slam.PHt * slam.Sk.inv(); [%i x %i] * [%i x %i]\n", slam.PHt.rows, slam.PHt.cols, slam.Sk.rows, slam.Sk.cols);
				slam.Kk = slam.PHt * slam.Sk.inv(); // Kalman Gain, // K = PHtr * Sk
				// mfile.cellCvMatDoubles_multline(slam.Kk, "uavSLAM.Kk", slam.slamCount);
				// fprintf(mfile.filename,"%% slam.vk = cv::Mat(slam.augmentedMeasurementVector) - cv::Mat(slam.predictedMeasurementVector); [%i x 1] - [%i x 1]\n", int(slam.augmentedMeasurementVector.size()), int(slam.predictedMeasurementVector.size()));
				// slam.vk = cv::Mat(slam.augmentedMeasurementVector) - cv::Mat(slam.predictedMeasurementVector);
				slam.vk = cv::Mat(slam.predictedMeasurementVector) - cv::Mat(slam.augmentedMeasurementVector);
				mfile.cellCvMatDoubles_multline(slam.vk, "uavSLAM.vk", slam.slamCount);
				fprintf(mfile.filename,"%% slam.correction = slam.Kk * slam.vk; [%i x %i] * [%i x %i]\n", slam.Kk.rows, slam.Kk.cols, slam.vk.rows, slam.vk.cols);
				slam.correction = slam.Kk * slam.vk;
				mfile.cellCvMatDoubles_multline(slam.correction, "uavSLAM.correction", slam.slamCount);


				// last thing to do, update the covariance matrix
				// Ck = (I-K H) Ck
				// fprintf(mfile.filename,"%% cv::Mat KH = slam.Kk * SLAM_H; [%i x %i] * [%i x %i]\n", slam.Kk.rows, slam.Kk.cols, SLAM_H.rows, SLAM_H.cols);
				cv::Mat KH = slam.Kk * SLAM_H;
				int KHrows = KH.rows;
				int KHcols = KH.cols;
				// ROS_INFO("KH [rows x cols] = [%i %i]", KHrows, KHcols);

				int Crows = slam.augmentedCovariance.rows;
				int Ccols = slam.augmentedCovariance.cols;
				// ROS_INFO("uavSLAM.Ck [rows x cols] = [%i %i]", Crows, Ccols);

				cv::Mat KHI = cv::Mat::eye(KH.rows, KH.cols, CV_64F); //64f == double
				// fprintf(mfile.filename,"%% slam.augmentedCovariance = (KHI - KH) * slam.augmentedCovariance; ([%i x %i]-[%i x %i]) * [%i x %i]\n", KHI.rows, KHI.cols, KH.rows, KH.cols, slam.augmentedCovariance.rows, slam.augmentedCovariance.cols);
				slam.augmentedCovariance = (KHI - KH) * slam.augmentedCovariance;


				// update uav pose
					//update uav heading
					uav.EstPhi_gl += slam.correction.at<double>(3,0);
					uav.EstPhi_gl = wrapRadians(uav.EstPhi_gl);

					double uavcos  = cos(uav.EstPhi_gl);
					double uavsin  = sin(uav.EstPhi_gl);
					cv::Mat correction_Rgl2lo = (cv::Mat_<double>(3, 3) <<
							    uavcos, uavsin, 0,
							   -uavsin, uavcos, 0,
							    0, 0, 1);

					cv::Mat uav_correction = (cv::Mat_<double>(3, 1) <<
						slam.correction.at<double>(0,0),
						slam.correction.at<double>(1,0),
						slam.correction.at<double>(2,0));

					cv::Mat uav_correction_gl = correction_Rgl2lo.t() * uav_correction;

					uav.EstPosition_gl.at<double>(0,0) += uav_correction_gl.at<double>(0,0);
					uav.EstPosition_gl.at<double>(1,0) += uav_correction_gl.at<double>(1,0);
					uav.EstPosition_gl.at<double>(2,0) += uav_correction_gl.at<double>(2,0);

					slam.augmentedState[0] = uav.EstPosition_gl.at<double>(0,0);
					slam.augmentedState[1] = uav.EstPosition_gl.at<double>(1,0);
					slam.augmentedState[2] = uav.EstPosition_gl.at<double>(2,0);
					slam.augmentedState[3] = uav.EstPhi_gl;

					uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
					uav.Hlo2gl = invertH(uav.Hgl2lo);

				//update tag poses
				for(uint i = 0; i != updatingTags; i++)
				{
					uint ID = april[slam.updateForID[i]].id; // the numeric ID of the observed tag
					uint Ord = april[slam.updateForID[i]].order; // serial order of observation (zero indexed?)
					cv::Mat tag_correction = (cv::Mat_<double>(3, 1) <<
						slam.correction.at<double>(4*Ord+0,0),
						slam.correction.at<double>(4*Ord+1,0),
						slam.correction.at<double>(4*Ord+2,0));
	
					cv::Mat tag_correction_gl = correction_Rgl2lo.t() * tag_correction;
					
					april[ID].EstPosition_gl.at<double>(0,0) += tag_correction_gl.at<double>(0,0);
					april[ID].EstPosition_gl.at<double>(1,0) += tag_correction_gl.at<double>(1,0);
					april[ID].EstPosition_gl.at<double>(2,0) += tag_correction_gl.at<double>(2,0);
					april[ID].EstYaw_gl += slam.correction.at<double>(4*Ord+3,0);

					s_markerid = "tag_" + patch::to_string(ID);
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.correctionP_uav(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, tag_correction.at<double>(0,0), tag_correction.at<double>(1,0), tag_correction.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.correctionP_gl(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[ID].slamCount, tag_correction_gl.at<double>(0,0), tag_correction_gl.at<double>(1,0), tag_correction_gl.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.slam.correctionYaw(%u,:) = %6.4f;\n", s_markerid.c_str(), april[ID].slamCount, slam.correction.at<double>(4*Ord+3,0));
				}

				// int rows = slam.correction.rows;
				// for(uint rown = 0; rown != rows; rown++) 
				// {
				// 	slam.augmentedState[rown] += slam.correction.at<double>(rown,0);
				// }

				// uav.EstPosition_gl += slam.correction.at<double>(0,0);
				// uav.EstPosition_gl += slam.correction.at<double>(1,0);
				// uav.EstPosition_gl += slam.correction.at<double>(2,0);
				// uav.EstPhi_gl += slam.correction.at<double>(3,0);

				fprintf(mfile.filename,"uavSLAM.uav.est.p.global(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", slam.slamCount, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.uav.aug.p.global(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", slam.slamCount, slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2]);
				fprintf(mfile.filename,"uavSLAM.uav.est.p.correction(%u,:) = [% -6.14f, % -6.14f, % -6.14f];\n", slam.slamCount, uav_correction_gl.at<double>(0,0), uav_correction_gl.at<double>(1,0), uav_correction_gl.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.uav.aug.yaw.global(%u,:) = % -6.14f;\n", slam.slamCount, slam.augmentedState[3]);
				fprintf(mfile.filename,"uavSLAM.uav.est.yaw.global(%u,:) = % -6.14f;\n", slam.slamCount, uav.EstPhi_gl);
				fprintf(mfile.filename,"uavSLAM.uav.est.yaw.correction(%u,:) = % -6.14f;\n", slam.slamCount, slam.correction.at<double>(3,0));
				
				cv::Mat uav_Ck = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); 
					// mfile.writeCvMatDoubles(uav_Ck, "uavSLAM.uav.Ck", slam.slamCount);
					uav_Ck.release();
				// mfile.cellCvMatDoubles_multline(slam.augmentedCovariance, "uavSLAM.augmentedCovariance", slam.slamCount);
				mfile.cellVecDoubles(slam.augmentedState, "uavSLAM.augmentedState_k", slam.slamCount);

				fprintf(mfile.filename, "%% --------------- end KF increment --------------- %% \n");
				// end slam increment 
			} //end if (updatingTags>0)



			// If there are new tags, the whole system must be augmented:
			if (newTags>0)
			{
				if (slam.slamCount == 0)
				{
					// then set current uav pose as augmented state #1
					slam.augmentedState[0] = uav.EstPosition_gl.at<double>(0,0);
					slam.augmentedState[1] = uav.EstPosition_gl.at<double>(1,0);
					slam.augmentedState[2] = uav.EstPosition_gl.at<double>(2,0);
					slam.augmentedState[3] = uav.EstPhi_gl;
					cv::Mat aux_C = slam.augmentedCovariance.colRange(0,4).rowRange(0,4); 
					cv::Mat uav_C = uav.Qdk;
					uav_C.copyTo(aux_C); // this should copy Rk to the correct sub-block
					aux_C.release();
					uav_C.release();
				}

				cv::Mat newLandmarkPos, obsRk, newLandmarkMeas;
				double newLandmarkYaw;
				int newLandmarkID;
				// ROS_INFO("for(uint i = 0; i != newTags; i++) ");

				for(uint i = 0; i != newTags; i++) 
				{
					newLandmarkID = april[slam.firstTimeForID[i]].id;
					newLandmarkMeas = april[slam.firstTimeForID[i]].getMeasPosition_uav();
					newLandmarkPos = uav.Hlo2gl * april[slam.firstTimeForID[i]].getMeasPosition_uav();  // rotate to global 
					newLandmarkYaw = wrapRadians(april[slam.firstTimeForID[i]].MeasYaw_uav + uav.EstPhi_gl);
					obsRk = april[slam.firstTimeForID[i]].Rk_uav;
					augmentSLAM(newLandmarkID, newLandmarkPos, newLandmarkYaw, obsRk);
					april[slam.firstTimeForID[i]].order = slam.numberOfLandmarksOrdered; // should not start at zero
					april[slam.firstTimeForID[i]].EstPosition_gl = newLandmarkPos;
					april[slam.firstTimeForID[i]].EstYaw_gl = newLandmarkYaw;
					s_markerid = "tag_" + patch::to_string(newLandmarkID);
					fprintf(mfile.filename,"uavSLAM.april.%s.order = %i;\n", s_markerid.c_str(), slam.numberOfLandmarksOrdered);
					fprintf(mfile.filename,"uavSLAM.april.%s.init.EstPosition_gl(1,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), newLandmarkPos.at<double>(0,0), newLandmarkPos.at<double>(1,0), newLandmarkPos.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.init.EstYaw_gl(1,:) = %6.4f;\n", s_markerid.c_str(), newLandmarkYaw);
					fprintf(mfile.filename,"uavSLAM.april.%s.init.MeasPosition_uav(1,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), newLandmarkMeas.at<double>(0,0), newLandmarkMeas.at<double>(1,0), newLandmarkMeas.at<double>(2,0));
					fprintf(mfile.filename,"uavSLAM.april.%s.init.MeasYaw_uav(1,:) = %6.4f;\n", s_markerid.c_str(), april[slam.firstTimeForID[i]].MeasYaw_uav);
					mfile.writeCvMatDoubles(uav.Hlo2gl, "uavSLAM.april." + s_markerid + ".init.Hlo2gl", slam.slamCount);

				} // end for(uint i = 0; i != newTags; i++) 
			} //end if (newTags>0)

			mfile.writeString("%% slam.augmentedState at end of MarkerSLAM \n" );
			mfile.cellVecDoubles(slam.augmentedState, "uavSLAM.augmentedState_k", slam.slamCount);

			for(uint i = 0; i != slam.numberOfLandmarksOrdered; i++) 
			{
				s_markerid = "tag_" + patch::to_string(slam.landmarksOrdered[i]);
				fprintf(mfile.filename,"uavSLAM.april.%s.EstPosition_gl(%u,:) = [%6.4f, %6.4f, %6.4f];\n", s_markerid.c_str(), april[slam.landmarksOrdered[i]].measCount, slam.augmentedState[4*(i+1)+0], slam.augmentedState[4*(i+1)+1], slam.augmentedState[4*(i+1)+2]);
				fprintf(mfile.filename,"uavSLAM.april.%s.EstPhi_gl(%u,1) = %6.4f;\n", s_markerid.c_str(), april[slam.landmarksOrdered[i]].measCount, slam.augmentedState[4*(i+1)+3]);
				fprintf(mfile.filename,"uavSLAM.april.%s.EstTime(%u,1) = %6.4f;\n", s_markerid.c_str(), april[slam.landmarksOrdered[i]].measCount, scantime);
				
			}

			fprintf(mfile.filename, "%% --------------- slam loop --------------- %% \n");
		}

		void inertialUpdate(const ardrone_autonomy::Navdata::ConstPtr& navdata)
		{
			flightState = navdata->state;
			navHeadSeq = navdata->header.seq;
			navStamp = navdata->header.stamp;
			navTSpublished = navStamp.toSec();

			if (navTSpublished != navTS)
			{	/*----- Inertial Update */
				++navDataCounter;
				if ( navDataCounter % 10 == 5 )
				{ // publish slam state every tenth message
					slamState_pub.publish(slam_state);
				}

				// fprintf(mfile.filename,"uavSLAM.uav.EstPhi_gl_0(%u,:) = %6.14f;\n", navDataCounter, uav.EstPhi_gl);
				navdt = navTSpublished - navTS;
				navTS = navTSpublished;
				// suppress huge velocity estimates if motion starts from zero
				if (navdt>1000){
					navdt=0.006; // 0.006 is the mean dt of an early experiment
					uav.compassYaw = toRadians(navdata->rotZ); // this just prevents computing a potentially huge angular velocity
						uav.cos_compassYaw = cos((uav.compassYaw));
						uav.sin_compassYaw = sin((uav.compassYaw));
				} 

				uav.Qdk = uav.Qw*navdt;
				
				// update altitude measurements
				echoAltLast = echoAlt;
				echoAlt = 0.001 * double(navdata->altd);
				deltaAlt = echoAlt - echoAltLast;

				// Update heading measurements

				uav.compassYawLast = uav.compassYaw;
				uav.compassYaw = toRadians(navdata->rotZ);
					if ((uav.compassYaw - uav.compassYawLast) > 350) {uav.compassYawLast -= 360;}
					if ((uav.compassYaw - uav.compassYawLast) < -350) {uav.compassYawLast += 360;}

					uav.cos_compassYaw = cos((uav.compassYaw));
					uav.sin_compassYaw = sin((uav.compassYaw));

					uav.deltaPhi = uav.compassYaw - uav.compassYawLast;

				// Velocity update
				uav.MeasuredVel_lo = (cv::Mat_<double>(3, 1) <<
									0.001 * (navdata->vx), // not sure why these are backwards..
									0.001 * (navdata->vy),// not sure why these are backwards..
									deltaAlt/navdt); 

			// update estimated position by half of the delta angle
				uav.cosRdeltaPhi = cos(uav.EstPhi_gl + 0.5*uav.deltaPhi); 
				uav.sinRdeltaPhi = sin(uav.EstPhi_gl + 0.5*uav.deltaPhi);

				uav.RdeltaPhi_gl2lo = (cv::Mat_<double>(3, 3) <<
						   uav.cosRdeltaPhi, uav.sinRdeltaPhi, 0,
						  -uav.sinRdeltaPhi, uav.cosRdeltaPhi, 0,
						  0, 0, 1);

				cv::Mat gl_delta = uav.RdeltaPhi_gl2lo.t() * (navdt * uav.MeasuredVel_lo);
				uav.EstPosition_gl     += gl_delta;
				slam.augmentedState[0] += gl_delta.at<double>(0,0);
				slam.augmentedState[1] += gl_delta.at<double>(1,0);
				slam.augmentedState[2] += gl_delta.at<double>(2,0);
				slam.augmentedState[3] += uav.deltaPhi;

				uav.EstPhi_gl += uav.deltaPhi;

				double odomcos  = cos(uav.odomPhi + 0.5*uav.deltaPhi);
				double odomsin  = sin(uav.odomPhi + 0.5*uav.deltaPhi);
				cv::Mat odomRgl2lo = (cv::Mat_<double>(3, 3) <<
						   odomcos, odomsin, 0,
						   -odomsin, odomcos, 0,
						   0, 0, 1);

				uav.odom_gl += odomRgl2lo.t() * (navdt * uav.MeasuredVel_lo);
				uav.odomPhi += uav.deltaPhi;


				// fprintf(mfile.filename,"uavSLAM.uav.EstPhi_gl(%u,:) = %6.14f;\n", navDataCounter, uav.EstPhi_gl);
					uav.cosEstPhi_gl = cos((uav.EstPhi_gl));
					uav.sinEstPhi_gl = sin((uav.EstPhi_gl));
					uav.Rgl2lo = (cv::Mat_<double>(3, 3) <<
							   uav.cosEstPhi_gl, uav.sinEstPhi_gl, 0,
							  -uav.sinEstPhi_gl, uav.cosEstPhi_gl, 0,
							   0, 0, 1);
				uav.EstPosition_uav = uav.Rgl2lo * uav.EstPosition_gl;

				uav.Hgl2lo = wrapH(uav.Rgl2lo, uav.EstPosition_gl);
				uav.Hlo2gl = invertH(uav.Hgl2lo);

				tfPublisher(navTS);


				fprintf(mfile.filename,"\nuavSLAM.inertial.time(%u,1) = %6.4f;\n", navDataCounter, navTS);
				fprintf(mfile.filename,"uavSLAM.inertial.EstPosition_gl(%u,:) = [%6.4f, %6.4f, %6.4f];\n", navDataCounter, uav.EstPosition_gl.at<double>(0,0), uav.EstPosition_gl.at<double>(1,0), uav.EstPosition_gl.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.inertial.augmentedState(%u,:) = [%6.4f, %6.4f, %6.4f];\n", navDataCounter, slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2]);
				fprintf(mfile.filename,"uavSLAM.inertial.EstPhi_gl(%u,:) = %6.4f;\n", navDataCounter, uav.EstPhi_gl);
				fprintf(mfile.filename,"uavSLAM.inertial.augPhi(%u,:) = %6.4f;\n", navDataCounter, slam.augmentedState[3]);
				fprintf(mfile.filename,"uavSLAM.inertial.MeasuredVel_lo(%u,:) = [%6.4f, %6.4f, %6.4f];\n", navDataCounter, uav.MeasuredVel_lo.at<double>(0,0), uav.MeasuredVel_lo.at<double>(1,0), uav.MeasuredVel_lo.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.inertial.gl_delta(%u,:) = [%6.4f, %6.4f, %6.4f];\n", navDataCounter, gl_delta.at<double>(0,0), gl_delta.at<double>(1,0), gl_delta.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.inertial.deltaPhi(%u,:) = %6.4f;\n", navDataCounter, uav.deltaPhi);
				fprintf(mfile.filename,"uavSLAM.inertial.odom_gl(%u,:)  = [%6.4f, %6.4f, %6.4f];\n", navDataCounter, uav.odom_gl.at<double>(0,0), uav.odom_gl.at<double>(1,0), uav.odom_gl.at<double>(2,0));
				fprintf(mfile.filename,"uavSLAM.inertial.odomPhi(%u,:)  = %6.4f;\n", navDataCounter, uav.odomPhi);


			}
		}

		void augmentSLAM(int newLandmarkID, cv::Mat newLandmarkPos, double newLandmarkYaw, cv::Mat Rj)
		{
			// add new measurement of landmark to the augmented state (slam.est)
			slam.augmentedState.push_back(newLandmarkPos.at<double>(0,0)); //x
			slam.augmentedState.push_back(newLandmarkPos.at<double>(1,0)); //y
			slam.augmentedState.push_back(newLandmarkPos.at<double>(2,0)); //z
			slam.augmentedState.push_back(newLandmarkYaw); //yaw

			slam.landmarksOrdered.push_back(newLandmarkID); //this is useful for tracking the number of landmarks
			++slam.numberOfLandmarksOrdered; // this will be useful for determining size of augmented P matrix
			slam.timeOflandmarksOrdered.push_back(ros::Time::now().toSec());

			// zero pad measurement matrices to maintain vector sizes
			slam.augmentedMeasurementVector.push_back(0.0); // x
			slam.augmentedMeasurementVector.push_back(0.0); // y
			slam.augmentedMeasurementVector.push_back(0.0); // z
			slam.augmentedMeasurementVector.push_back(0.0); // yaw

			slam.predictedMeasurementVector.push_back(0.0); // x
			slam.predictedMeasurementVector.push_back(0.0); // y
			slam.predictedMeasurementVector.push_back(0.0); // z
			slam.predictedMeasurementVector.push_back(0.0); // yaw


			// 1) create new rows
				// submatrix are the top 4 rows of the current P matrix, vconcat and hconcat don't follow zero index ideas
				cv::Mat submatrix = slam.augmentedCovariance.colRange(0,4*slam.numberOfLandmarksOrdered).rowRange(0,4); 
				cv::Mat newrowsA, newcolsA;
				submatrix.copyTo(newrowsA);
				// add top 4 rows to bottom of augmented matrix
				cv::vconcat(slam.augmentedCovariance, newrowsA, slam.augmentedCovariance);
			// 2) define new columns, then add Cm3 to bottom of new columns
				newcolsA = newrowsA.t();
				cv::Mat CMn = slam.augmentedCovariance.colRange(0,4).rowRange(0,4)+100*Rj; 
				cv::vconcat(newcolsA, CMn, newcolsA);
			// 3) add new columns to right side of augmented matrix
				cv::hconcat(slam.augmentedCovariance, newcolsA, slam.augmentedCovariance);
		}

		void tfPublisher(ros::Time pubstamp)
		{//ROS_INFO("uav void tfPublisher(ros::Time pubstamp)");
			// ROS_INFO("uavRecorder: -------Pose_msg --------------------------");
			// ROS_INFO("uavRecorder: EstPosition_gl  = [%6.4f %6.4f %6.4f]  ", EstPosition_gl.at<double>(0,0), EstPosition_gl.at<double>(1,0), EstPosition_gl.at<double>(2,0));
			// ROS_INFO("uavRecorder: [00 01 02] = [%6.4f %6.4f %6.4f]  ",  cosyaw, sinyaw, 0.0);
			// ROS_INFO("uavRecorder: [10 11 12] = [%6.4f %6.4f %6.4f]  ", -sinyaw, cosyaw, 0.0);
			// ROS_INFO("uavRecorder: [20 21 22] = [%6.4f %6.4f %6.4f]  ",  0.0,0.0,1.0);

			
			if (slam_state.flag) 
			{ //publish when slam has been activated
				uav.R_TF.setValue(uav.cos_compassYaw, -uav.sin_compassYaw, 0,
													uav.sin_compassYaw,  uav.cos_compassYaw, 0,
													0,0,1);
				uav.R_TF.getRotation(uav.Q_TF);

				uav.TF.setRotation(uav.Q_TF);
				uav.TF.setOrigin( tf::Vector3(uav.EstPosition_gl.at<double>(0, 0), uav.EstPosition_gl.at<double>(1, 0), uav.EstPosition_gl.at<double>(2, 0)) );
				uav_TF_pub.sendTransform(tf::StampedTransform(uav.TF, pubstamp, s_ref_TF_frame, s_uav_TF_frame));
			}

			// ROS_WARN("oneCKF::tfPublisher");
			// ROS_INFO("  parent: %s, ", s_TF_gl_parent.c_str());
			// ROS_INFO("  child:  %s ", s_TF_gl_child.c_str());
			// ROS_INFO("  TF_gl.setOrigin : [%6.4f %6.4f %6.4f]", EstPosition_gl.at<double>(0, 0), EstPosition_gl.at<double>(1, 0), EstPosition_gl.at<double>(2, 0));
		}

		bool slamOnOffSwitch(hast::slamswitch::Request &req, hast::slamswitch::Response &res)
		{ // software control, switch between SLAM and External pose estimates for UAV
			if(req.flip)
			{// True -> UAV pose is estimated via slam
				slam_state.flag = true;
				slamState_pub.publish(slam_state);

				ROS_INFO("uavSLAM: Begin using images for slam");
				slamSwitch = true;
				uav.EstPosition_gl = (cv::Mat_<double>(3, 1) << req.pose.position.x,req.pose.position.y,req.pose.position.z);
				
				double qroll, qpitch, qyaw;
				tf::Vector3 t_uav_pose_gl(req.pose.position.x,req.pose.position.y,req.pose.position.z);
				tf::Quaternion q_uav_pose_gl(req.pose.orientation.x,req.pose.orientation.y,req.pose.orientation.z,req.pose.orientation.w);
				tf::Transform Xform_uav_pose_gl(q_uav_pose_gl, t_uav_pose_gl);
				tf::Matrix3x3(Xform_uav_pose_gl.getRotation()).getRPY(qroll, qpitch, qyaw);
				// uav.EstPhi_gl = toDegrees(qyaw);
				uav.EstPhi_gl = qyaw; //stay in radians where possible

				slam.augmentedState[0] = req.pose.position.x;
				slam.augmentedState[1] = req.pose.position.y;
				slam.augmentedState[2] = req.pose.position.z;
				slam.augmentedState[3] = uav.EstPhi_gl;

				fprintf(mfile.filename, "\n%% --------------- init slam from switch --------------- %% \n");
				fprintf(mfile.filename,"uavSLAM.slam_init.augmentedState = [%6.4f, %6.4f, %6.4f, %6.4f];\n", slam.augmentedState[0], slam.augmentedState[1], slam.augmentedState[2], slam.augmentedState[3]);
				fprintf(mfile.filename,"uavSLAM.slam_init.EstPosition_gl = [%6.4f, %6.4f, %6.4f];\n", req.pose.position.x,req.pose.position.y,req.pose.position.z);
				fprintf(mfile.filename,"uavSLAM.slam_init.EstPhi_gl = %6.4f;\n", uav.EstPhi_gl);
				fprintf(mfile.filename, "%% --------------- init slam from switch --------------- %% \n\n");
				res.flop = true;
			}
			if(!req.flip)
			{// False -> UAV pose is estimated externally
				ROS_INFO("uavSLAM: deactivate slam");
				slamSwitch = false;
				res.flop = false;
			}
			return true; //this needs to be true, otherwise the service doesn't send the response.
		}

		void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
		{
			// ROS_WARN("ckfRecorder: nodeShutDown");
			if (ShutDown->flag)
			{
				// aprilAlloc();
				// ugv.openAllocFile()
				ROS_INFO("uavSLAM: Shutdown requested..");
				// fprintf(mfile.filename,"uavSLAM=slam;\n");
				// fprintf(mfile.filename,"uavSLAM.april=april;\n");
				ros::Duration(1.5).sleep();
				ros::shutdown();
			}
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uavSLAM");

	uavSLAM uS;
	ros::spin();
	return 0;
}


