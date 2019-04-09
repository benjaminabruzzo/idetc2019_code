/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Marcus Liebhardt
 *
 * This work has been inspired by Nate Koenig's Gazebo plugin for the iRobot Create.
 */
#include <random> // for normal distribution
// #include <algorithm> 
// #include <assert.h>


#include <vicon_plugins/gazebo_ros_kobuki_vicon.h>

namespace gazebo {


void GazeboRosKobuki::updateJointState()
{
  /*
   * Joint states
   */
  std::string baselink_frame = gazebo_ros_->resolveTF("base_link");
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.header.frame_id = baselink_frame;

  #if GAZEBO_MAJOR_VERSION >= 9
    joint_state_.position[LEFT] = joints_[LEFT]->Position(0);
    joint_state_.position[RIGHT] = joints_[RIGHT]->Position(0);
  #else
    joint_state_.position[LEFT] = joints_[LEFT]->GetAngle(0).Radian();
    joint_state_.position[RIGHT] = joints_[RIGHT]->GetAngle(0).Radian();
  #endif

  joint_state_.velocity[LEFT] = joints_[LEFT]->GetVelocity(0);
  joint_state_.velocity[RIGHT] = joints_[RIGHT]->GetVelocity(0);


  joint_state_pub_.publish(joint_state_);
}

/*
 * Odometry (encoders & IMU)
 */
void GazeboRosKobuki::updateOdometry(common::Time& step_time)
{
  // std::string odom_frame = gazebo_ros_->resolveTF("odom");
  // std::string base_frame = gazebo_ros_->resolveTF("base_footprint");
  odom_.header.stamp = joint_state_.header.stamp;
  odom_.header.frame_id = odom_frame_str;
  odom_.child_frame_id = base_frame_str;

  ckf_odom_msg_.header.stamp = joint_state_.header.stamp;
  ckf_odom_msg_.header.frame_id = ugv_n + "/hast/odom";
  ckf_odom_msg_.child_frame_id = ugv_n + "/hast/base_footprint";

  ugvn_hast_tf.header.frame_id = ckf_odom_msg_.header.frame_id;
  ugvn_hast_tf.child_frame_id = ckf_odom_msg_.child_frame_id;


  // Distance travelled by main wheels
  double d1, d2, nd1, nd2;
  double dr, da, nda, ndr;
  d1 = d2 = nd1 = nd2 = 0;
  dr = da = ndr = nda = 0;
  
  // Can see NaN values here, just zero them out if needed
  d1 = step_time.Double() * (wheel_diam_ / 2) * joints_[LEFT]->GetVelocity(0);
    if (std::isnan(d1)){d1 = 0;}
  d2 = step_time.Double() * (wheel_diam_ / 2) * joints_[RIGHT]->GetVelocity(0);
    if (std::isnan(d2)){d2 = 0;}

  std::normal_distribution<double> distribution(normMean, normStd);   
  double noise_left, noise_right;
  
  if (cmd_vel_bool)
  {
    noise_left = distribution(generator);
    noise_right= distribution(generator);
  } else {
    noise_left = noise_right = 0;
  }

  nd1 = step_time.Double() * (wheel_diam_ / 2) * (joints_[LEFT]->GetVelocity(0)+ noise_left);
    if (std::isnan(nd1)){nd1 = 0;}
  nd2 = step_time.Double() * (wheel_diam_ / 2) * (joints_[RIGHT]->GetVelocity(0)+ noise_right);
    if (std::isnan(nd2)){nd2 = 0;}

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / wheel_sep_; // ignored

  ndr = (nd1 + nd2) / 2;
  nda = (nd2 - nd1) / wheel_sep_; // ignored

  // Just as in the Kobuki driver, the angular velocity is taken directly from the IMU
  vel_angular_ = imu_->AngularVelocity();

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );

  ckf_odom_pose_[0] += ndr * cos( ckf_odom_pose_[2] );
  ckf_odom_pose_[1] += ndr * sin( ckf_odom_pose_[2] );

  #if GAZEBO_MAJOR_VERSION >= 9
    odom_pose_[2] += vel_angular_.Z() * step_time.Double();
    ckf_odom_pose_[2] += vel_angular_.Z() * step_time.Double();
  #else
    odom_pose_[2] += vel_angular_.z * step_time.Double();
    ckf_odom_pose_[2] += vel_angular_.z * step_time.Double();
  #endif

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;

  ckf_odom_vel_[0] = ndr / step_time.Double();
  ckf_odom_vel_[1] = 0.0;

  #if GAZEBO_MAJOR_VERSION >= 9
    odom_vel_[2] = vel_angular_.Z();
    ckf_odom_vel_[2] = vel_angular_.Z();
  #else
    odom_vel_[2] = vel_angular_.z;
    ckf_odom_vel_[2] = vel_angular_.z;
  #endif

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  ckf_odom_msg_.pose.pose.position.x = ckf_odom_pose_[0];
  ckf_odom_msg_.pose.pose.position.y = ckf_odom_pose_[1];
  ckf_odom_msg_.pose.pose.position.z = 0;

  tf::Quaternion qt, nqt;
  qt.setEuler(0,0,odom_pose_[2]);
  nqt.setEuler(0,0,ckf_odom_pose_[2]);

  odom_.pose.pose.orientation.x = qt.getX();
  odom_.pose.pose.orientation.y = qt.getY();
  odom_.pose.pose.orientation.z = qt.getZ();
  odom_.pose.pose.orientation.w = qt.getW();

  ckf_odom_msg_.pose.pose.orientation.x = nqt.getX();
  ckf_odom_msg_.pose.pose.orientation.y = nqt.getY();
  ckf_odom_msg_.pose.pose.orientation.z = nqt.getZ();
  ckf_odom_msg_.pose.pose.orientation.w = nqt.getW();

  odom_.pose.covariance[0]  = 0.1;
  odom_.pose.covariance[7]  = 0.1;
  odom_.pose.covariance[35] = 0.05;
  odom_.pose.covariance[14] = 1e6;
  odom_.pose.covariance[21] = 1e6;
  odom_.pose.covariance[28] = 1e6;

  ckf_odom_msg_.pose.covariance[0]  = 0.1;
  ckf_odom_msg_.pose.covariance[7]  = 0.1;
  ckf_odom_msg_.pose.covariance[35] = 0.05;
  ckf_odom_msg_.pose.covariance[14] = 1e6;
  ckf_odom_msg_.pose.covariance[21] = 1e6;
  ckf_odom_msg_.pose.covariance[28] = 1e6;

  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = odom_vel_[2];
  odom_pub_.publish(odom_); // publish odom message
  
  ckf_odom_msg_.twist.twist.linear.x = ckf_odom_vel_[0];
  ckf_odom_msg_.twist.twist.linear.y = 0;
  ckf_odom_msg_.twist.twist.linear.z = 0;
  ckf_odom_msg_.twist.twist.angular.x = 0;
  ckf_odom_msg_.twist.twist.angular.y = 0;
  ckf_odom_msg_.twist.twist.angular.z = ckf_odom_vel_[2];
  ckf_wheel_pub_.publish(ckf_odom_msg_); // publish odom message

  if (publish_tf_)
  {
    odom_tf_.header = odom_.header;
    odom_tf_.child_frame_id = odom_.child_frame_id;
    odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf_.transform.rotation = odom_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf_);
  }
}


/*
 * Publish IMU data
 */
void GazeboRosKobuki::updateIMU()
{
  imu_msg_.header = joint_state_.header;

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Quaterniond quat = imu_->Orientation();
    imu_msg_.orientation.x = quat.X();
    imu_msg_.orientation.y = quat.Y();
    imu_msg_.orientation.z = quat.Z();
    imu_msg_.orientation.w = quat.W();
  #else
    math::Quaternion quat = imu_->Orientation();
    imu_msg_.orientation.x = quat.x;
    imu_msg_.orientation.y = quat.y;
    imu_msg_.orientation.z = quat.z;
    imu_msg_.orientation.w = quat.w;
  #endif


  imu_msg_.orientation_covariance[0] = 1e6;
  imu_msg_.orientation_covariance[4] = 1e6;
  imu_msg_.orientation_covariance[8] = 0.05;

  #if GAZEBO_MAJOR_VERSION >= 9
    imu_msg_.angular_velocity.x = vel_angular_.X();
    imu_msg_.angular_velocity.y = vel_angular_.Y();
    imu_msg_.angular_velocity.z = vel_angular_.Z();
  #else
    imu_msg_.angular_velocity.x = vel_angular_.x;
    imu_msg_.angular_velocity.y = vel_angular_.y;
    imu_msg_.angular_velocity.z = vel_angular_.z;
  #endif


  imu_msg_.angular_velocity_covariance[0] = 1e6;
  imu_msg_.angular_velocity_covariance[4] = 1e6;
  imu_msg_.angular_velocity_covariance[8] = 0.05;

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d lin_acc = imu_->LinearAcceleration();
    imu_msg_.linear_acceleration.x = lin_acc.X();
    imu_msg_.linear_acceleration.y = lin_acc.Y();
    imu_msg_.linear_acceleration.z = lin_acc.Z();
  #else
    math::Vector3 lin_acc = imu_->LinearAcceleration();
    imu_msg_.linear_acceleration.x = lin_acc.x;
    imu_msg_.linear_acceleration.y = lin_acc.y;
    imu_msg_.linear_acceleration.z = lin_acc.z;
  #endif


  imu_pub_.publish(imu_msg_); // publish odom message
}

/*
 * Propagate velocity commands
 * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
 */
void GazeboRosKobuki::propagateVelocityCommands()
{
  if (((prev_update_time_- last_cmd_vel_time_).Double() > cmd_vel_timeout_) || !motors_enabled_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }
  joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));


  // test_msg_.linear.x = wheel_speed_cmd_[LEFT];
  // test_msg_.linear.y = wheel_speed_cmd_[RIGHT];
  // test_msg_.linear.z = 0.0;
  // test_msg_.angular.x = joints_[LEFT]->GetVelocity(0);
  // test_msg_.angular.y = joints_[RIGHT]->GetVelocity(0);
  // test_msg_.angular.z = 0.0;

  // test_pub_.publish(test_msg_);


}

/*
 * Cliff sensors
 * Check each sensor separately
 */
void GazeboRosKobuki::updateCliffSensor()
{
  // Left cliff sensor
  if ((cliff_detected_left_ == false) &&
      (cliff_sensor_left_->Range(0) >= cliff_detection_threshold_))
  {
    cliff_detected_left_ = true;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::LEFT;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_left_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_left_ == true) &&
            (cliff_sensor_left_->Range(0) < cliff_detection_threshold_))
  {
    cliff_detected_left_ = false;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::LEFT;
    cliff_event_.state = kobuki_msgs::CliffEvent::FLOOR;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_left_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  // Centre cliff sensor
  if ((cliff_detected_center_ == false) &&
      (cliff_sensor_center_->Range(0) >= cliff_detection_threshold_))
  {
    cliff_detected_center_ = true;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::CENTER;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_center_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_center_ == true) &&
            (cliff_sensor_center_->Range(0) < cliff_detection_threshold_))
  {
    cliff_detected_center_ = false;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::CENTER;
    cliff_event_.state = kobuki_msgs::CliffEvent::FLOOR;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_center_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  // Right cliff sensor
  if ((cliff_detected_right_ == false) &&
      (cliff_sensor_right_->Range(0) >= cliff_detection_threshold_))
  {
    cliff_detected_right_ = true;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::RIGHT;
    cliff_event_.state = kobuki_msgs::CliffEvent::CLIFF;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_right_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_right_ == true) &&
            (cliff_sensor_right_->Range(0) < cliff_detection_threshold_))
  {
    cliff_detected_right_ = false;
    cliff_event_.sensor = kobuki_msgs::CliffEvent::RIGHT;
    cliff_event_.state = kobuki_msgs::CliffEvent::FLOOR;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_right_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
}

/*
 * Bumpers
 */
// In order to simulate the three bumper sensors, a contact is assigned to one of the bumpers
// depending on its position. Each sensor covers a range of 60 degrees.
// +90 ... +30: left bumper
// +30 ... -30: centre bumper
// -30 ... -90: right bumper
void GazeboRosKobuki::updateBumper()
{
  // reset flags
  bumper_left_is_pressed_ = false;
  bumper_center_is_pressed_ = false;
  bumper_right_is_pressed_ = false;

  // parse contacts
  msgs::Contacts contacts;
  contacts = bumper_->Contacts();
  double robot_heading;
  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d current_pose = model_->WorldPose();
    robot_heading = current_pose.Rot().Yaw();
  #else
    math::Pose current_pose = model_->GetWorldPose();
    robot_heading = current_pose.rot.GetYaw();
  #endif


  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    double rel_contact_pos;
    #if GAZEBO_MAJOR_VERSION >= 9
      rel_contact_pos =  contacts.contact(i).position(0).z() - current_pose.Pos().Z();
    #else
      rel_contact_pos =  contacts.contact(i).position(0).z() - current_pose.pos.z;
    #endif
    // Actually, only contacts at the height of the bumper should be considered, but since we use a simplified collision model
    // contacts further below and above need to be consider as well to identify "bumps" reliably.
    if ((rel_contact_pos >= 0.01)
        && (rel_contact_pos <= 0.13))
    {
      // using the force normals below, since the contact position is given in world coordinates
      // also negating the normal, because it points from contact to robot centre
      double global_contact_angle = std::atan2(-contacts.contact(i).normal(0).y(), -contacts.contact(i).normal(0).x());
      double relative_contact_angle = global_contact_angle - robot_heading;

      if ((relative_contact_angle <= (M_PI/2)) && (relative_contact_angle > (M_PI/6)))
      {
        bumper_left_is_pressed_ = true;
      }
      else if ((relative_contact_angle <= (M_PI/6)) && (relative_contact_angle >= (-M_PI/6)))
      {
        bumper_center_is_pressed_ = true;
      }
      else if ((relative_contact_angle < (-M_PI/6)) && (relative_contact_angle >= (-M_PI/2)))
      {
        bumper_right_is_pressed_ = true;
      }
    }
  }

  // check for bumper state change
  if (bumper_left_is_pressed_ && !bumper_left_was_pressed_)
  {
    bumper_left_was_pressed_ = true;
    bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::LEFT;
    bumper_event_pub_.publish(bumper_event_);
  }
  else if (!bumper_left_is_pressed_ && bumper_left_was_pressed_)
  {
    bumper_left_was_pressed_ = false;
    bumper_event_.state = kobuki_msgs::BumperEvent::RELEASED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::LEFT;
    bumper_event_pub_.publish(bumper_event_);
  }
  if (bumper_center_is_pressed_ && !bumper_center_was_pressed_)
  {
    bumper_center_was_pressed_ = true;
    bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::CENTER;
    bumper_event_pub_.publish(bumper_event_);
  }
  else if (!bumper_center_is_pressed_ && bumper_center_was_pressed_)
  {
    bumper_center_was_pressed_ = false;
    bumper_event_.state = kobuki_msgs::BumperEvent::RELEASED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::CENTER;
    bumper_event_pub_.publish(bumper_event_);
  }
  if (bumper_right_is_pressed_ && !bumper_right_was_pressed_)
  {
    bumper_right_was_pressed_ = true;
    bumper_event_.state = kobuki_msgs::BumperEvent::PRESSED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::RIGHT;
    bumper_event_pub_.publish(bumper_event_);
  }
  else if (!bumper_right_is_pressed_ && bumper_right_was_pressed_)
  {
    bumper_right_was_pressed_ = false;
    bumper_event_.state = kobuki_msgs::BumperEvent::RELEASED;
    bumper_event_.bumper = kobuki_msgs::BumperEvent::RIGHT;
    bumper_event_pub_.publish(bumper_event_);
  }
}

void GazeboRosKobuki::publishViconBaseTF()
{

  math::Pose base_pose = model_->GetWorldPose();
    vicon_tf_.header.stamp = joint_state_.header.stamp;
    vicon_tf_.header.frame_id = "/vicon/origin";
    vicon_tf_.child_frame_id = viconBaseTF_frame_;
    vicon_tf_.transform.translation.x = base_pose.pos.x;
    vicon_tf_.transform.translation.y = base_pose.pos.y;
    vicon_tf_.transform.translation.z = base_pose.pos.z;
    vicon_tf_.transform.rotation.x = base_pose.rot.x;
    vicon_tf_.transform.rotation.y = base_pose.rot.y;
    vicon_tf_.transform.rotation.z = base_pose.rot.z;
    vicon_tf_.transform.rotation.w = base_pose.rot.w;
    tf_broadcaster_.sendTransform(vicon_tf_);
      // ROS_INFO("vicon_tf_.header.child_frame_id: %s", viconBaseTF_frame_.c_str());
      // ROS_INFO("base_pose.t [x,y,z] = [%6.4f %6.4f %6.4f]", base_pose.pos.x, base_pose.pos.y, base_pose.pos.z);
      // ROS_INFO("base_pose.q [x,y,z,w] = [%6.4f %6.4f %6.4f %6.4f]", base_pose.rot.x, base_pose.rot.y, base_pose.rot.z, base_pose.rot.w);

    hast_tf_.header.stamp = joint_state_.header.stamp;
    hast_tf_.header.frame_id = odom_frame_str;
    hast_tf_.child_frame_id = base_frame_str;


    // hast_tf_.header.frame_id = "hast/kobuki/odom";
    // hast_tf_.child_frame_id = "hast/kobuki/base_footprint";
    hast_tf_.transform.translation.x = odom_.pose.pose.position.x;
    hast_tf_.transform.translation.y = odom_.pose.pose.position.y;
    hast_tf_.transform.translation.z = odom_.pose.pose.position.z;
    hast_tf_.transform.rotation = odom_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(hast_tf_);


    ugvn_hast_tf.header.stamp = joint_state_.header.stamp;
    ugvn_hast_tf.transform.translation.x = ckf_odom_msg_.pose.pose.position.x;
    ugvn_hast_tf.transform.translation.y = ckf_odom_msg_.pose.pose.position.y;
    ugvn_hast_tf.transform.translation.z = ckf_odom_msg_.pose.pose.position.z;
    ugvn_hast_tf.transform.rotation = ckf_odom_msg_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(ugvn_hast_tf);

    // ROS_INFO("tf_broadcaster_.sendTransform(ugvn_hast_tf);");
    // std::string parent = ugvn_hast_tf.header.frame_id;
    // std::string child = ugvn_hast_tf.child_frame_id;
    // ROS_INFO("ugvn_hast_tf.header.frame_id: %s", parent.c_str());
    // ROS_INFO("ugvn_hast_tf.child_frame_id: %s", child.c_str());


}

} // end namespace gazebo {







