Gazebo topics and TF:
======

in gazebo_vicon_diff_drive.cpp:

	void GazeboViconDiffDrive::publishViconBaseTF()
	transform_broadcaster_->sendTransform (tf::StampedTransform ( baseTF, current_time, "/vicon/origin", viconBaseTF_frame_ ) );

viconBaseTF_frame_ is defined as:

	gazebo_ros_->getParameter<std::string> ( viconBaseTF_frame_, "viconBaseTF", "/vicon/kobuki/base_TF" );
	<viconBaseTF>/vicon/${X_ROBOT_NAMESPACE}/base_TF</viconBaseTF>
	[ INFO] [1541643670.874272629, 0.173000000]: viconBaseTF_frame_ :  [/vicon/ugv1/base_TF]


This is the position of the ugv in the gazebo/vicon world frame.  It is the actual position, not an estimated position.







gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
gazebo_ros_->getParameter<std::string> ( driverOdom_frame_, "driverOdom", "/hast/kobuki/odom" );
gazebo_ros_->getParameter<std::string> ( driverFootprint_frame_, "driverFootprint", "/hast/kobuki/base_footprint" );

rostopic pub -r 10 /ugv1/cmd_vel_raw geometry_msgs/Twist   '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.5}}'
rostopic pub -r 10 /ugv2/cmd_vel_raw geometry_msgs/Twist   '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -0.5}}'
rostopic pub -r 10 /ugv3/cmd_vel_raw geometry_msgs/Twist   '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.5}}'
rostopic pub -r 10 /ugv4/cmd_vel_raw geometry_msgs/Twist   '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -0.5}}'
