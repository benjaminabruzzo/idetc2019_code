cd ~/ros
catkin build apriltags_ros -DCMAKE_BUILD_TYPE=Release
catkin build hast hast_gazebo hast_gazebo_msgs message_to_tf metahast robot_descriptions robot_plugins hast_multi
source ~/ros/devel/setup.sh
