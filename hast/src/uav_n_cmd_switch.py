#!/usr/bin/env python

# Import required Python code.
import cv2
import roslib
import rospy
from geometry_msgs.msg import Twist

class cmd_remap():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		input_topic = rospy.get_param('~input_topic')
		output_topic = rospy.get_param('~output_topic')

		# subscribers and publishers
		self.cmd_vel_sub = rospy.Subscriber(input_topic,Twist, self.cmd_vel_callback)
		self.cmd_vel_pub = rospy.Publisher(output_topic,Twist, queue_size=10)
		
		while not rospy.is_shutdown():
			rospy.spin()


	def cmd_vel_callback(self,data):
		self.cmd_vel_pub.publish(data)
		# self.cmd_vel_pub_msg = Twist()
		# if abs(data.linear.x) >= self.cmd_vel_x_limit:
		# 	if data.linear.x >= 0.0:
		# 		self.cmd_vel_pub_msg.linear.x = self.cmd_vel_x_limit
		# 	else:
		# 		self.cmd_vel_pub_msg.linear.x = -self.cmd_vel_x_limit
		# else:
		# 		self.cmd_vel_pub_msg.linear.x = data.linear.x
		# if abs(data.angular.z) >= self.cmd_vel_w_limit:
		# 	if data.angular.z >= 0.0:
		# 		self.cmd_vel_pub_msg.angular.z = self.cmd_vel_w_limit
		# 	else:
		# 		self.cmd_vel_pub_msg.angular.z = -self.cmd_vel_w_limit
		# else:
		# 		self.cmd_vel_pub_msg.angular.z = data.angular.z
		# # print "cmd_vel.linear.x : " + str(data.linear.x)
		# # print "cmd_vel.angular.z : " + str(data.angular.z)
		# # print "self.cmd_vel_pub_msg.linear.x : " + str(self.cmd_vel_pub_msg.linear.x)
		# # print "self.cmd_vel_pub_msg.angular.z : " + str(self.cmd_vel_pub_msg.angular.z)
		# # fprintf (mFile, "uavRecorder.ckfinit.Rk(1,:) = [%6.14f, %6.14f, %6.14f, %6.14f];\n",ckf.Rk.at<double>(0, 0),ckf.Rk.at<double>(0, 1),ckf.Rk.at<double>(0, 2),ckf.Rk.at<double>(0, 3));
		




if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('uav_n_cmd_remap')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        c_rmp = cmd_remap()
    except rospy.ROSInterruptException: pass
