#!/usr/bin/env python3
import rospy
import message_filters
from turtlesim_helper.msg import UnitsLabelled
from odometry_hw.msg import DistWheel, Pose2D
from duckietown_msgs.msg import WheelEncoderStamped
from math import pi, cos, sin
class odoMeasure:
	def __init__(self):
		self.pub = rospy.Publisher("/rombie/pose", Pose2D, queue_size=10)
		left_sub = message_filters.Subscriber("/rombie/right_wheel_encoder_node/tick", WheelEncoderStamped)	
		right_sub = message_filters.Subscriber("/rombie/left_wheel_encoder_node/tick", WheelEncoderStamped)


		ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.1, allow_headerless=True)
		ts.registerCallback(self.odom)
		
		self.pose = Pose2D()
		
		self.started = 0
		
		self.left_dist_init = 0
		self.right_dist_init = 0

		self.x = 0
		self.y = 0
		self.theta = 0
		
	def odom(self, msg1, msg2):
		#rospy.loginfo(self.right_dist_cm)
		if self.started == 0:
			self.left_dist_init = msg1.data
			self.right_dist_init = msg2.data
			self.started = 1
		else:
			left_dist = msg1.data - self.left_dist_init
			right_dist = msg2.data - self.right_dist_init
			
			left_dist_cm = left_dist * 0.001592
			right_dist_cm = right_dist * 0.001592
			

			s_l = left_dist_cm
			s_r = right_dist_cm

			delta_s = (s_l + s_r)/2
			delta_theta = (s_r - s_l)/(0.1)
			delta_x = delta_s * cos(self.theta + delta_theta/2)
			delta_y = delta_s * sin(self.theta + delta_theta/2)
			self.x += delta_x
			self.y += delta_y
			self.theta += delta_theta

			if self.theta > pi:
			    self.theta -= 2*pi
			if self.theta < -pi:
			    self.theta += 2*pi

			pose = Pose2D()
			pose.x = self.x
			pose.y = self.y
			pose.theta = self.theta
			self.pub.publish(pose)
			


if __name__== "__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("odoMeasure", anonymous=True)
	odoMeasure()
	rospy.spin()
