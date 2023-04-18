#!/usr/bin/env python3

# draw square in Python Turtle
import rospy 
import math
import turtle
import sys
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped

def move_duck():
	i = 0;
	rospy.init_node('move_turtle', anonymous=True)
	pub = rospy.Publisher('/rombie/car_cmd_switch_node/cmd',Twist2DStamped,queue_size=10)
	rate = rospy.Rate(0.5) 	
	
	vel = Twist2DStamped()
	
	pub.publish(vel)
	rate.sleep()
	#while not rospy.is_shutdown():
	#for i in range(4):	
	vel.v = 2
	distance = 2
	time = distance/vel.linear.x
	
	start_time = rospy.Time.now()
	end_time = start_time + rospy.Duration.from_sec(time)
	
	while rospy.Time.now() < end_time:
		pub.publish(vel)
	vel.v = 0
	pub.publish(vel)

	'''
	vel.omega = 1
	distance = (math.pi)/2
	time = distance/vel.angular.z
	
	start_time = rospy.Time.now()
	end_time = start_time + rospy.Duration.from_sec(time)
	
	while rospy.Time.now() < end_time:
		pub.publish(vel)
	vel.omega = 0
	pub.publish(vel)
	'''
if __name__ == '__main__':
	try:
		move_duck()
	except rospy.ROSInterruptException:
		pass
