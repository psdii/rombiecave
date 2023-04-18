#!/usr/bin/env python3

# draw square in Python Turtle
import rospy 
import math
import turtle
import sys
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def move_turtle():
	i = 0;
	rospy.init_node('move_turtle', anonymous=True)
	pub = rospy.Publisher('/turtlesim/turtle1/cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(0.5) 	
	
	vel = Twist()
	
	pub.publish(vel)
	rate.sleep()
	#while not rospy.is_shutdown():
	#for i in range(4):	
		vel.linear.x = 2
		distance = 2
		time = distance/vel.linear.x
		
		start_time = rospy.Time.now()
		end_time = start_time + rospy.Duration.from_sec(time)
		
		while rospy.Time.now() < end_time:
			pub.publish(vel)
		vel.linear.x = 0
		pub.publish(vel)


		vel.angular.z = 1
		distance = (math.pi)/2
		time = distance/vel.angular.z
		
		start_time = rospy.Time.now()
		end_time = start_time + rospy.Duration.from_sec(time)
		
		while rospy.Time.now() < end_time:
			pub.publish(vel)
		vel.angular.z = 0
		pub.publish(vel)
	
	

if __name__ == '__main__':
	try:
		move_turtle()
	except rospy.ROSInterruptException:
		pass
