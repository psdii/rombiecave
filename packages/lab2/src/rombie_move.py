#!/usr/bin/env python3

# draw square in Python Turtle
import rospy 
import math
import turtle
import sys
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, FSMState

class move_duck():
	def __init__(self):
		rospy.Subscriber("/rombie/fsm_node/mode", FSMState, self.move_cb)
		self.movePub = rospy.Publisher('/rombie/lane_controller_node/car_cmd',Twist2DStamped,queue_size=1)
		#self.movePub = rospy.Publisher('/rombie/car_cmd_switch_node/cmd',Twist2DStamped,queue_size=1)
		#self.rate = rospy.Rate(0.2)

	def duckie_box(self):
		
		self.vel = Twist2DStamped()
		self.movePub.publish(self.vel)
		'''
		self.vel.v = 0
		self.vel.omega = 0
		self.movePub.publish(self.vel)
		'''
		rospy.sleep(5)
		vdur = 4
		tdur = .5
		sdur = 5
		for x in range(4):
			self.vel.v = .3
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			rospy.sleep(vdur)
			self.vel.v = 0
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			rospy.sleep(sdur)
			
			self.vel.omega = 4	
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			rospy.sleep(tdur)
			self.vel.omega = 0
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			self.movePub.publish(self.vel)
			rospy.sleep(sdur)
		'''
		
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(vdur)
		self.vel.v = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		
		self.vel.omega = 4	
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(tdur)
		self.vel.omega = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		
		
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(vdur)
		self.vel.v = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		
		self.vel.omega = 4	
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(tdur)
		self.vel.omega = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		
		
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(vdur)
		self.vel.v = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		
		self.vel.omega = 4	
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(tdur)
		self.vel.omega = 0
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		self.movePub.publish(self.vel)
		rospy.sleep(sdur)
		'''
		
		
		
	def move_cb(self, msg):
		if msg.state=="LANE_FOLLOWING":
			self.duckie_box()
'''
def move_duck():
	i = 0;
	
	pub = rospy.Publisher('/rombie/car_cmd_switch_node/cmd',Twist2DStamped,queue_size=10)
	rate = rospy.Rate(0.2) 	
	
	vel = Twist2DStamped()
	
	pub.publish(vel)
	rate.sleep()

	vel.v = .7
	distance = .07
	time = distance/vel.v
	
	start_time = rospy.Time.now()
	end_time = start_time + rospy.Duration.from_sec(time)
	
	pub.publish(vel)
	while rospy.Time.now() < end_time:
		pub.publish(vel)
	vel.v = 0
	pub.publish(vel)
	
	rate.sleep()

	vel.omega = 1.5
	distance = .005
	time = distance/vel.omega
	
	start_time = rospy.Time.now()
	end_time = start_time + rospy.Duration.from_sec(time)
	
	pub.publish(vel)
	rate.sleep()
	while rospy.Time.now() < end_time:
		pub.publish(vel)
	vel.omega = 0
	pub.publish(vel)
	

	rate.sleep()
	#while not rospy.is_shutdown():
	#for i in range(4):	
	vel.v = .7
	distance = .07
	time = distance/vel.v
	
	start_time = rospy.Time.now()
	end_time = start_time + rospy.Duration.from_sec(time)
	
	pub.publish(vel)
	rate.sleep()
	while rospy.Time.now() < end_time:
		pub.publish(vel)
	vel.v = 0
	pub.publish(vel)
'''


if __name__ == '__main__':
	rospy.init_node('move_turtle', anonymous=True)
	try:
		dog = move_duck()
		rospy.spin()
		#dog.move_cb()
		#dog.duckie_turn()
	except rospy.ROSInterruptException:
		pass
