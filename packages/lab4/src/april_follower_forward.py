#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped, LanePose
from std_srvs.srv import SetBool, SetBoolResponse



class AprilTagFollower:	
	def __init__(self):
		rospy.init_node('AprilTagFollower')
		rospy.Service('line_detector_node/switch', SetBool, self.ld_switch)
		rospy.Service('lane_filter_node/switch', SetBool, self.lf_switch)
		#host = os.getenv('HOSTNAME')
		rospy.Subscriber('/rombie/lane_filter_node/lane_pose',LanePose,self.tag_orient)
		self.pub = rospy.Publisher('/rombie/lane_controller_node/car_cmd',Twist2DStamped,queue_size=1)
		self.control = Twist2DStamped()
		self.kp = 10
		self.ki = 0.1
		self.kd = 0

		self.integral = 0
		self.last_error = 0
		self.error_tolerance = 0.5
		self.control_signal_tolerance = 0.02
		
		self.kp_f = 10
		self.ki_f = 0.1
		self.kd_f = 0

		self.integral_f = 0
		self.last_error_f = 0
		self.error_tolerance_f = 0.1
		self.control_signal_tolerance_f = 0.1
		
		rospy.logwarn("Outside Callback")
	def tag_orient(self, msg):
		omega_a = 0
		omega_b = 0
		rospy.logwarn("Entered callback")
		angle = msg.phi
		dis = msg.d
		#rospy.logwarn("Entered for loop")
		err = -angle
		err_f = -dis
		
		if abs(err) < self.error_tolerance and abs(err_f) < self.error_tolerance_f:
			rospy.logwarn("err is less than tolerance")
			# If the error is within the tolerance range, set the control signal to zero
			self.control.omega = 0
		else:
			#rospy.loginfo("Doing PID calculations")
			# Compute the proportional term
			proportional = self.kp * err

			# Compute the integral term
			
			self.integral += err
			integral = self.ki * self.integral

			# Compute the derivative term
			derivative = self.kd * (err - self.last_error)
			self.last_error = err

			# Compute the control signal using the PID terms
			#self.control.omega = proportional + integral + derivative
			omega_a = proportional + integral + derivative


			# Check if the control signal is within the tolerance range
			if abs(omega_a) < self.control_signal_tolerance:
				# If the control signal is within the tolerance range, set it to zero
				#self.control.omega=0
				omega_a = 0
		# Publish the control signal to the control_signal topic
		#else:
			#rospy.loginfo("Doing length PID calculations")
			# Compute the proportional term
			proportional_f = self.kp_f * err_f

			# Compute the integral term
			
			self.integral_f += err_f
			integral_f = self.ki_f * self.integral_f
			#rospy.logwarn(integral_f)

			# Compute the derivative term
			derivative_f = self.kd_f * (err_f - self.last_error_f)
			self.last_error_f = err_f

			# Compute the control signal using the PID terms
			#self.control.omega = proportional_f + integral_f + derivative_f
			omega_b = proportional_f + integral_f + derivative_f
			# Check if the control signal is within the tolerance range
			if abs(omega_b) < self.control_signal_tolerance_f:
				omega_b = 0
		self.control.v = .2
		
		self.control.omega = omega_a + omega_b
		rospy.loginfo(self.control.omega)
		self.pub.publish(self.control)
	def ld_switch(self, msg):
		return True, ""

	def lf_switch(self, msg):
		return True, ""

if __name__ == '__main__':
	try:
		pid_controller = AprilTagFollower()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
