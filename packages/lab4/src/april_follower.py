#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray




class AprilTagFollower:
	def __init__(self):
		rospy.init_node('AprilTagFollower')
		#host = os.getenv('HOSTNAME')
		rospy.Subscriber('/rombie/apriltag_detector_node/detections',AprilTagDetectionArray,self.tag_orient)
		self.pub = rospy.Publisher('/rombie/car_cmd_switch_node/cmd',Twist2DStamped,queue_size=1)
		self.control = Twist2DStamped()
		self.kp = 10
		self.ki = 0.1
		self.kd = 3

		self.integral = 0
		self.last_error = 0
		self.error_tolerance = 0.05
		self.control_signal_tolerance = 0.02
		
		self.kp_f = 1
		self.ki_f = 0.05
		self.kd_f = 0.2

		self.integral_f = 0
		self.last_error_f = 0
		self.error_tolerance_f = 0.01
		self.control_signal_tolerance_f = 0.1
	def tag_orient(self, msg):
		#rospy.logwarn("Entered callback")
		detect = msg.detections
		if len(detect) == 0:
				self.control.omega=0
				self.control.v=0
				self.pub.publish(self.control)
		
		for x in detect:
			#rospy.logwarn("Entered for loop")
			err = -detect[0].transform.translation.x
			err_f = -(.1 - detect[0].transform.translation.z)
			
			if abs(err) < self.error_tolerance:
				rospy.logwarn("err is less than tolerance")
				# If the error is within the tolerance range, set the control signal to zero
				self.control.omega = 0
			else:
				rospy.loginfo("Doing PID calculations")
				# Compute the proportional term
				proportional = self.kp * err

				# Compute the integral term
				
				self.integral += err
				integral = self.ki * self.integral

				# Compute the derivative term
				derivative = self.kd * (err - self.last_error)
				self.last_error = err

				# Compute the control signal using the PID terms
				self.control.omega = proportional + integral + derivative

				# Check if the control signal is within the tolerance range
				if abs(self.control.omega) < self.control_signal_tolerance:
					# If the control signal is within the tolerance range, set it to zero
					self.control.omega=0
			# Publish the control signal to the control_signal topic
			if abs(err_f) < self.error_tolerance_f:
				rospy.logwarn("Within 10-ish centimeters of sign")
				# If the error is within the tolerance range, set the control signal to zero
				self.control.v = 0
			else:
				rospy.loginfo("Doing length PID calculations")
				# Compute the proportional term
				proportional_f = self.kp_f * err_f

				# Compute the integral term
				
				self.integral_f += err_f
				integral_f = self.ki_f * self.integral_f
				rospy.logwarn(integral_f)

				# Compute the derivative term
				derivative_f = self.kd_f * (err_f - self.last_error_f)
				self.last_error_f = err_f

				# Compute the control signal using the PID terms
				self.control.v = proportional_f + integral_f + derivative_f
				# Check if the control signal is within the tolerance range
				if abs(self.control.v) < self.control_signal_tolerance_f:
					# If the control signal is within the tolerance range, set it to zero
					self.control.v=0
				if abs(self.control.v) > .4:
					# If the control signal is within the tolerance range, set it to zero
					self.control.v=.4
				
			self.pub.publish(self.control)
			

if __name__ == '__main__':
	try:
		pid_controller = AprilTagFollower()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

'''
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray

class PIDnode:
    def __init__(self):
        rospy.Subscriber('/rombie/apriltag_detector_node/detections',Twist2DStamped,queue_size=1)
        pub = rospy.Publisher('/rombie/car_cmd_switch_node/cmd',Twist2DStamped,queue_size=1)
        self.error_intergration = 0
        self.error_derivative = 0
        self.error_product = 0

        self.t1 = 0
        self.t0 = 0
        self.last_errorVal = 0
    
    def callback(self, msg) :
        #pub = rospy.Publisher('/control_input', Float32, queue_size=1)
        errorVal = msg.detections.pose_error
        kp = 0.27
        ki = 0.1
        kd = 0.55
        self.t0 = rospy.get_time()
        dt = self.t0 - self.t1
        self.error_product = kp*errorVal
        self.error_intergration = ki*(self.error_intergration +(self.error_intergration*dt))
        self.error_derivative = kd*((errorVal - self.last_errorVal)/dt)
        op_value = (self.error_product + self.error_intergration + self.error_derivative)
        pub.publish(op_value)

        self.t1 = self.t0
        self.last_errorVal = errorVal




if __name__ == '__main__':
    rospy.init_node('pid_node', anonymous=True)
    bob = PIDnode()
    rospy.spin()
'''
