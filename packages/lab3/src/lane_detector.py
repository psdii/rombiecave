#!/usr/bin/env python3
import rospy
import cv2
import message_filters
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from math import pi, cos, sin

class ImageCropper:
	def __init__(self):
		#rospy.Subscriber("/rombie/camera_node/image/compressed", CompressedImage, self.cropper_cb)
		rospy.Subscriber("/rombie/camera_node/image/compressed", CompressedImage, self.cropper_cb, queue_size=1, buff_size=2**24)
		
		self.pub1 = rospy.Publisher("/cropped_lines", Image, queue_size=10)
		#self.pub2 = rospy.Publisher("/yellow_lines", Image, queue_size=10)
		self.pub3 = rospy.Publisher("/lane_lines", Image, queue_size=10)
		self.pub4 = rospy.Publisher("/test", Image, queue_size=10)
		# Instantiate the converter class once by using a class member
		self.bridge = CvBridge()
	def cropper_cb(self, msg):
		# convert to a ROS image using the bridge
		cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		# flip along the horizontal axis using an OpenCV function
		
		image_size = (160, 120)
		offset = 40
		new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
		cv_cropped = new_image[offset:, :]	
		rospy.loginfo("QUACK")
		
		#cv_cropped = cv_img[240:480,0:640]
		hsv_cropped = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
		
		# convert new image to ROS to send
		
		sensitivity = 60
		white_cropped = cv2.inRange(hsv_cropped, (0,0,255-sensitivity), (255,sensitivity,255))
		yellow_cropped = cv2.inRange(hsv_cropped, (0,70,90), (70,255,255))
		
		#ros_yellow = self.bridge.cv2_to_imgmsg(yellow_cropped, "mono8")
		#ros_white = self.bridge.cv2_to_imgmsg(white_cropped, "mono8")
		#ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped,"bgr8")
		
		# convert to a ROS image using the bridge
		cv_edges = cv2.Canny(cv_cropped, 100, 200)
		
		
		ros_edges = self.bridge.cv2_to_imgmsg(cv_edges, "mono8")
		
		self.pub1.publish(ros_edges)
		
		#yellow portion
		cv_outliney = cv2.Canny(yellow_cropped,100,200)
		lines = cv2.HoughLinesP(cv_outliney,1,pi/180,2,2,2)
		
		cv_imge_lines = cv_cropped.copy()
		if lines is not None:
			for line in lines:
				x1, y1, x2, y2 = line[0]
				cv2.line(cv_imge_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)
			#ros_linesy = self.bridge.cv2_to_imgmsg(cv_imge_yellowlines, "bgr8")
			#self.pub2.publish(ros_linesy)
		
		#white portion
		cv_outlinew = cv2.Canny(white_cropped,100,200)
		self.pub4.publish(self.bridge.cv2_to_imgmsg(cv_outlinew, "mono8"))
		lines = cv2.HoughLinesP(cv_outlinew,1,pi/180,2,2,2)
		
		#cv_imge_whitelines = cv_cropped.copy()
		if lines is not None:
			for line in lines:
				x1, y1, x2, y2 = line[0]
				cv2.line(cv_imge_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)
			ros_linesw = self.bridge.cv2_to_imgmsg(cv_imge_lines, "bgr8")
			self.pub3.publish(ros_linesw)
		
		# publish flipped image
		#self.pub.publish(ros_yellow)
		#self.pub2.publish(ros_white)
		#self.pub3.publish(ros_cropped)

if __name__=="__main__":
	# initialize our node and create a publisher as normal
	rospy.init_node("image_cropper", anonymous=True)
	img_crop = ImageCropper()
	rospy.spin()
