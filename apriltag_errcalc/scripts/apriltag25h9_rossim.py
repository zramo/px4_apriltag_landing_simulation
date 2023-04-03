#! /usr/bin/env python
import apriltag
import cv2
import numpy as np
import math
import sys

import rospy
from std_msgs.msg import Float32, String
from apriltag_errcalc.msg import TagPoseErr
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class apriltag_errcalc:
	def __init__(self):
		self.pub_tagerr = rospy.Publisher('/apriltag_errcalc/tagerr', TagPoseErr, queue_size=10)
		self.bridge = CvBridge()

		#self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
		self.image_sub = rospy.Subscriber("/iris_0/camera/image_raw",Image,self.callback)
		self.image_pub = rospy.Publisher('/apriltag_errcalc/image_processed', Image)

		curr_info_img_sub = rospy.Subscriber("/offb_node_py/curr_info", String, self.curr_info_cb)

	curr_info_str = String()
	def curr_info_cb(self, info_str):
		self.curr_info_str = info_str
		pass

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as err:
			print(err)
		
		apriltagDetector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

		frame_shape = cv_image.shape
		frame_w = frame_shape[1]
		frame_h = frame_shape[0]

		#print(frame_w)
		#print(frame_h)
		curr_tag_pose_err = TagPoseErr()
		
		grayFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		tags = apriltagDetector.detect(grayFrame)
		# font
		font = cv2.FONT_HERSHEY_SIMPLEX
		
		# fontScale
		fontScale = 1
		# Blue color in BGR
		color = (255, 0, 0)
		# Line thickness of 2 px
		thickness = 2
		# Using cv2.putText() method
		curr_print_str = ''
		line_cnt = 0
		for curr_ch in range(len(self.curr_info_str.data)):
			#print(self.curr_info_str.data[curr_ch])
			if(self.curr_info_str.data[curr_ch] == '\n'):
				line_cnt += 1
				cv2.putText(cv_image, curr_print_str, (50, 20+line_cnt*36), font, 1,  color, 2, cv2.LINE_AA)
				curr_print_str = ''
			else:
				curr_print_str += self.curr_info_str.data[curr_ch]
			pass
		

		for tag in tags:
			if(tag.tag_id != 21):
				continue
			(cornerA, cornerB, cornerC, cornerD) = tag.corners
			cornerA = (int(cornerA[0]), int(cornerA[1]))
			cornerB = (int(cornerB[0]), int(cornerB[1]))
			cornerC = (int(cornerC[0]), int(cornerC[1]))
			cornerD = (int(cornerD[0]), int(cornerD[1]))
			center = (int((cornerA[0]+cornerB[0]+cornerC[0]+cornerD[0])/4),\
					int((cornerA[1]+cornerB[1]+cornerC[1]+cornerD[1])/4)\
					)
			cv2.circle(cv_image, cornerA, 4, (255, 0, 0), 3)
			cv2.circle(cv_image, cornerB, 4, (0, 255, 0), 3)
			cv2.circle(cv_image, cornerC, 4, (0, 0, 255), 3)
			cv2.circle(cv_image, cornerD, 4, (255, 255, 0), 3)
			cv2.circle(cv_image, center, 4, (128, 128, 255), 3)
			size_ratio = frame_shape[1]
			tag_error = (\
				(1)*(center[0]-frame_shape[1]/2.0)/size_ratio,\
				(1)*(center[1]-frame_shape[0]/2.0)/size_ratio\
				)
			curr_tag_pose_err.w_err = tag_error[0]
			curr_tag_pose_err.h_err = tag_error[1]

			#print(curr_tag_pose_err)
			try:
				#print("ERR_SENT")
				self.pub_tagerr.publish(curr_tag_pose_err)
			except CvBridgeError as err:
				print(err)
			
			break
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as err:
			print(err)
		#print("shown")
		cv2.imshow('USB GLOBAL SHUTTER CAM',cv_image)
		#cv2.imwrite('/home/hy/Desktop/111/SOMETHING.BMP', cv_image)
		


def main(args):
	rospy.init_node('apriltag_node_python', anonymous=True)
	app_aptg = apriltag_errcalc()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("KeyboardInterrupt:Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)