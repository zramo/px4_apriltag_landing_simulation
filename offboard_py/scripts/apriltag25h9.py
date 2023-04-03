import apriltag
import cv2
import numpy as np
import math

# call USB camera
camera = cv2.VideoCapture(0)
#call CSI camera
#camera = cv2.VideoCapture("nvarguscamerasrc\
#	!video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1 \
#	! nvvidconv flip-method=0 ! videoconvert !video/x-raw, format = BGR ! appsink")


apriltagDetector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

isRead, frame = camera.read()
frame_shape = frame.shape
frame_w = frame_shape[1]
frame_h = frame_shape[0]
#print(frame_w)
#print(frame_h)
while isRead:
	grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	tags = apriltagDetector.detect(grayFrame)

	for tag in tags:
		(cornerA, cornerB, cornerC, cornerD) = tag.corners
		cornerA = (int(cornerA[0]), int(cornerA[1]))
		cornerB = (int(cornerB[0]), int(cornerB[1]))
		cornerC = (int(cornerC[0]), int(cornerC[1]))
		cornerD = (int(cornerD[0]), int(cornerD[1]))
		center = (int((cornerA[0]+cornerB[0]+cornerC[0]+cornerD[0])/4),\
				  int((cornerA[1]+cornerB[1]+cornerC[1]+cornerD[1])/4)\
				)
		cv2.circle(frame, cornerA, 4, (255, 0, 0), 3)
		cv2.circle(frame, cornerB, 4, (0, 255, 0), 3)
		cv2.circle(frame, cornerC, 4, (0, 0, 255), 3)
		cv2.circle(frame, cornerD, 4, (255, 255, 0), 3)
		cv2.circle(frame, center, 4, (128, 128, 255), 3)
		tag_error = (center[0]-frame_shape[1]/2, center[1]-frame_shape[0]/2)
		
		print(tag_error)
		break
	#cv2.imshow('USB GLOBAL SHUTTER CAM',frame)
	if cv2.waitKey(1) == 27:#ESC quit
		break
	isRead, frame = camera.read()
	#cv2.imwrite('/home/hy/Desktop/111/SOMETHING.BMP', frame)

camera.release()
cv2.destroyAllWindows()