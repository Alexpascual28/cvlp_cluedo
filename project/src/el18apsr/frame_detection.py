#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class frameDetection():
	def __init__(self):
		# Initialise self.sensitivity colour detection value (10 should be enough)
		self.sensitivity = 15

		# Initialise a CvBridge()
		self.bridge = CvBridge()

        # Set up a subscriber to the image topic you wish to use
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
		
	def callback(self, data):
		# Convert received image into opencv image
        # Wrap call to conversion method in exception handler
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as error:
			print(error)

		# Set upper and lower bounds for HSV colours
		# Red (Scarlet) = 0
		hsv_red_lower = np.array([0-self.sensitivity, 100, 100])
		hsv_red_upper = np.array([0+self.sensitivity, 255, 255])
		# Yellow (Mustard) = 30
		hsv_yellow_lower = np.array([30-self.sensitivity, 100, 100])
		hsv_yellow_upper = np.array([30+self.sensitivity, 255, 255])
        # Cyan (Peacock) = 90
		hsv_cyan_lower = np.array([90-self.sensitivity, 100, 100])
		hsv_cyan_upper = np.array([90+self.sensitivity, 255, 255])
		# Magenta (Plum) = 150
		hsv_magenta_lower = np.array([150-self.sensitivity, 30, 30])
		hsv_magenta_upper = np.array([150+self.sensitivity, 255, 255])
		
		# Convert rgb image into hsv image
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out colours separately using cv2.inRange() method
		red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
		yellow_mask = cv2.inRange(hsv_image, hsv_yellow_lower, hsv_yellow_upper)
		cyan_mask = cv2.inRange(hsv_image, hsv_cyan_lower, hsv_cyan_upper)
		magenta_mask = cv2.inRange(hsv_image, hsv_magenta_lower, hsv_magenta_upper)

        # Combine masks using cv2.bitwise_or() method (TWO images at once)
		ry_mask = cv2.bitwise_or(red_mask, yellow_mask)
		cm_mask = cv2.bitwise_or(cyan_mask, magenta_mask)
		colour_mask = cv2.bitwise_or(ry_mask, cm_mask)

		# Generalize mask by using morphology
		kernel = np.ones((30,30), np.uint8)
		colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, kernel)
		colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, kernel)
			
		# resize image
		# scale_percent = 40
		# width = int(cv_image.shape[1] * scale_percent / 100)
		# height = int(cv_image.shape[0] * scale_percent / 100)
		# dimensions = (width, height)
		# resized_image = cv2.resize(cv_image, dimensions, interpolation = cv2.INTER_AREA)
		# ratio = cv_image.shape[0] / float(resized_image.shape[0])
		
		# convert image into grayscale, blur it, pply threshold to binarize it and clean by applying morphology
		# grayscale_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
		# enhanced_image = cv2.equalizeHist(grayscale_image)
		# blurred_image = cv2.GaussianBlur(grayscale_image, (5,5), cv2.BORDER_DEFAULT)
		# threshold_image = cv2.threshold(blurred_image, 95, 255, cv2.THRESH_BINARY_INV) [1]
		# kernel = np.ones((10,10), np.uint8)
		# gradient_image = cv2.morphologyEx(threshold_image, cv2.MORPH_GRADIENT, kernel)
		# closing_image = cv2.morphologyEx(threshold_image, cv2.MORPH_CLOSE, kernel)
		
		frame_mask = colour_mask
		
		# width = int(small_mask.shape[1] * 100 / scale_percent)
		# height = int(small_mask.shape[0] * 100 / scale_percent)
		# dimensions = (width, height)
		# frame_mask = cv2.resize(small_mask, dimensions, interpolation = cv2.INTER_AREA)
		
		# Apply mask to original image using cv2.bitwise_and() method
		mask_image = cv2.bitwise_and(cv_image,cv_image, mask=frame_mask)
		
		# Detect contours within image
		contours = cv2.findContours(frame_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		# loop over contours
		for c in contours[0]:

			# calculate center of contour and classified name
			M = cv2.moments(c)
			if M['m00'] != 0:
				center_x, center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			else:
				center_x, center_y = 0, 0
			shape, approximation = self.classify(c)

			# Draw and label contours
			if shape == "rectangle":
				cv2.drawContours(mask_image, [approximation], -1, (0, 255, 0), 2)
				cv2.putText(mask_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
				cv2.drawContours(cv_image, [approximation], -1, (0, 255, 0), 2)
				cv2.putText(cv_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
			
		# Show the resultant images
		# cv2.namedWindow('camera_mask_Feed')
		# cv2.namedWindow('camera_cv_Feed')
		# cv2.imshow('camera_mask_Feed', mask_image)
		# cv2.imshow('camera_cv_Feed', cv_image)
		cv2.namedWindow('camera_cyan')
		cv2.imshow('camera_cyan', cyan_mask)
		cv2.namedWindow('camera_red')
		cv2.imshow('camera_red', red_mask)
		cv2.namedWindow('camera_yellow')
		cv2.imshow('camera_yellow', yellow_mask)
		cv2.namedWindow('camera_magenta')
		cv2.imshow('camera_magenta', magenta_mask)
		cv2.waitKey(3)
	
	def classify(self, contour):

		# Approximate shape of contour
		shape = "unknown"
		perimeter = cv2.arcLength(contour, True)
		approximation = cv2.approxPolyDP(contour, 0.06*perimeter, True)
		
		# Identify contour shape based on approximation
		if len(approximation) == 4:
			shape = "rectangle"
		else:
			shape = "not rectangle"
		
		# Return identified shapes
		return shape, approximation
		
def main(args):
    # Instantiate class
	fD = frameDetection()

    # And rospy.init the entire node
	rospy.init_node('frame_detection', anonymous=True)

    # Ensure that node continues running with rospy.spin()
    # Wrap rospy.spin() in exception handler for KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Closing Program")

    # Destroy image windows before closing node
	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
