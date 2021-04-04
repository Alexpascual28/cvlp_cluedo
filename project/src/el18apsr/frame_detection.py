#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import message_filters

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Trigger, TriggerRequest
from cv_bridge import CvBridge, CvBridgeError

class frameDetection():
    def __init__(self):
        # Initialise self.sensitivity colour detection value (10 should be enough)
        self.red_sensitivity = 10
        self.yellow_sensitivity = 12
        self.cyan_sensitivity = 16
        self.magenta_sensitivity = 16

        # Initialise a CvBridge()
        self.bridge = CvBridge()

        # Set up a subscriber to the image topic you wish to use
        self.image_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)
        self.pc_sub = message_filters.Subscriber('/camera/depth/points', PointCloud2)

        self.synchroniser = message_filters.TimeSynchronizer([self.image_sub, self.pc_sub], 10)
        self.synchroniser.registerCallback(self.tsCallback)

        return
    
    def tsCallback(self, img_msg, pc_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as error:
            print(error)

        point_cloud = point_cloud2.read_points(pc_msg, skip_nans=False, field_names=("x", "y", "z"))
        xyz = point_cloud

        # Set upper and lower bounds for HSV colours
        # Red (Scarlet) = 0
        hsv_red_lower = np.array([0-self.red_sensitivity, 100, 100])
        hsv_red_upper = np.array([0+self.red_sensitivity, 255, 255])
        # Yellow (Mustard) = 30
        hsv_yellow_lower = np.array([30-self.yellow_sensitivity, 100, 100])
        hsv_yellow_upper = np.array([30+self.yellow_sensitivity, 255, 255])
        # Cyan (Peacock) = 90
        hsv_cyan_lower = np.array([90-self.cyan_sensitivity, 100, 100])
        hsv_cyan_upper = np.array([90+self.cyan_sensitivity, 255, 255])
        # Magenta (Plum) = 150
        hsv_magenta_lower = np.array([150-self.magenta_sensitivity, 30, 30])
        hsv_magenta_upper = np.array([150+self.magenta_sensitivity, 255, 255])
        
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
        kernel_open = np.ones((15,15), np.uint8)
        kernel_close = np.ones((30,30), np.uint8)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, kernel_close)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, kernel_open)

        frame_mask = colour_mask
        
        # Apply mask to original image using cv2.bitwise_and() method
        mask_image = cv2.bitwise_and(cv_image,cv_image, mask=frame_mask)
        
        # Detect contours within image
        contours = cv2.findContours(frame_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # queue of points to go to
        array = list()
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
                cv2.putText(mask_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                cv2.drawContours(mask_image, [approximation], -1, (0, 255, 0), 2)
                cv2.drawContours(cv_image, [approximation], -1, (0, 255, 0), 2)
                cv2.putText(cv_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                # idx = (center_x*pc_msg.row_step) + (center_y*pc_msg.point_step)
                # point_xyz = struct.unpack_from('fff', pc_msg.data, offset=idx)
                # array.append(point_xyz)

                # check point here

        # or check points here
        
        cv2.imshow("rgb_mask", cv2.bitwise_or(ry_mask, cm_mask))
        cv2.imshow("frame_mask", frame_mask)
        cv2.imshow("camera_mask_feed", mask_image)
        #cv2.namedWindow('camera_cyan')
        #cv2.imshow('camera_cyan', cyan_mask)
        #cv2.namedWindow('camera_red')
        #cv2.imshow('camera_red', red_mask)
        #cv2.namedWindow('camera_yellow')
        #cv2.imshow('camera_yellow', yellow_mask)
        #cv2.namedWindow('camera_magenta')
        #cv2.imshow('camera_magenta', magenta_mask)
        cv2.waitKey(3)

        return
    
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
