#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import time
import message_filters
import tf
import math

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from cv_bridge import CvBridge, CvBridgeError

class findCluedo():
    def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        self.velocity = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        
        # Initiate global variables
        self.desired_velocity = Twist()

        # Initialise standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.forward = 0.3
        self.backward = -0.3
        self.left = 0.1
        self.right = -0.1
        self.stop = 0
        
        self.img_shape = None

        # Initial state
        self.current_state = 0
        
        # Define state flags
        self.frame_detected = False
        self.point_reached = False
        self.card_detected = False
        
        # Initialise any flags that signal a colour has been detected in view (default to false)
        self.red_detected = False
        self.yellow_detected = False
        self.cyan_detected = False
        self.magenta_detected = False
        
        # Initialise self.sensitivity colour detection value (10 should be enough)
        self.red_sensitivity = 10
        self.yellow_sensitivity = 12
        self.cyan_sensitivity = 16
        self.magenta_sensitivity = 16
        
        # 3D point from camera
        self.point_xyz = []

        # Initialise a CvBridge()
        self.bridge = CvBridge()
        
        self.srv = rospy.Service('room_search', Trigger, self.srvCallback)
        self.cluedo_identifier_client = rospy.ServiceProxy('cluedo_identify', Trigger)
        
        # Set up a subscribers to the image topic and the depth point cloud
        self.image_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)
        self.pc_sub = message_filters.Subscriber('/camera/depth/points', PointCloud2)

        # Synchronize depth point cloud and raw images
        self.synchroniser = message_filters.TimeSynchronizer([self.image_sub, self.pc_sub], 10)
        self.synchroniser.registerCallback(self.tsCallback)

        return
        
    def tsCallback(self, img_msg, pc_msg):
        # Read cv_image from image message and cv bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as error:
            print(error)

        self.img_shape = cv_image.shape

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

        # Generalize mask using morphology
        kernel_open = np.ones((15,15), np.uint8)
        kernel_close = np.ones((30,30), np.uint8)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, kernel_close)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, kernel_open)

        frame_mask = colour_mask
        
        # Apply mask to original image using cv2.bitwise_and() method
        mask_image = cv2.bitwise_and(cv_image,cv_image, mask=frame_mask)
        
        # Detect contours within image
        contours = cv2.findContours(frame_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours[0]) > 0:
            max_contour = max(contours[0], key=cv2.contourArea)
            if cv2.contourArea(max_contour) > 100:
                # calculate center of contour and classified name
                M = cv2.moments(max_contour)
                if M['m00'] != 0:
                    center_x, center_y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                else:
                    center_x, center_y = 0, 0
                shape, approximation = self.classify(max_contour)

                # Draw and label contours in mask image and in actual image
                if shape == "target":
                    cv2.putText(mask_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                    cv2.drawContours(mask_image, [approximation], -1, (0, 255, 0), 2)
                    cv2.drawContours(cv_image, [approximation], -1, (0, 255, 0), 2)
                    cv2.putText(cv_image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                    # Calculate position of center point of contour in 3D space (from the canera)
                    point_cloud = point_cloud2.read_points(pc_msg, skip_nans=False, field_names=("x", "y", "z"), uvs=[[center_x, center_y]])
                    self.point_xyz = next(point_cloud) # point is in Camera Optical frame, i.e. point[0] x-down,  point[1] y-left, point[2] z-forward
                    
                    self.frame_point = [center_x, center_y]
                    self.frame_detected = not math.isnan(self.point_xyz[2])
                else:
                    self.frame_detected = False
            else:
                self.frame_detected = False
        else:
            self.frame_detected = False
        
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(3)

        return
        
    def classify(self, contour):

        # Approximate shape of contour
        shape = "unknown"
        perimeter = cv2.arcLength(contour, True)
        approximation = cv2.approxPolyDP(contour, 0.06*perimeter, True)
        
        # Identify contour shape based on approximation
        # If its a rectangle (4 edges), its a target
        if len(approximation) == 4:
            shape = "target"
        else:
            shape = "not target"
        
        # Return identified shapes
        return shape, approximation  
    
    def srvCallback(self, req):
        self.start = True
        
        self.run_robot()
        
        self.start = False
        
        res = TriggerResponse()
        res.success = self.card_detected
        return res
    
    def run_robot(self):
        
        start_t = time.time()
        while self.start and (time.time() - start_t) <= 240:
            # Finite state machine
            
            fsm = {
                0: {
                    "output": self.search_output,
                    "condition": self.frame_detected,
                    "next_state": [0,1]
                    },
                1: {
                    "output": self.approach_output,
                    "condition": self.point_reached and self.frame_detected,
                    "next_state": [1,2]
                    },
                2: {
                    "output": self.wait_output,
                    "condition": self.card_detected,
                    "next_state": [0,3]
                    },
                3: {
                    "output": self.exit_output,
                    "condition": False,
                    "next_state": [3,3]
                    }
            }
            state = fsm.get(self.current_state)
            #print(state)
            output = state.get("output")
            condition = state.get("condition")
            #print("Before:", self.point_reached)
            #print(condition)
            self.current_state = state.get("next_state")[condition]
            
            output()
            #print("After:", self.point_reached)
        
        return
        
    def search_output(self):
        self.desired_velocity.angular.z = self.left
        start_time = time.time()
        # Rotate for 2 seconds
        self.velocity.publish(self.desired_velocity)
        
        print("Searching.", "Frame Detected: ", self.frame_detected)
        return
    
    def approach_output(self):
        """self.desired_velocity.angular.z = self.stop
        self.velocity.publish(self.desired_velocity)"""
        reached = [False, False] 
        if self.point_xyz[2] < 0.9:
            # Too close to object, need to move backwards
            self.desired_velocity.linear.x = self.backward
            reached[0] = False
        elif self.point_xyz[2] > 1.1:
            # Too far away from object, need to move forwards
            self.desired_velocity.linear.x = self.forward
            reached[0] = False
        else:
            self.desired_velocity.linear.x = self.stop
            reached[0] = True
        
        #Alternatively cx,cy can be used to position and rotate towards the object
        
        _, width, _ = self.img_shape
        centre = (width/2) - 1
        
        if math.fabs(self.frame_point[0] - centre) > 50:
            
            self.desired_velocity.angular.z = self.left if centre > self.frame_point[0] else self.right
            reached[1] = False
        else:
            self.desired_velocity.angular.z = self.stop
            reached[1] = True
        
        print(self.point_xyz[1])
        print(self.point_xyz[2])
        print(reached[0])
        print(reached[1])
        
        # Change flag if point is reached
        if reached == [True, True]:
            self.point_reached = True
        else:
            self.point_reached = False
            
        self.velocity.publish(self.desired_velocity)
        
        return
        
    def wait_output(self):
        self.desired_velocity.linear.x = self.stop
        self.desired_velocity.angular.z = self.stop
        self.velocity.publish(self.desired_velocity)
        self.card_detected = True
        #req = TriggerRequest()
        #res = self.cluedo_identifier_client(req)
        #self.card_detected = res.success
        print("Waiting.", "Card Detected: ", self.card_detected)
        return
        
    def exit_output(self):
        print("Exiting.")
        self.start = False
        return
        
def main(args):
    # Instantiate class
    fC = findCluedo()

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
