#!/usr/bin/env python

from __future__ import division

import rospy
import cv2
import sys
import os
import time
import numpy as np

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RoomIdentifier:
    def __init__(self):
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.imgCallback)
        self.vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        self.srv = rospy.Service("room_search", Trigger, self.srvCallback)

        self.cv_bridge = CvBridge()
        
        self.script_path = os.path.dirname(os.path.realpath(sys.argv[0]))

        self.start = False
        self.img_saved = False
        
        thresh = 20

        self.red_lower = np.array([0 - thresh, 100, 100])
        self.red_upper = np.array([0 + thresh, 255, 255])

        self.green_lower = np.array([60 - thresh, 100, 60])
        self.green_upper = np.array([60 + thresh, 255, 255])

    def imgCallback(self, img_msg):
        frame = None
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            print("Caught CvBridge error!")
            print(e)
        
        if frame is not None and self.start:
            self.detectColour(frame)
    
    def detectColour(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # not needed
        # red_msk = cv2.inRange(hsv_img, self.red_lower, self.red_upper)

        green_msk = cv2.inRange(hsv_img, self.green_lower, self.green_upper)
        
        cnts, _ = cv2.findContours(green_msk, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # cv2.imshow("Detection", frame)
        # cv2.imshow("mask", green_msk)
        # cv2.waitKey(1)

        if len(cnts) == 0:
            return False
        
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 1000 and cv2.contourArea(c) < 500:
            return False
        
        # obtain centre point and draw a circle around the detection
        M = cv2.moments(c)
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        (x, y), radius = cv2.minEnclosingCircle(c)
        centre = (int(x), int(y))
        cv2.circle(frame, centre, int(radius), (255, 0, 255), 1)

        # ensure circle is close to the middle of the frame by rotating the turtlebot
        _, width, _ = frame.shape
        mid_x = width/2
        vel_msg = Twist()
        if abs(mid_x - cx) > 30:
            print("[ROOM_IDENTIFIER]: Turning to centre target!")
            vel_msg.angular.z = 0.05 if mid_x > cx else -0.05
            self.vel_pub.publish(vel_msg)
        elif not self.img_saved:
            # if circle centred, save image
            # print(abs(mid_x - cx))
            self.saveImage(frame)

    def saveImage(self, frame):
        name = os.path.join(self.script_path, 'green_circle.png')
        cv2.imwrite(name, frame)
        print("[ROOM_IDENTIFIER]: Saved img to {}".format(name))
        self.img_saved = True
    
    def srvCallback(self, req):
        t_start = time.time()
        self.start = True
        self.img_saved = False

        # process camera feed for 10 seconds max
        while time.time() - t_start <= 10:
            if self.img_saved:
                break
        
        self.start = False
        res = TriggerResponse()
        res.success = self.img_saved
        print("success: {}".format(res.success))
        return res

def main(args):
    rospy.init_node("room_identification_node", anonymous=True)

    identifier = RoomIdentifier()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    print("[ROOM_IDENTIFIER]: Node shut down!")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
