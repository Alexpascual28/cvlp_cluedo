#!/usr/bin/env python

from __future__ import division

import rospy
import cv2
import sys
import os
import rospkg

from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imgCallback)
        self.pub = rospy.Publisher("/cluedo_img", Image, queue_size=1)
        self.srv = rospy.Service("save_image", Empty, self.saveImgCallback)
        self.bridge = CvBridge()
        self.img_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.frame = None
        self.msg = Image()
    
    def imgCallback(self, img_msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            self.msg = img_msg
        except CvBridgeError as e:
            print("Caught CvBridge error!")
            print(e)
        
        cv2.imshow("frame", self.frame)
        cv2.waitKey(1)

    def saveImgCallback(self, req):
        if self.frame is not None:
            name = os.path.join(self.img_path, 'cluedo_character.png')
            cv2.imwrite(name, self.frame)
            self.pub.publish(self.msg)
            print("[IMG_SAVER]: Saved img to {}".format(name))
        return EmptyResponse()

def main(args):
    rospy.init_node("img_saver_node", anonymous=True)
    
    saver = ImageSaver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
