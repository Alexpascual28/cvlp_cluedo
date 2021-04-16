#!/usr/bin/env python

from __future__ import division

import rospy
import cv2
import sys
import os

from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CluedoIdentifier:
    def __init__(self):
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.imgCallback)
        self.srv = rospy.Service("cluedo_identify", Trigger, self.srvCallback)
        
        self.cv_bridge = CvBridge()
        self.orb = cv2.ORB()

        self.frame = None
        self.threshold = 0.75 # for feature matching and template matching
        self.min_matches = 20 # for feature matching
        
        self.success = False

        # this path will be used to correctly locate the cluedo images and to save the
        # detection result in the correct directory
        self.script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.loadTemplates()

        # for debug purposes
        # self.identifyTemplate()
        # self.featureDetector()
        
        return
    
    def srvCallback(self, req):        
        self.success = False
        self.featureDetector()
        
        res = TriggerResponse()
        res.success = self.success
        return res
     
    def imgCallback(self, img_msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            print("Caught CvBridge error!")
            print(e)

        return
    
    def featureDetector(self):
        # feature detection and matching using ORB
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(frame, None)

        # brute force matching
        bf_matcher = cv2.BFMatcher()

        idx_params = dict(algorithm=6, table_number=12, key_size=20, multi_probe_level=2)
        search_params = dict(checks=50)

        flann = cv2.FlannBasedMatcher(idx_params, search_params)
        
        best_result = (None, 0)    
        for template in self.templates:
            good = []
            # matches = bf_matcher.knnMatch(des, template[4], k=2)

            # for i, j in matches:
            #     if i.distance <= self.threshold*j.distance:
            #         good.append([i])

            matches = flann.knnMatch(des, template[4], k=2)
            for (m_n) in (matches):
                if len(m_n) != 2:
                    continue
                m, n = m_n
                if m.distance < 0.75*n.distance:
                    good.append([m])

            if len(good) > best_result[1] and len(good) >= self.min_matches:
                best_result = (template[0], len(good))
            
            print("Num matches for {}: {}".format(template[0], len(good)))
        
        if best_result[1] != 0:
            print("Best match: {} with {} matches".format(best_result[0], best_result[1]))
            self.saveDetection(best_result[0])
            self.success = True
        else:
            rospy.logerr("Failed to identify any character in the frame!")
            self.success = False
        
        return
    
    def identifyTemplate(self):
        # template matching method
        # implementation is somewhat scaling invariant but still can fail
        # it is NOT rotation invariant

        img_path = os.path.join(self.script_path, 'cluedo_character.png')
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        cv2.imshow('img', img)
        # print(img.shape)

        best_result = None
        for template in self.templates:
            w, h = template[1].shape[::-1]
            print("Testing: {}".format(template[0]))

            for i in reversed(range(40, 110, 10)):
                print(i)
                resized = cv2.resize(gray, (0,0), fx=i/100, fy=i/100)
                
                if resized.shape[0] < w or resized.shape[0] < h:
                    break
                
                result = cv2.matchTemplate(resized, template[1], cv2.TM_CCOEFF)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                ratio = float(100/i)
                print(ratio)

                if max_val >= self.threshold:
                    # cv2.rectangle(resized, max_loc, ((max_loc[0]+w), (max_loc[1]+h)), 255, 2)
                    # cv2.imshow("resized", resized)
                    # cv2.waitKey(0)

                    if best_result is None or max_val > best_result[0]:
                        best_result = (max_val, max_loc, ratio, template[0])
                        self.success = True
        
        val, loc, ratio, name = best_result
        print("Found match with: {}".format(name))
        self.saveDetection(name)

        # draw rectangle around the detected frame
        # cv2.rectangle(img, (int(loc[0]*ratio), int(loc[1]*ratio)), (int(ratio*(loc[0]+w)), int(ratio*(loc[1]+h))), 255, 2)
        # cv2.imshow("result", result)
        # cv2.putText(img, name, (int(loc[0]*ratio), int(loc[1]*ratio)-10), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.75, (255, 255, 0), 2)
        # cv2.imshow("Detection", img)
        # cv2.waitKey(0)
        return

    # saves the detection result into a txt file
    def saveDetection(self, detection):
        img = os.path.join(self.script_path, 'cluedo_character.png')
        cv2.imwrite(img, self.frame)
        file = os.path.join(self.script_path, 'cluedo_character.txt')
        with open(file, "w") as f:
            f.write(detection+"\n")

    # save the cluedo images and their names in a list
    def loadTemplates(self):
        self.templates = list()
        
        path = os.path.join(self.script_path, '../../cluedo_images/')
        for file in os.listdir(path):
            img_path = os.path.join(path, file)
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            w, h = img.shape[::-1]
            
            w = int(w*0.10)
            h = int(h*0.10)

            kp, des = self.orb.detectAndCompute(img, None)
            # print(des)
            template = (os.path.basename(img_path), cv2.resize(img, (w, h)), img, kp, des)
            self.templates.append(template)

            # print("Found {} template".format(template[0]))
            # cv2.imshow(template[0], template[1])
            # print(template[0])
            # cv2.waitKey(0)

        if len(self.templates) != 4:
            string = ""
            for temp in self.templates:
                string += temp[0] + " "
            rospy.logerr("Did not find all four templates!\nOnly found {} templates\n{}".format(len(self.templates), string))
        else:
            print("[CLUEDO_IDENTIFIER]: Found {} templates to use".format(len(self.templates)))

        return

def main(args):
    rospy.init_node("cluedo_identification_node", anonymous=True)

    identifier = CluedoIdentifier()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    print("[CLUEDO_IDENTIFIER]: Node shut down!")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
