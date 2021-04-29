#!/usr/bin/env python

from __future__ import division

import rospy
import sys
import os
import numpy as np
import yaml
import actionlib

from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger, TriggerRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class TurtlebotControl:
    def __init__(self):
        self.room_detected = False
        self.cluedo_detected = False

        self._load_inputs()
        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.room_identify_client = rospy.ServiceProxy('/room_identify', Trigger)
        self.room_search_client = rospy.ServiceProxy('/room_search', Trigger)
        
        self.cluedo_room_centre = None

        print("Waiting for move_base server")
        self.move_base.wait_for_server()
        print("Node initialised!")
        return
    
    def odomCallback(self, msg):
        self.robot_pose = msg.pose.pose
        return
    
    def run(self):
        if not self.room_detected:
            self.searchForRoom()
        elif not self.cluedo_detected:
            self.searchForCluedo()

    # same as lab4 file
    def sendCmd(self, pos, quat):
        self.goal_sent = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()
        
        self.goal_sent = False
        return

    def searchForRoom(self):
        room_1_searched = False
        room_2_searched = False

        current_room = 0

        theta = 0.0
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        while not self.room_detected:
            if not room_1_searched:
                position = {'x': self.room1_ent[0], 'y' : self.room1_ent[1]}
                print("Moving to room 1")
                self.sendCmd(position, quaternion)
                current_room = 1
                room_1_searched = True
            elif not room_2_searched:
                position = {'x': self.room2_ent[0], 'y' : self.room2_ent[1]}
                print("Moing to room 2")
                self.sendCmd(position, quaternion)
                current_room = 2
                room_2_searched = True
            else:
                room_1_searched = False
                room_2_searched = False
                current_room = 0
                continue

            req = TriggerRequest()
            res = self.room_identify_client(req)

            if res.success:
                self.room_detected = True
                print("Entering room {}".format(current_room))
                if current_room == 1:
                    position = {'x': self.room1_cent[0], 'y' : self.room1_cent[1]}
                    self.cluedo_room_centre = self.room1_cent
                    self.sendCmd(position, quaternion)
                else:
                    position = {'x': self.room2_cent[0], 'y' : self.room2_cent[1]}
                    self.cluedo_room_centre = self.room2_cent
                    self.sendCmd(position, quaternion)
        return

    def searchForCluedo(self):
        print("Sending request to room_search service")

        rate = rospy.Rate(10)
        pos = {'x': self.cluedo_room_centre[0], 'y': self.cluedo_room_centre[1]}
        while not self.cluedo_detected:
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : self.robot_pose.orientation.z, 'r4' : self.robot_pose.orientation.w}
            self.sendCmd(pos, quaternion)
            req = TriggerRequest()
            res = self.room_search_client(req)
            
            self.cluedo_detected = res.success
            rate.sleep()
        
        return

    def _load_inputs(self):
        self.script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.input_file = os.path.join(self.script_path, '../../example/input_points.yaml')

        with open(self.input_file) as f:
            data = yaml.safe_load(f)
            self.room1_ent = data['room1_entrance_xy']
            self.room2_ent = data['room2_entrance_xy']
            self.room1_cent = data['room1_centre_xy']
            self.room2_cent = data['room2_centre_xy']
        return

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        return

def main(args):
    rospy.init_node("turtlebot_control_node", anonymous=True)
    rate = rospy.Rate(10)

    controller = TurtlebotControl()
    while not rospy.is_shutdown():
        controller.run()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
