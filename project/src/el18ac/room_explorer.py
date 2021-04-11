#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():

	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	desired_velocity = Twist()
	desired_velocity.linear.x = 0.5 #head straight up
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #90deg clock
	desired_velocity.angular.z = -0.7
	for i in range(25):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight left
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #90deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(25):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight down
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight down
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #90deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(25):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight right
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight right
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #90deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(25):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight up
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight up
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg clock
	desired_velocity.angular.z = -0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #180deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(48):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #90deg anti-clock
	desired_velocity.angular.z = 0.7
	for i in range(25):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0.5 #straight left
	desired_velocity.angular.z = 0
	for i in range(30):
		pub.publish(desired_velocity)
		rate.sleep()
	desired_velocity.linear.x = 0 #stop
	desired_velocity.angular.z = 0
	for i in range(5):
		pub.publish(desired_velocity)
		rate.sleep()
if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
