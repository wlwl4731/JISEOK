#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('my_node', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

msg = Twist()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
	for i in range(0,8):
		msg.linear.x = 1.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = 1.8
    		pub.publish(msg)
    		rate.sleep()

	for j in range(0,8):
		msg.linear.x = 1.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = -1.8
		pub.publish(msg)
    		rate.sleep()
