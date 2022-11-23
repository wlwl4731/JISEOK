#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Publisher의 메시지를 받아 출력

import rospy
from std_msgs.msg import Int32MultiArray

# 받은 토픽에서 .data만 출력
def callback(msg):
	print(msg.data)

rospy.init_node('ultra4_sub')
sub = rospy.Subscriber('ultra4', Int32MultiArray, callback)

rospy.spin()
