#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 초음파센서가 보낸 거리정보를 토픽에 담아 Publishing

import serial, time, rospy
from std_msgs.msg import Int32MultiArray, Int32

FRONT = [0, 0, 0, 0]

# 아두이노가 연결된 포트 지정
ser_front = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 9600,
	)

# 시리얼 데이터를 한번에 문자열로 받아옴
def read_sensor():
	sensor_date = ser_front.readline()
	ser_front.flushInput()
	ser_front.flushOutput()
	FRONT = read_Sdate(sensor_data)
	msg.data = FRONT

# 문자열에서 숫자 4개 추출
def read_Sdate(s):
	s = s.replace(" ", "")
	s_data = s.split("mm")
	s_data.remove('\r\n')
	s_data = list(map(int, s_data))
	return s_data

# 노드 생성, 토픽 발행 준비
if __name__ == '__main__':
	rospy.init_node('ultra4_pub', anonymous=False)
	pub = rospy.Publisher('ultra4', Int32MultiArray, queue_size=1)

	msg = Int32MultiArray()
	while not rospy.is_shutdown():
		read_sensor()
		pub.publish(msg)
		time.sleep(0.2)

	ser_front.close() # 시리얼포트 닫고 정리함
