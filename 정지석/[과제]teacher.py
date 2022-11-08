#!/usr/bin/env python #파이썬 인터프리터 사용

import rospy #rospy 라이브러리 사용
from std_msgs.msg import String #std_msgs.msg모듈에서 String관련 부분만 사용

rospy.init_node('teacher') #'teacher'라는 이름의 노드 생성

pub = rospy.Publisher('my_topic', String) #'my_topic'이라는 이름의 토픽을 발행하겠다고 ROS시스템에 자신을 publish로 등록하는 부분, 그리고 'my_topic'이라는 토픽에 담는 메시지의 데이터 타입은 string

rate = rospy.Rate(2) #1초에 2반 loop를 반복할 수 있도록 rate라는 객체 생성, 타임슬롯을 0.5초 할당하겠다.

while not rospy.is_shutdown(): #rospy.is_shutdown()이 True가 될 때까지(파이썬 코드의 실행이 멈출때) while loop안에 있는 실행문들을 반복
    pub.publish('call me please') #pub은 'my_topic'이라는 이름의 토픽을 발행하기 위해 만든 publisher인스턴스, publish는 'my_topic'이라는 토픽에 'call me please'라는 데이터를 담아서 발행하는 기능을 수행
    rate.sleep() #while 안에 있는 pub.publish 코드가 0.5초에 한번씩 동작하게 한다.
