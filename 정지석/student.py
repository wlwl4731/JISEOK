#!/usr/bin/env python #파이썬 인터프리터 사용

import rospy #rospy 라이브러리 사용
from std_msgs.msg import String #std_msgs.msg모듈에서 String관련 부분만 사용

def callback(msg): #'callback'이라는 함수를 정의
    print msg.data #msg.data를 화면에 출력
    
rospy.init_node('student') #'student'라는 이름의 노드 생성

sub = rospy.Subscriber('my_topic', String, callback) #만든 노드는 토픽을 받는 subscriber임을 선언, 받고자 하는 토픽의 이름은 'my_topic'이며 그 안에 담긴 데이터는 string타입이다. 토픽이 도착할 때마다 'callback'함수 실행할 것을 ROS시스템에 요청

rospy.spin() #ROS 노드가 shutdown될 때까지 block하는 함수
