#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

# 영상 사이즈는 가로세로 640x480
Width = 640
Height = 480
# ROI영역 = 세로 480크기에서 420~460, 40픽셀 만큼만 잘라서 사용
Offset = 420
Gap = 40

# draw lines
def draw_lines(img, lines): # lines = 허프변환 함수로 구한 선분들
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0] # 선분이니까 시작점, 끝점 좌표
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2) # 선분은 ROI에서 찾은거다 그래서 좌표는 y에 +Offset해주는거다
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0): # lpos=왼쪽 차선 좌표, rpos=오른쪽 차선 좌표
    center = (lpos + rpos) / 2
	
	# lpos에 사각형
    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
	# rpos에 사각형
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
	# 차선 중간에 사각형
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
	# 영상 한가운데 사각형
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0: # 시작점x - 끝점x = 0 =>> 수직선
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1) # 기울기
        
		# 기울기의 절댓값이 10보다 작은것들만 저장
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
	# 허브변환 함수로 검출한 선불들의 기울기를 비교하여 왼쪽 차선과 오른쪽 차선을 구분
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line
		
		# OpenCV 좌표계에서는 아래방향으로 y까 증가하므로 기울기 부호가 반대다
		# 기울기가 음수고 화면 왼쪽에 있을때 왼쪽차선으로 간주
        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
		# 기울기가 양수고 화면 오른쪽에 있을때 오른쪽 차선으로 간주
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
# 선분들의 평균 m(기울기), b(y절편)값 구하기
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
	# 구해진 선분 없으면 리턴 0, 0
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = float(x_sum) / float(size * 2)
    y_avg = float(y_sum) / float(size * 2)

    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
# 차선 찾기
def get_line_pos(lines, left=False, right=False):
    global Width, Height
    global Offset, Gap
	
    m, b = get_line_params(lines)
    
    # ??x1, x2 = 0, 0
	# lines=0일때, 선분 못찾았을때
    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2 # ROI좌표에서 y값 중앙으로 고정
        pos = (y - b) / m # x좌표

		# ROI영역이 아닌 전체 영상에 차선 그려주기
		# ROI좌표계에서 얻어진 좌표를 전체 영상 좌표로 바꿔줘야된다.
        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0, 0), 3)
	
    return x1, x2, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge (외곽선 따기)
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP (ROI영역에서 선분 찾기)
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
	# 찾아낸 직선이 하나도 없으면 차선의 위치는 0, 640 리턴
    if all_lines is None:
        return (0, 640), frame

	# 왼쪽 차선과, 오른쪽 차선 분류해서 따로 저장
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
	# 왼쪽, 오른쪽 찾은 선분들 평균내서 대표 선분 찾아서 차선 찾기
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)
    
    # ? lx1, lx2, lpos = get_line_pos(left_lines, left=True)
    # ? rx1, rx2, rpos = get_line_pos(right_lines, right=True)

    # ? frame = cv2.line(frame, (int(lx1), Height), (int(lx2), (Height/2)), (255, 0,0), 3)
    # ? frame = cv2.line(frame, (int(rx1), Height), (int(rx2), (Height/2)), (255, 0,0), 3)

    # draw lines
	# 찾은 선분들 모두, 중앙선 그려주기
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    return (lpos, rpos), frame

# 화면 중앙에 핸들이랑 화살표 그림 넣기
def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

	# 이미지 읽어들이기
    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

	# 이미지 크기 영사 크기에 맞춰서 축소하기 위해 크기 계산
    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

	# 이미지 steer_angle에 비례하여 회전
    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 2.5, 0.7)    
	# 이미지 크기를 영상에 맞춤
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

	# 전체 그림 위에 이미지 오버레이
    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

	# image에는 원본사진+검출차선+평균차선+차선위치, 화면중앙 사각형+핸들그림,화살표표시
    cv2.imshow('steer', image)


def start():

    global image, Width, Height

	# 동영상 파일 열기
    cap = cv2.VideoCapture('hough_track.avi')

    while not rospy.is_shutdown():

		# 동영상 파일에서 이미지 한장 읽기
        ret, image = cap.read()
        time.sleep(0.03)

        pos, frame = process_image(image)
        
		# 조향각 결정
        center = (pos[0] + pos[1]) / 2        
        angle = 320 - center

        steer_angle = angle * 0.4
        draw_steer(frame, steer_angle)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()
