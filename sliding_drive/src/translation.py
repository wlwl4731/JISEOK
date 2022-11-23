#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread('girl.png')
rows,cols = img.shape[0:2] # 이미지의 크기정보 가져오기
dx, dy = 100, 50 # 평행이동할 픽셀 거리

# 변환행렬
mtrx = np.float32([[1, 0, dx],
                   [0, 1, dy]])  

# 단순 이동
dst = cv2.warpAffine(img, mtrx, (cols+dx, rows+dy))   

# 탈락된 외곽 픽셀(사진이 이동해서 새로생긴 공간, 빈공간)을 파랑색으로 보정
dst2 = cv2.warpAffine(img, mtrx, (cols+dx, rows+dy), None, \
                        cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, (255,0,0) )
# 탈락된 외곽 픽셀을 원본을 반사시켜서 보정
dst3 = cv2.warpAffine(img, mtrx, (cols+dx, rows+dy), None, \
                                cv2.INTER_LINEAR, cv2.BORDER_REFLECT)

cv2.imshow('original', img)
cv2.imshow('trans', dst)
cv2.imshow('BORDER_CONSTATNT', dst2)
cv2.imshow('BORDER_REFLECT', dst3)
cv2.waitKey(0)
cv2.destroyAllWindows()
