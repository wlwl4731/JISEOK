#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('chess.png')

rows,cols = img.shape[0:2]

# 변환전 4점의 좌표
pts1 = np.float32([[20,20],[20,280],[380,20],[380,280]])
# 변환후 4점의 좌표
pts2 = np.float32([[100,20],[20,280],[300,20],[380,280]])

# 4개 점의 위치에 다른 색으로 원 그리기
cv2.circle(img, (20,20), 20, (255,0,0),-1)
cv2.circle(img, (20,280), 20, (0,255,0),-1)
cv2.circle(img, (380,20), 20, (0,0,255),-1)
cv2.circle(img, (380,280), 20, (0,255,255),-1)

# 변환행렬 계산
M = cv2.getPerspectiveTransform(pts1, pts2)
print M

# 이미지 변환
dst = cv2.warpPerspective(img, M, (cols,rows))

plt.subplot(121),plt.imshow(img),plt.title('image')
plt.subplot(122),plt.imshow(dst),plt.title('Perspective')
plt.show()
