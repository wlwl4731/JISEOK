#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('chess.png')

rows,cols = img.shape[0:2]


# 변환전 3개 점 좌표
pts1 = np.float32([[50,50],[200,50],[50,200]])
# 변환후 3개 점 좌표
pts2 = np.float32([[10,100],[200,50],[100,250]])

# 기존점이 새로운 점으로 이동시킬 때 필요한 행렬 찾기
M = cv2.getAffineTransform(pts1,pts2)
print M

# 이미지 변환
dst = cv2.warpAffine(img,M,(cols,rows))

plt.subplot(121),plt.imshow(img),plt.title('Input')
plt.subplot(122),plt.imshow(dst),plt.title('Output')
plt.show()

