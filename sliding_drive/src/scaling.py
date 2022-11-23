#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread('girl.png')
height, width = img.shape[0:2]

# 축소변환 행렬 생성
m_small = np.float32([[0.5, 0, 0],
                      [0, 0.5, 0]])  

# 확대변환 행렬 생성
m_big = np.float32([[2, 0, 0],
                    [0, 2, 0]])  

# 보간법 없이 축소
dst1 = cv2.warpAffine(img, m_small, (int(height*0.5), int(width*0.5)))

# 보간법 적용한 축소
dst2 = cv2.warpAffine(img, m_small, (int(height*0.5), int(width*0.5)), \
                        None, cv2.INTER_AREA)

# 보간법 없이 확대
dst3 = cv2.warpAffine(img, m_big, (int(height*2), int(width*2)))

# 보간법 적용한 확대
dst4 = cv2.warpAffine(img, m_big, (int(height*2), int(width*2)), \
                        None, cv2.INTER_CUBIC)

cv2.imshow("original", img)
cv2.imshow("small", dst1)
cv2.imshow("small INTER_AREA", dst2)
cv2.imshow("big", dst3)
cv2.imshow("big INTER_CUBIC", dst4)
cv2.waitKey(0)
cv2.destroyAllWindows()
