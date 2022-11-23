#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread('girl.png')
height, width = img.shape[0:2]

# 크기 지정으로 축소
dst1 = cv2.resize(img, (int(width*0.5), int(height*0.5)), \
                         interpolation=cv2.INTER_AREA)

# 배율 지정으로 확대, 가로:0.5배, 세로:1.5배
dst2 = cv2.resize(img, None,  None, 0.5, 1.5, cv2.INTER_CUBIC)



cv2.imshow("original", img)
cv2.imshow("small", dst1)
cv2.imshow("big", dst2)
cv2.waitKey(0)
cv2.destroyAllWindows()

