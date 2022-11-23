#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread('girl.png')

rows,cols = img.shape[0:2]


# 회전변환 행렬 안구해도 되서 더 편하다
# cv2.getRotationMatrix2D(회전축, 회전각도(Degree)-반시계방향, 확대축소)
m45 = cv2.getRotationMatrix2D((cols/2,rows/2),45,0.5) 
# 구해진 2x3 변환행렬값 출력
print m45

m90 = cv2.getRotationMatrix2D((cols/2,rows/2),90,1.5) 
print m90

r45 = cv2.warpAffine(img, m45,(cols, rows))
r90 = cv2.warpAffine(img, m90,(cols, rows))


cv2.imshow("origin", img)
cv2.imshow("45", r45)
cv2.imshow("90", r90)
cv2.waitKey(0)
cv2.destroyAllWindows()

