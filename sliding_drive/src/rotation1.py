#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread('girl.png')

rows,cols = img.shape[0:2]

# 45도, 90도 회전 각도를 라디안값으로 변경
d45 = 45.0 * np.pi / 180    
d90 = 90.0 * np.pi / 180    

# 회전하면 이미지 왼쪽 모서리를 축으로 회전해서 상하좌우 이동시켜줘야 
# 화면에 이미지 나온다.

# 45도 회전 행렬
m45 = np.float32( [[ np.cos(d45), -1* np.sin(d45), rows//2],
                    [np.sin(d45), np.cos(d45), -1*cols//4]])
# 90도 회전 행렬
m90 = np.float32( [[ np.cos(d90), -1* np.sin(d90), rows],
                    [np.sin(d90), np.cos(d90), 0]])

# 이미지 회전
r45 = cv2.warpAffine(img,m45,(cols,rows))
r90 = cv2.warpAffine(img,m90,(cols,rows))


cv2.imshow("origin", img)
cv2.imshow("45", r45)
cv2.imshow("90", r90)
cv2.waitKey(0)
cv2.destroyAllWindows()
