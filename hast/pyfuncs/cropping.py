#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
# call as python calibrate.py 20180209 004

import numpy as np
import cv2
import glob
import sys
import yaml

with open("cropping.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

y1 = 790
y2 = 835
x1 = 770
x2 = 815

# img = cv2.imread("lenna.png")
original_img = cv2.imread(cfg['path'] + cfg['original'])
lU_img = cv2.imread(cfg['path'] + cfg['yuv_U'])
lV_img = cv2.imread(cfg['path'] + cfg['yuv_V'])
bool_img = cv2.imread(cfg['path'] + cfg['bool'])
circle_img = cv2.imread(cfg['path'] + cfg['circle'])

original_crop_img = original_img[y1:y2, x1:x2]
lU_crop_img = lU_img[y1:y2, x1:x2]
lV_crop_img = lV_img[y1:y2, x1:x2]
bool_crop_img = bool_img[y1:y2, x1:x2]
circle_crop_img = circle_img[y1:y2, x1:x2]

# cv2.imshow("original_cropped", original_crop_img)
# cv2.waitKey(0)
# cv2.imshow("lU_cropped", lU_crop_img)
# cv2.waitKey(0)
# cv2.imshow("lV_cropped", lV_crop_img)
# cv2.waitKey(0)
# cv2.imshow("bool_cropped", bool_crop_img)
# cv2.waitKey(0)
# cv2.imshow("circle_cropped", circle_crop_img)
# cv2.waitKey(0)



cv2.imwrite(cfg['path'] + "cropped/01_original_cropped.png", original_crop_img)
cv2.imwrite(cfg['path'] + "cropped/02_lU_cropped.png", lU_crop_img)
cv2.imwrite(cfg['path'] + "cropped/03_lV_cropped.png", lV_crop_img)
cv2.imwrite(cfg['path'] + "cropped/04_bool_cropped.png", bool_crop_img)
cv2.imwrite(cfg['path'] + "cropped/05_circle_cropped.png", circle_crop_img)
# for section in cfg:
#     print(section)
# print(cfg['mysql'])
# print(cfg['other'])