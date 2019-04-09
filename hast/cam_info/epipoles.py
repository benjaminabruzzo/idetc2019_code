#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
import numpy as np
import cv2
import glob

# function for epipole analysis
from matplotlib import pyplot as plt
def drawlines(img0,lines,pts0,pts1):
	''' img0 - image on which we draw the epilines for the points in img0 lines - corresponding epilines ''' 
	r,c = img0.shape[:2]
	for L,pt0,pt1 in zip(lines,pts0,pts1):
		color = tuple(np.random.randint(0,255,3).tolist())
		x0,y0 = map(int, [0,  -L[2]/L[1] ])
		x1,y1 = map(int, [c, -(L[2]+L[0]*c)/L[1] ])
		cv2.line(img0, (x0,y0), (x1,y1), color,1)
		cv2.circle(img0,tuple(pt0),5,color,-1)
		# cv2.circle(img1,tuple(pt1),5,color,-1)
	return img0


def meanIntersect(imgshape,lines):
	# epipole = meanIntersect(img0.shape[:2],lines)
	i = 0
	L_minus1 = []
	intersects = []
	mean_intersect = [0,0]
	for L in lines:
		if i == 0:
			L_minus1 = L
		else:
			crossProduct = np.cross(L,L_minus1)
			xp_pt = crossProduct[:2]/crossProduct[2]
			xp_point = (xp_pt[0], xp_pt[1])
			mean_intersect[0]+=xp_pt[0]
			mean_intersect[1]+=xp_pt[1]
			intersects.append(xp_point)
		i+=1
	mean_intersect = [mean_intersect[0]/(i-1), mean_intersect[1]/(i-1)]
	return mean_intersect

# Define termination criteria = ( type, max_iter = 30 , epsilon = 0.001 )
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)

CAL_BLOCK_SIZE = 0.111 # meters

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = CAL_BLOCK_SIZE * np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
arrayOfCorners = [];

# images = glob.glob('*.png')
images = glob.glob('/home/benjamin/pycal/*.png')

i = 0
last_image = []
for current_image in images:
	if i == 0:
		last_image = current_image
	else:
		# print(last_image)
		# print(current_image)
		img1 = cv2.imread(last_image)
		img2 = cv2.imread(current_image)
		gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
		gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
		ret1, corners1 = cv2.findChessboardCorners(gray1, (8,6),None)
		ret2, corners2 = cv2.findChessboardCorners(gray2, (8,6),None)

		F, mask = cv2.findFundamentalMat(corners1,corners2,cv2.FM_RANSAC)
		# now find epilines
		lines1 = cv2.computeCorrespondEpilines(corners2,2,F)
		lines1 = lines1.reshape(-1,3)
		lines2 = cv2.computeCorrespondEpilines(corners1,1,F)
		lines2 = lines2.reshape(-1,3)
		pts1 = []
		pts2 = []
		for i in range(0,corners1.shape[0]):
			point1 = (corners1[i,0,0], corners1[i,0,1])
			pts1.append(point1)
			point2 = (corners2[i,0,0], corners2[i,0,1])
			pts2.append(point2)

		img1_lined = drawlines(img1,lines1,pts1,pts2)
		img2_lined = drawlines(img2,lines2,pts2,pts1)
		epipole1 = meanIntersect(img1.shape[:2],lines1)
		epipole2 = meanIntersect(img2.shape[:2],lines2)
		print("epipole1")
		print(epipole1)
		print("epipole2")
		print(epipole2)
		cv2.imshow(last_image,img1_lined)
		cv2.imshow(current_image,img2_lined)
		last_image = current_image
	i+=1

cv2.waitKey()
