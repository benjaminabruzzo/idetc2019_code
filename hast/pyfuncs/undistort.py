#!/bin/python
#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
# call as python calibrate.py 20180209 004

import numpy as np
import cv2
import glob
import sys
# print(sys.argv)  # Note the first argument is always the script filename.

def nxm2mfile(fileservice, data, datastring): # write data to file
	for i in range(0,data.shape[0]):
		line = datastring + "(" + str(i+1) + ",:)=["
		for j in range(0,data.shape[1]):
			line += (str(data[i,j])) + " "
		line += "];\n"
		fileservice.write(line)
	fileservice.write("\n")

def wrapH(R,t): 
	H = np.concatenate((R, -R*t), axis=1)
	H = np.concatenate((H, np.matrix('0 0 0 1')), axis=0)
	return H


# Define termination criteria = ( type, max_iter = 30 , epsilon = 0.001 )
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = 0.111*np.mgrid[0:8,0:6].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# images = glob.glob('*.png')
images = glob.glob('/home/benjamin/pycal/*.png')

for fname in sorted(images):
	# fname=images[0]
	print(fname)
	img = cv2.imread(fname)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
	if ret == True: # If found, add object points, image points (after refining them)
		objpoints.append(objp)
		imgpoints.append(corners)


ret, cameraMatrix, distortionCoeffs, rotationVecs, translationVecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


fname = "/home/benjamin/pycal/left_raw_00112.png"
img = cv2.imread(fname)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (8,6),None)


h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameraMatrix,distortionCoeffs,(w,h),1,(w,h))

# undistort
dst = cv2.undistort(img, cameraMatrix, distortionCoeffs, None, newcameramtx)

cv2.imshow('dst',dst)
cv2.waitKey()


# # crop the image
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# outname = "/home/benjamin/pycal/left_raw_00112_cal.png"
# cv2.imwrite(outname,dst)



# # undistort
# mapx,mapy = cv2.initUndistortRectifyMap(cameraMatrix,distortionCoeffs,None,newcameramtx,(w,h),5)
# dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

# # crop the image
# # x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# cv2.imshow('dst',dst)
# cv2.waitKey()
