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

def invertH:
	return 0

def calibrateUsingImages(images):
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1) 
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*8,3), np.float32)
	objp[:,:2] = 0.111*np.mgrid[0:8,0:6].T.reshape(-1,2)
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
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
	return ret, cameraMatrix, distortionCoeffs, rotationVecs, translationVecs


# images = glob.glob('*.png')
LeftImages = glob.glob('/home/benjamin/pycal/left*.png')
RightImages = glob.glob('/home/benjamin/pycal/right*.png')

LeftReturn, LeftCameraMatrix, LeftDistortionCoeffs, LeftRotationVecs, LeftTranslationVecs = calibrateUsingImages(LeftImages)
RightReturn, RightCameraMatrix, RightDistortionCoeffs, RightRotationVecs, RightTranslationVecs = calibrateUsingImages(RightImages)

print("f = open(" + filename + " , 'w')")
f = open(filename, 'w')
nxm2mfile(f,cameraMatrix, "caldata.cameraMatrix")
nxm2mfile(f,objp, "caldata.chessboard3Dpoints")
nxm2mfile(f,distortionCoeffs, "caldata.distortionCoeffs")

i = 1
for v,t in zip(rotationVecs,translationVecs):
	R,J = cv2.Rodrigues(v)
	dataname = "caldata.R.R" + str(i) # print R
	nxm2mfile(f,R, dataname) # print R
	dataname = "caldata.t.t" + str(i) # print R
	nxm2mfile(f,t, dataname) # print R
	R = np.matrix(R)
	t = np.matrix(t)
	H = wrapH(R,t)
	dataname = "caldata.H.H" + str(i) # print R
	nxm2mfile(f,H, dataname) # print R
	i+=1


f.close()



fname = "/home/benjamin/pycal/left_raw_00112.png"
img = cv2.imread(fname)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (8,6),None)


print(objp[0])
print(corners[0][0])

print(objp[5])
print(corners[5][0])

print(objp[32])
print(corners[32][0])


# black circle = origin
cv2.circle(img, (corners[0][0][0], corners[0][0][1]), 10, (0,0,0), -1)

# blue circle = +x in  object space
cv2.circle(img, (corners[5][0][0], corners[5][0][1]), 10, (255,0,0), -1)

# red circle = +y in  object space
cv2.circle(img, (corners[32][0][0], corners[32][0][1]), 10, (0,255,0), -1)


x = int(corners[32][0][0])
y = int(corners[32][0][1]+10)
cv2.circle(img, (x, y), 10, (255,255,255), 3)


# this corresponds to x = 0 and y = 0 in the (rectifierd?) camera optical frame
x = int(735)
y = int(441)
cv2.circle(img, (x, y), 10, (255,255,255), 3)

cv2.imshow('img',img)
cv2.waitKey()


cv2.destroyAllWindows()



R,J = cv2.Rodrigues(rotationVecs[0])
