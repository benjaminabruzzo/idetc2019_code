#!/bin/python
#!/bin/python
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
# call as python stereo_calibrate.py 20180209 004
import numpy as np
import cv2
import glob
import sys
print(sys.argv)  # Note the first argument is always the script filename.

def nxm2mfile(fileservice, data, datastring): # write data to file
	for i in range(0,data.shape[0]):
		line = datastring + "(" + str(i+1) + ",:)=["
		for j in range(0,data.shape[1]):
			line += (str(data[i,j])) + " "
		line += "];\n"
		fileservice.write(line)
	fileservice.write("\n")

def makeHforCalOuts(R,t): 
	H = np.concatenate((R, t), axis=1)
	H = np.concatenate((H, np.matrix('0 0 0 1')), axis=0)
	return H

def calibrateUsingImages(images):
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.1) 
	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((6*8,3), np.float32)
	objp[:,:2] = 0.111*np.mgrid[0:8,0:6].T.reshape(-1,2)
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	# print(images)
	for fname in sorted(images):
		# fname=images[0]
		print(fname)
		img = cv2.imread(fname)
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
		if ret == True: # If found, add object points, image points (after refining them)
			# cv2.cornerSubPix
			rt = cv2.cornerSubPix(gray, corners, (11, 11),(-1, -1), criteria)
			objpoints.append(objp)
			imgpoints.append(corners)
	ret, cameraMatrix, distortionCoeffs, rotationVecs, translationVecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
	camera_model = dict([('M', cameraMatrix), ('dist', distortionCoeffs),('rvecs', rotationVecs), ('tvecs', translationVecs), 
						('img_shape', gray.shape[::-1]), ('objp', objp), ('objpoints', objpoints), ('imgpoints', imgpoints)])
	return camera_model
	# return ret, cameraMatrix, distortionCoeffs, rotationVecs, translationVecs, objp, objpoints, imgpoints

def stereo_calibrate(LeftDict, RightDict):
	flags = 0
	# flags |= cv2.CALIB_FIX_INTRINSIC
	# flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
	flags |= cv2.CALIB_USE_INTRINSIC_GUESS
	flags |= cv2.CALIB_FIX_FOCAL_LENGTH
	# flags |= cv2.CALIB_FIX_ASPECT_RATIO
	flags |= cv2.CALIB_ZERO_TANGENT_DIST
	# flags |= cv2.CALIB_RATIONAL_MODEL
	# flags |= cv2.CALIB_SAME_FOCAL_LENGTH
	# flags |= cv2.CALIB_FIX_K3
	# flags |= cv2.CALIB_FIX_K4
	# flags |= cv2.CALIB_FIX_K5
	stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
	Ldist = LeftDict["dist"]
	Rdist = RightDict["dist"]
	LM = LeftDict["M"]
	RM = RightDict["M"]
	objpts = LeftDict["objpoints"]
	img_shape = LeftDict["img_shape"]
	Limgpts = LeftDict["imgpoints"]
	Rimgpts = RightDict["imgpoints"]
	ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(objpts, Limgpts, Rimgpts, img_shape, LM, Ldist, RM, Rdist, criteria=stereocalib_criteria, flags=flags)	
	# ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
	# 	LeftDict["objpoints"], 
	# 	LeftDict["imgpoints"], RightDict["imgpoints"], 
	# 	LeftDict["M"], LeftDict["dist"], RightDict["M"], RightDict["dist"], 
	# 	LeftDict["img_shape"], criteria=stereocalib_criteria, flags=flags)
	print('Intrinsic_mtx_1', M1)
	print('dist_1', d1)
	print('Intrinsic_mtx_2', M2)
	print('dist_2', d2)
	print('R', R)
	print('T', T)
	print('E', E)
	print('F', F)
	stereo_model = dict([('LM', M1), ('RM', M2), ('Ldist', d1),
						('Rdist', d2), ('Lrvecs', LeftDict["rvecs"]),
						('Rrvecs', RightDict["rvecs"]), ('R', R), ('T', T),
						('E', E), ('F', F)])
	return stereo_model


# images = glob.glob('*.png')
# try:
imgdir = '/home/benjamin/pycal/' + sys.argv[1] + '/' + sys.argv[2]
print(imgdir)
LeftImages = glob.glob('/home/benjamin/pycal/' + sys.argv[1] + '/' + sys.argv[2] + '/left*.png')
RightImages = glob.glob('/home/benjamin/pycal/' + sys.argv[1] + '/' + sys.argv[2] + '/right*.png')
filename = '/home/benjamin/ros/data/' + sys.argv[1] + '/' + sys.argv[2] + '/caldata_' + sys.argv[2] + '.m'
# except:
# 	LeftImages = glob.glob('/home/benjamin/pycal/20180209/003/left*.png')
# 	RightImages = glob.glob('/home/benjamin/pycal/20180209/003/right*.png')
# 	filename = '/home/benjamin/pycal/caldata_.m'

# print(LeftImages)

LeftCamera = calibrateUsingImages(LeftImages)
RightCamera = calibrateUsingImages(RightImages)
stereoModel = stereo_calibrate(LeftCamera, RightCamera)
# LeftReturn, LeftCameraMatrix, LeftDistortionCoeffs, LeftRotationVecs, LeftTranslationVecs, LeftObjP, Lobjpoints, Limgpoints = calibrateUsingImages(LeftImages)
# RightReturn, RightCameraMatrix, RightDistortionCoeffs, RightRotationVecs, RightTranslationVecs, RightObjP, Robjpoints, Rimgpoints = calibrateUsingImages(RightImages)





# print("f = open(" + filename + " , 'w')")
# f = open(filename, 'w')
# # Left Data
# nxm2mfile(f,LeftObjP, "caldata.left.chessboard3Dpoints")
# nxm2mfile(f,LeftCameraMatrix, "caldata.left.cameraMatrix")
# nxm2mfile(f,LeftDistortionCoeffs, "caldata.left.distortionCoeffs")

# i = 1
# for v,t in zip(LeftRotationVecs,LeftTranslationVecs):
# 	R,J = cv2.Rodrigues(v)
# 	dataname = "caldata.left.R.R" + str(i) # print R
# 	nxm2mfile(f,R, dataname) # print R
# 	dataname = "caldata.left.t.t" + str(i) # print R
# 	nxm2mfile(f,t, dataname) # print R
# 	R = np.matrix(R)
# 	t = np.matrix(t)
# 	H = makeHforCalOuts(R,t)
# 	dataname = "caldata.left.H.H" + str(i) # print R
# 	nxm2mfile(f,H, dataname) # print R
# 	i+=1

# nxm2mfile(f,RightObjP, "caldata.right.chessboard3Dpoints")
# nxm2mfile(f,RightCameraMatrix, "caldata.right.cameraMatrix")
# nxm2mfile(f,RightDistortionCoeffs, "caldata.right.distortionCoeffs")

# i = 1
# for v,t in zip(RightRotationVecs,RightTranslationVecs):
# 	R,J = cv2.Rodrigues(v)
# 	dataname = "caldata.right.R.R" + str(i) # print R
# 	nxm2mfile(f,R, dataname) # print R
# 	dataname = "caldata.right.t.t" + str(i) # print R
# 	nxm2mfile(f,t, dataname) # print R
# 	R = np.matrix(R)
# 	t = np.matrix(t)
# 	H = makeHforCalOuts(R,t)
# 	dataname = "caldata.right.H.H" + str(i) # print R
# 	nxm2mfile(f,H, dataname) # print R
# 	i+=1

# f.close()





# s = str(cameraMatrix)
# f.write(s)


# 	# Draw and display the corners
# 	cv2.drawChessboardCorners(img, (8,6), corners2,ret)
# 	cv2.imshow('img',img)
# 	cv2.waitKey()


# corners2 = corners
# cv2.cornerSubPix(gray,corners2,(11,11),(-1,-1),criteria)
# imgpoints.append(corners2)
