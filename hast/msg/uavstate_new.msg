bool odomBIT 	# boolean bit for guidance/odometry information
bool odomMODE	# boolean bit for linear/angular odometry mode

matrix3x3 PCov 	# 3D Position covariance matrix
matrix3x3 R 	# 3D rotation matrix
matrix3x3 RCov 	# 3D Rotation covariance matrix #deprecated?

pixels red
pixels blue
pixels green

vector3 V 		# 3D Velocity
vector3 P 		# 3D Position

float64 yaw_cov 	# estimated compass direction covariance
float64 yaw 	# estimated compass direction

float64 stamp 	# time of measurement
uint32 id # serial number of message
