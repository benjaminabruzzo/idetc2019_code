float64 stamp 	# time of measurement
uint32 id # serial number of message

bool odomBIT 	# boolean bit for guidance/odometry information
bool odomMODE	# boolean bit for linear/angular odometry mode

vector3 P 		# 3D Position
matrix3x3 PCov 	# 3D Position covariance matrix

vector3 V 		# 3D Velocity
vector3 A 		# 3D Acceleration

float64 yaw 	# estimated compass direction
float64 yaw_cov 	# estimated compass direction covariance

matrix3x3 R 	# 3D rotation matrix
matrix3x3 RCov 	# 3D Rotation covariance matrix #deprecated?

#vector3 ihat 	# estimated heading vector
#vector3 KfP		# Position rider for odometry

pixels red
pixels blue
pixels green
