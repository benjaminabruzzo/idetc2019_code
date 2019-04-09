## ~~~~~~~~~~~~ 3D Data ~~~~~~~~~~~~~~~~~~ ##

# Drone 3D position in tb frame
# The drone frame rotation is calculated in the state estimator, where a KF is employed
Vector3 P_tb
float64 PCovTrace_tb
#mat3x3 PCov_tb

# Drone heading unit vector (fused 3 target data)
Vector3 ihat_tb
float64 iCovTrace_tb

# Time stamp of image capture
float64 stamp



## ~~~~~~~~~~~~ 2D Data ~~~~~~~~~~~~~~~~~~ ##
# Drone horizontal 
# mat2x2 


# Pose.iCov.row0.x = s2(0,0);
# Pose.iCov.row0.y = s2(0,1);
# Pose.iCov.row0.z = s2(0,2);
# Pose.iCov.row1.x = s2(1,0);
# Pose.iCov.row1.y = s2(1,1);
# Pose.iCov.row1.z = s2(1,2);
# Pose.iCov.row2.x = s2(2,0);
# Pose.iCov.row2.y = s2(2,1);
# Pose.iCov.row2.z = s2(2,2);