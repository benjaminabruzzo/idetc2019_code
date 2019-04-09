// "extern" means that this is not a variable declaration:

#ifndef HAST_UTILS_H
#define HAST_UTILS_H

#include "genheaders.hpp"

cv::Mat wrapH(cv::Mat R_in, cv::Mat t_in);
cv::Mat invertH(cv::Mat H);
double wrapDegrees(double angle);
double wrapRadians(double angle);
double toDegrees(double radians);
double toRadians(double degrees);


#endif