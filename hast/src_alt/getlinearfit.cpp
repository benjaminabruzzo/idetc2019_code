template <typename T>
std::vector<T> GetLinearFit(const std::vector<T>& data)
{
	T xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
	std::vector<T> xData;
	
	printf("data.size() = %d\n", int(data.size()));
	for (long i = 0; i < data.size(); i++)
	{
		xData.push_back(static_cast<T>(i));
	}
	for (long i = 0; i < data.size(); i++)
	{
		xSum += xData[i];
		ySum += data[i];
		xxSum += xData[i] * xData[i];
		xySum += xData[i] * data[i];
	}
	slope = (data.size() * xySum - xSum * ySum) / (data.size() * xxSum - xSum * xSum);
	intercept = (ySum - slope * xSum) / data.size();
	ROS_INFO("slope : %6.4f", slope);
	ROS_INFO("intercept : %6.4f", intercept);

	std::vector<T> res;
	res.push_back(slope);
	res.push_back(intercept);
	return res;



}

	/* Uasge example
	std::vector<double> myData;
	myData.push_back(1);
	myData.push_back(3);
	myData.push_back(4);
	myData.push_back(2);
	myData.push_back(5);

	std::vector<double> linearReg = GetLinearFit(myData);
	double slope = linearReg[0];
	double intercept = linearReg[1];

	*/


			// void findCompassDrift()
		// {

		// 	if 	(!liftoffSwitch)
		// 	{
		// 		if (flightState==2)
		// 		{
		// 			++compassCounter;
		// 			compassReadings.push_back(1);
		// 		} else { // set the switch to true and the drift rate
		// 			std::vector<double> linearReg = GetLinearFit(compassReadings);
		// 			yaw_drift_rate = linearReg[0];
		// 			ROS_WARN("yaw_drift_rate = %6.4f", yaw_drift_rate);
		// 			fprintf (mFile, "uavRecorder.est.yaw_drift_rate = %6.14f;\n", yaw_drift_rate);
		// 			// double intercept = linearReg[1];
		// 			liftoffSwitch = true; // only compute once before first liftoff
		// 		}
		// 	}
		// }
