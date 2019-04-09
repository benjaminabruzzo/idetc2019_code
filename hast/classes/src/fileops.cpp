#include "fileops.hpp"

fileops::fileops(){};

void fileops::init_fileops(std::string mfile)
{
	s_filename = mfile;
	ROS_INFO("fileops::filename: %s", s_filename.c_str());
	filename = std::fopen (s_filename.c_str(), "w");
	fprintf (filename, "%% %s\n", s_filename.c_str());
	fprintf (filename, "%%clc; \n%%clear all;\n%%close all;\n\n");

}

void fileops::writeString(std::string data){fprintf (filename, "%s", data.c_str());}


void fileops::writeVecInt(std::vector<int> vectorOfInts, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfInts.size();

	fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++) 
	{
		fprintf (filename, "%i ", vectorOfInts[i]);
	}
	fprintf (filename, "];\n");
}

void fileops::writeVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s(%i,:) = [", vectorName.c_str(), count);
	// fprintf (filename, "%i ", vectorSize);
	// ROS_INFO("augmentSLAM:: %s .size() = %i ", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++) 
	{
		fprintf (filename," %6.8f", vectorOfDoubles[i]);
	}
	fprintf (filename, "];\n");
}

void fileops::cellVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfDoubles.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++) 
	{
		fprintf (filename," %6.8f", vectorOfDoubles[i]);
	}
	fprintf (filename, "];\n");
}

void fileops::cellVecInts(std::vector<int> vectorOfInts, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfInts.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++) 
	{
		fprintf (filename," %i", vectorOfInts[i]);
	}
	fprintf (filename, "];\n");
}

void fileops::cellVecUints(std::vector<uint> vectorOfUints, std::string vectorName, int count)
{//writeCvMatDoubles(slam.slamMfile, SLAM_H, "slam.SLAM_H", slam.slamCount);
	int vectorSize = vectorOfUints.size();

	fprintf (filename, "%s{1,%i} = [", vectorName.c_str(), count);
	for(uint i = 0; i != vectorSize; i++) 
	{
		fprintf (filename," %u", vectorOfUints[i]);
	}
	fprintf (filename, "];\n");
}

void fileops::writeCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;

	for(uint rown = 0; rown != rows; rown++) 
	{
		fprintf (filename, "%s(%i,:, %i) = [", matrixName.c_str(), rown+1, count);
		for(uint coln = 0; coln != cols; coln++) 
		{
			fprintf (filename, " %6.8f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "];\n");
	}
}

void fileops::cellCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;


	fprintf (filename, "%s{%i,1} = [", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++) 
	{
		
		for(uint coln = 0; coln != cols; coln++) 
		{
			fprintf (filename, " %6.8f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ");
	}
	fprintf (filename, "];\n");
}

void fileops::cellCvMatDoubles_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count)
{
	int rows = matrixOfDoubles.rows;
	int cols = matrixOfDoubles.cols;


	fprintf (filename, "%s{%i,1} = [...\n", matrixName.c_str(), count);
	for(uint rown = 0; rown != rows; rown++) 
	{
		
		for(uint coln = 0; coln != cols; coln++) 
		{
			fprintf (filename, " %12.8f", matrixOfDoubles.at<double>(rown,coln));
		}
		fprintf (filename, "; ...\n");
	}
	fprintf (filename, "];\n");
}


