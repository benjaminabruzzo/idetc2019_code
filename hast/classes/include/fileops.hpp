// "extern" means that this is not a variable declaration:

#ifndef HAST_FILEOPS_H
#define HAST_FILEOPS_H

#include "genheaders.hpp"
class fileops
{
	private:

	public:
		std::FILE * filename;
		std::string s_filename;
		int printlength;
		std::string datastring;


		fileops();
		
		void init_fileops(std::string mfile);
		void writeString(std::string data);
		void writeVecInt(std::vector<int> vectorOfInts, std::string vectorName, int count);
		void writeVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void cellVecDoubles(std::vector<double> vectorOfDoubles, std::string vectorName, int count);
		void cellVecInts(std::vector<int> vectorOfInts, std::string vectorName, int count);
		void cellVecUints(std::vector<uint> vectorOfUints, std::string vectorName, int count);
		void writeCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDoubles(cv::Mat matrixOfDoubles, std::string matrixName, int count);
		void cellCvMatDoubles_multline(cv::Mat matrixOfDoubles, std::string matrixName, int count);




};

#endif