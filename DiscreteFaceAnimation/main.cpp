#include "geneFace.h"

const std::string movFile="t9_720.avi";
const std::string ymlFile="CommonPoints.yml";
const std::string output="result.avi";

const cv::Rect rect(535,260,146,100);
const int candiSize=10;

GENEFACE geneface(movFile,ymlFile,candiSize);

int main()
{
	geneface.makeSentense(output);

	return 0;
}
