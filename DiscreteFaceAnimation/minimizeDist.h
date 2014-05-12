/*
 *  minimizeDist.h
 *
 *  Created by Shunsuke Saito on 3/25/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */

#ifndef __20140325DISCRETEFACE__
#define __20140325DISCRETEFACE__

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "candiPhon.h"

#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"opencv_core249d.lib")
    #pragma comment(lib,"opencv_imgproc249d.lib")
    #pragma comment(lib,"opencv_highgui249d.lib")
#else
    //Releaseモードの場合
    #pragma comment(lib,"opencv_core249.lib")
    #pragma comment(lib,"opencv_imgproc249.lib")
    #pragma comment(lib,"opencv_highgui249.lib")
#endif

struct MINIMIZEDISTORTION
{
	CANDIPHON candidate;
	cv::Mat commonPointsMat;

	MINIMIZEDISTORTION(const string &ymlfile,const int candidateNum): candidate(candidateNum){
		readYML(ymlfile);		
	}
	~MINIMIZEDISTORTION(){};
	std::vector<PHONSEQ> result;

	void readYML(const string &ymlfile){
		std::cout << "Reading YML..." << std::endl;
		cv::FileStorage fs(ymlfile.c_str(), cv::FileStorage::READ);
		fs["CommonPoints"] >> commonPointsMat;
		std::cout << "Done..." << std::endl;
	}
	void minimizeDistortion();
};

#endif