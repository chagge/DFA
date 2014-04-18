/*
 *  geneFace.h
 *
 *  Created by Shunsuke Saito on 3/26/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */
#ifndef __20140326GENEFACE__
#define __20140326GENEFACE__

#include "minimizeDist.h"

struct GENEFACE
{
	cv::VideoCapture backMovie;
	MINIMIZEDISTORTION miniDist;
	const int patchSize;
	const int scaleDown;
	const double beta;

	GENEFACE(const string &filename,const std::string ymlFile,const int candiNum) : backMovie(filename), miniDist(ymlFile, candiNum), patchSize(7), scaleDown(0), beta(0.8){};
	~GENEFACE(){};

	void makeInterpFrame(const cv::Mat &imgA,const cv::Mat &imgB,const int frameNum,vector<cv::Mat> &vImage);
	void makeSentense(const string &output,const cv::Rect &rect);
};

#endif
