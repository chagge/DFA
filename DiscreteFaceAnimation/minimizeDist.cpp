/*
 *  minimizeDist.cpp
 *
 *  Created by Shunsuke Saito on 3/25/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */

#include <math.h>
#include <algorithm>
#include "minimizeDist.h"

void MINIMIZEDISTORTION::minimizeDistortion()
{
	const int candiSize=candidate.vSetPhon.size();
	const int phonSize=candidate.vSetPhon[0].size();
	std::cout << "candiSize:" << candiSize << " PhonemeSize:" << phonSize << std::endl;
	//1,13,14,15 are anchor points.
	//const int totalComb=(int)powf((float)candiSize,(float)phonSize);
	//std::cout << "totalComb " << totalComb << endl;

	std::vector<int> data;
	for(int i=0; i<phonSize; ++i){
		data.push_back(0);
	}

	std::vector<int> minPhonSeq;
	double totalMinE=1.0e20;
	while(1)
	{
		double totalCurE=0.0;

		for(int i=1;i<phonSize;++i)
		{
			double minE=1.0e20;
			int candiIndex=0;
			for(int j=0;j<candiSize;++j)
			{
				double curE=0.0;
				
				//Adding the error of position
				cv::Point2f ptF[2],ptB[2];
				int backEndFrame=candidate.vSetPhon[data[i-1]][i-1].endFrame;
				int foreStartFrame=candidate.vSetPhon[j][i].startFrame;
				ptB[0] = commonPointsMat.at<cv::Point2f>(0,backEndFrame);
				ptB[1] = commonPointsMat.at<cv::Point2f>(12,backEndFrame);

				ptF[0] = commonPointsMat.at<cv::Point2f>(0,foreStartFrame);
				ptF[1] = commonPointsMat.at<cv::Point2f>(12,foreStartFrame);

				for(int k=0;k<2;++k)
				{
					cv::Point2f diff=ptF[k]-ptB[k];
					curE+=diff.ddot(diff);
				}

				//coherence term
				if(i>1){
					cv::Point2f ptBB[2],ptBF[2];
	
					int bbackEndFrame=candidate.vSetPhon[data[i-2]][i-2].endFrame;
					int backStartFrame=candidate.vSetPhon[data[i-1]][i-1].startFrame;

					ptBB[0] = commonPointsMat.at<cv::Point2f>(0,bbackEndFrame);
					ptBB[1] = commonPointsMat.at<cv::Point2f>(12,bbackEndFrame);

					ptBF[0] = commonPointsMat.at<cv::Point2f>(0,backStartFrame);
					ptBF[1] = commonPointsMat.at<cv::Point2f>(12,backStartFrame);

					for(int k=0;k<2;++k)
					{
						cv::Point2f diff=ptF[k]-ptB[k]-(ptBF[k]-ptBB[k]);
						curE+=diff.ddot(diff);
					}
				}

				if(curE<minE){
					minE=curE;
					candiIndex=j;
				}
			}
			totalCurE+=minE;
			data[i]=candiIndex;
		}
		std::cout << "totalCurE: " << totalCurE << std::endl;
		if(totalCurE<totalMinE){
			std::cout << "Update" << std::endl;
			totalMinE=totalCurE;
			minPhonSeq=data;
		}
		if(data[0]==candiSize-1) break;
		
		++data[0];
	}

	cout << "Finish" << endl;

	//Reflect the result
	result.resize(phonSize);
	for(int i=0;i<phonSize;++i)
	{
		result[i]=candidate.vSetPhon[data[i]][i];
		result[i].showValue();
	}

	//compute offset vector
	computeOffsetVec();
}

void MINIMIZEDISTORTION::computeOffsetVec(){
	offsetVec.resize(result.size()-1);

	//Adding the error of position
	cv::Point2f offset;
	for(int i=0;i<result.size()-1;++i)
	{
		int midFramePrev=result[i].getMiddle();
		int midFrameFore=result[i+1].getMiddle();
		offset = commonPointsMat.at<cv::Point2f>(0,midFrameFore)-commonPointsMat.at<cv::Point2f>(0,midFramePrev);
		offset += commonPointsMat.at<cv::Point2f>(12,midFrameFore)-commonPointsMat.at<cv::Point2f>(12,midFramePrev);
		offset *= 0.5;

		offsetVec[i] = (cv::Point)offset;		
	}

}