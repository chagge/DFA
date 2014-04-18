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
				cv::Point2f ptF[4],ptB[4];
				int backEndFrame=candidate.vSetPhon[data[i-1]][i-1].endFrame;
				int foreStartFrame=candidate.vSetPhon[j][i].startFrame;
				ptB[0] = commonPointsMat.at<cv::Point2f>(0,backEndFrame);
				ptB[1] = commonPointsMat.at<cv::Point2f>(12,backEndFrame);
				ptB[2] = commonPointsMat.at<cv::Point2f>(13,backEndFrame);
				ptB[3] = commonPointsMat.at<cv::Point2f>(14,backEndFrame);

				ptF[0] = commonPointsMat.at<cv::Point2f>(0,foreStartFrame);
				ptF[1] = commonPointsMat.at<cv::Point2f>(12,foreStartFrame);
				ptF[2] = commonPointsMat.at<cv::Point2f>(13,foreStartFrame);
				ptF[3] = commonPointsMat.at<cv::Point2f>(14,foreStartFrame);

				for(int k=0;k<4;++k)
				{
					cv::Point2f diff=ptF[k]-ptB[k];
					curE+=diff.ddot(diff);
				}

				//coherence term
				if(i>1){
					cv::Point2f ptBB[4],ptBF[4];
	
					int bbackEndFrame=candidate.vSetPhon[data[i-2]][i-2].endFrame;
					int backStartFrame=candidate.vSetPhon[data[i-1]][i-1].startFrame;

					ptBB[0] = commonPointsMat.at<cv::Point2f>(0,bbackEndFrame);
					ptBB[1] = commonPointsMat.at<cv::Point2f>(12,bbackEndFrame);
					ptBB[2] = commonPointsMat.at<cv::Point2f>(13,bbackEndFrame);
					ptBB[3] = commonPointsMat.at<cv::Point2f>(14,bbackEndFrame);

					ptBF[0] = commonPointsMat.at<cv::Point2f>(0,backStartFrame);
					ptBF[1] = commonPointsMat.at<cv::Point2f>(12,backStartFrame);
					ptBF[2] = commonPointsMat.at<cv::Point2f>(13,backStartFrame);
					ptBF[3] = commonPointsMat.at<cv::Point2f>(14,backStartFrame);

					for(int k=0;k<4;++k)
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


}