/*
 *  geneFace.cpp
 *
 *  Created by Shunsuke Saito on 3/26/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */

#include "geneFace.h"
#include "movingPatch.h"

void GENEFACE::makeInterpFrame(const cv::Mat &imgA,const cv::Mat &imgB,const int frameNum,vector<cv::Mat> &vImage)
{
	cv::Mat imageA_Lab;
	cv::Mat imageB_Lab;
	cv::cvtColor(imgA,imageA_Lab,cv::COLOR_BGR2Lab);
	cv::cvtColor(imgB,imageB_Lab,cv::COLOR_BGR2Lab);

	vector<cv::Mat> vImageA;
	vector<cv::Mat> vImageB;

	cv::buildPyramidAnySize(imageA_Lab,vImageA,scaleDown,0.83);
	cv::buildPyramidAnySize(imageB_Lab,vImageB,scaleDown,0.83);
	
	//Initailize 
	vector<cv::Mat> vFrame;
	movingPatch(15,frameNum,vImageA[scaleDown],vImageB[scaleDown],vFrame);

	int maxIterate=1;
	for(int a=0;a<=scaleDown;++a)
	{
		cout << "Scale: " << a << endl;
		for(int b=0;b<maxIterate;++b)
		{
			cout << "Iteration: " << b << endl;
			cout << "Frame: ";
			for(int i=1;i<frameNum-1;++i)
			{
				cout << i << " ";
				double alpha=(frameNum-i)/(double)frameNum;

				//Compute T3
				cout << "Computing T3" << endl;
				cv::Mat T3=vFrame[i].clone();
				minimizeDistComp(5,0,vFrame[0],T3);

				//Compute T4
				cout << "Computing T4" << endl;
				cv::Mat T4=vFrame[i].clone();
				minimizeDistComp(5,0,vFrame[frameNum-1],T4);

				//Compute T5
				cout << "Computing T5" << endl;
				cv::Mat T5=vFrame[i].clone();
				minimizeDAlphaDisCohere(patchSize,0,alpha,vFrame[0],vFrame[frameNum-1],T5);

				//Add T1~5
				vFrame[i]=beta*alpha*T3+beta*(1.0-alpha)*T4+(1.0-beta)*T5;
			}
			cout << endl;
		}

		if(a!=scaleDown){
			//Upscaling
			for(int c=1;c<frameNum-1;++c)
			{
				cv::resize(vFrame[c],vFrame[c],vImageA[scaleDown-a-1].size(),cv::INTER_LANCZOS4);
			}
			vFrame[0]=vImageA[scaleDown-a-1].clone(); 
			vFrame[frameNum-1]=vImageB[scaleDown-a-1].clone();
			maxIterate=6*(scaleDown-a-1)/scaleDown+2;
		}
	}

	cout << "Save Image..." << endl;
	//Save Image
	vImage.resize(frameNum-2);
	for(int i=1;i<frameNum-1;++i)
	{
		cv::cvtColor(vFrame[i],vFrame[i],cv::COLOR_Lab2BGR);
		vImage[i-1]=vFrame[i];
	}
}

void GENEFACE::makeSentense(const string &output,const cv::Rect &rect)
{
	miniDist.minimizeDistortion();

	cv::Size imgSize;
	if(rect==cv::Rect()) imgSize=cv::Size((int)(backMovie.get(CV_CAP_PROP_FRAME_WIDTH)),(int)(backMovie.get(CV_CAP_PROP_FRAME_HEIGHT)));
	else imgSize=rect.size();
	cv::VideoWriter outVideo(output,static_cast<int>(backMovie.get(CV_CAP_PROP_FOURCC)),static_cast<int>(backMovie.get(CV_CAP_PROP_FPS)),imgSize,true);

	bool inInterp=false;
	int frameSize=0;
	cv::Mat frameA,frameB;

	for(int i=0;i<miniDist.result.size();++i)
	{
		const int duration=miniDist.result[i].duration;
		if(i%2==0){
			for(int j=0;j<duration;++j)
			{
				cv::Mat frame;
				backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+j*(duration/(double)miniDist.result[i].actualSize));
				backMovie >> frame;
				if(i!=0&&j==0){
					if(rect==cv::Rect()) frameB=frame.clone();
					else frameB=frame(rect).clone();
					std::vector<cv::Mat> vImage;
					makeInterpFrame(frameA,frameB,frameSize,vImage);					
					for(int k=0;k<vImage.size();++k)
					{
						outVideo << vImage[k];
					}
				}				
				if(rect==cv::Rect()) outVideo << frame;
				else outVideo << frame(rect);				
				
				if(j==duration-1){
					if(rect==cv::Rect()) frameA=frame.clone();
					else frameA=frame(rect).clone();
				}
			}
		}
		else{
			frameSize=duration;
		}
	}

	//for(int i=0;i<miniDist.result.size();++i)
	//{
	//	const int duration=miniDist.result[i].duration;
	//	if(duration<4){
	//		if(inInterp){
	//			frameSize+=duration;
	//		}
	//		else if(i==0){
	//			cv::Mat frame;
	//			backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame);
	//			backMovie >> frame;
	//			if(rect==cv::Rect()) outVideo << frame;
	//			else outVideo << frame(rect);
	//			frameSize+=duration-1;
	//		}
	//		else if(i==miniDist.result.size()-1){
	//			if(frameSize==0){cout << "frameSize is 0" << endl; abort();}
	//			frameSize+=duration-1;
	//			for(int j=0;j<duration-1;++j)
	//			{
	//				backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+j*(duration/(double)miniDist.result[i].actualSize));
	//				backMovie >> frameB;
	//				std::vector<cv::Mat> vImage;
	//				if(rect==cv::Rect()) makeInterpFrame(frameA,frameB,frameSize,vImage);
	//				else makeInterpFrame(frameA(rect).clone(),frameB(rect).clone(),frameSize,vImage);
	//				
	//				for(int k=0;k<vImage.size();++k)
	//				{
	//					outVideo << vImage[k];
	//				}
	//				if(rect==cv::Rect()) outVideo << frameB;
	//				else outVideo << frameB(rect);
	//			}
	//		}
	//		else{
	//			cout << "duration error" << endl;
	//			abort();
	//		}

	//		continue;
	//	}

	//	for(int j=0;j<duration;++j)
	//	{
	//		++frameSize;			

	//		if(i!=0&&j==1){
	//			backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+2);
	//			backMovie >> frameB;
	//			std::vector<cv::Mat> vImage;
	//			if(rect==cv::Rect()) makeInterpFrame(frameA,frameB,frameSize,vImage);
	//			else makeInterpFrame(frameA(rect).clone(),frameB(rect).clone(),frameSize,vImage);
	//			for(int k=0;k<vImage.size();++k)
	//			{
	//				outVideo << vImage[k];
	//			}
	//			if(rect==cv::Rect()) outVideo << frameB;
	//			else outVideo << frameB(rect);
	//			inInterp=false;
	//		}

	//		if(j==duration-2){
	//			backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].endFrame-2);
	//			backMovie >> frameA;
	//			if(rect==cv::Rect()) outVideo << frameA;
	//			else outVideo << frameA(rect);
	//			inInterp=true;
	//			frameSize=1;
	//		}

	//		if(!inInterp){
	//			cv::Mat frame;
	//			backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+j*(duration/(double)miniDist.result[i].actualSize));
	//			backMovie >> frame;
	//			if(rect==cv::Rect()) outVideo << frame;
	//			else outVideo << frame(rect);
	//		}

	//	}

	//}
}