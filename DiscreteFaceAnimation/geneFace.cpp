/*
 *  geneFace.cpp
 *
 *  Created by Shunsuke Saito on 3/26/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */

#include "geneFace.h"
#include "movingPatch.h"

void GENEFACE::makeInterpFrame(const cv::Mat &imgA,const cv::Mat &imgB,const int frameNum,vector<cv::Mat> &vImage,cv::Point &offset,cv::Point &trans)
{
	if(frameNum==2){cout << "FrameNum==2" << endl; abort();}

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
	movingPatch(15,frameNum,vImageA[scaleDown],vImageB[scaleDown],vFrame,offset,trans);

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
	for(int i=0;i<frameNum;++i)
	{
		cv::cvtColor(vFrame[i],vFrame[i],cv::COLOR_Lab2BGR);
	}

	vImage = vFrame;
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

	int count=0;
	cv::Point trans(0,0);
	cv::Point offset(0,0);

	for(int i=0;i<miniDist.result.size();++i)
	{
		const int duration=miniDist.result[i].duration;

		//For debug
		//cv::Mat debug;
		//backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].getMiddle());
		//backMovie >> debug;
		//			
		//cv::circle(debug,(cv::Point)miniDist.commonPointsMat.at<cv::Point2f>(0,miniDist.result[i].getMiddle()),3,cv::Scalar(200,0,0), -1, CV_AA);
		//cv::circle(debug,(cv::Point)miniDist.commonPointsMat.at<cv::Point2f>(1,miniDist.result[i].getMiddle()),3,cv::Scalar(200,0,0), -1, CV_AA);
		//cv::circle(debug,(cv::Point)miniDist.commonPointsMat.at<cv::Point2f>(11,miniDist.result[i].getMiddle()),3,cv::Scalar(200,0,0), -1, CV_AA);
		//cv::circle(debug,(cv::Point)miniDist.commonPointsMat.at<cv::Point2f>(12,miniDist.result[i].getMiddle()),3,cv::Scalar(200,0,0), -1, CV_AA);

		//cv::imshow("Debug",debug);
		//cv::waitKey(30);
		//getchar();
		cout << miniDist.result.size() << " " << i << " " << duration << endl;
		if(duration>3){
			bool flag = false;
			for(int j=0;j<duration;++j)
			{
				cv::Mat frame;
				//frameSize++;

				backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+j*(duration/(double)miniDist.result[i].actualSize));
				backMovie >> frame;

				if(i!=0&&j==1){
					if(frameSize==3){ frameSize+=duration; break;}
					frameSize++;
					if(rect==cv::Rect()) frameB=frame.clone();
					else frameB=frame(rect).clone();
					std::vector<cv::Mat> vImage;
					makeInterpFrame(frameA,frameB,frameSize,vImage,miniDist.offsetVec[i-1],trans);
					cout << "makeInterpolation " << frameSize << endl;
					count += frameSize;
					for(int k=0;k<vImage.size();++k)
					{
						outVideo << vImage[k];
						cout << "Add interpolated image" << endl;
						
					}
					flag = true;
					continue;
				} else if(i!=miniDist.result.size()-1&&j==duration-2){					
					if(rect==cv::Rect()) frameA=frame.clone();
					else frameA=frame(rect).clone();
					cout << "Input A" << endl;
					frameSize=2;
					frameSize++;
					flag = false;
					continue;
				}
				if(i==0) flag = true;
				if(flag){
					if(rect==cv::Rect()) outVideo << frame;
					else outVideo << frame(rect);						
					cout << "Add image" << endl;
					count++;
				}
			}
		}
		else{
			if(frameSize>10){
				if(duration==1){
					cv::Mat frame;
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame);
					backMovie >> frame;
					if(rect==cv::Rect()) frameB=frame.clone();
					else frameB=frame(rect).clone();
					std::vector<cv::Mat> vImage;
					makeInterpFrame(frameA,frameB,frameSize,vImage,miniDist.offsetVec[i-1],trans);
					cout << "makeInterpolation " << frameSize << endl;
					count += frameSize;
					for(int k=0;k<vImage.size()-1;++k)
					{
						outVideo << vImage[k];
						cout << "Add interpolated image" << endl;
						
					}
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame);
					backMovie >> frame;
					if(rect==cv::Rect()) frameA=frame.clone();
					else frameA=frame(rect).clone();
					cout << "Input A" << endl;
					frameSize=2;
				}else if(duration==2){
					cv::Mat frame;
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame);
					backMovie >> frame;
					if(rect==cv::Rect()) frameB=frame.clone();
					else frameB=frame(rect).clone();
					std::vector<cv::Mat> vImage;
					makeInterpFrame(frameA,frameB,frameSize,vImage,miniDist.offsetVec[i-1],trans);
					cout << "makeInterpolation " << frameSize << endl;
					count += frameSize;
					for(int k=0;k<vImage.size();++k)
					{
						outVideo << vImage[k];
						cout << "Add interpolated image" << endl;
						
					}
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+1);
					backMovie >> frame;
					if(rect==cv::Rect()) frameA=frame.clone();
					else frameA=frame(rect).clone();
					cout << "Input A" << endl;
					frameSize=2;
				}else if(duration==3){
					cv::Mat frame;
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame);
					backMovie >> frame;
					if(rect==cv::Rect()) frameB=frame.clone();
					else frameB=frame(rect).clone();
					std::vector<cv::Mat> vImage;
					makeInterpFrame(frameA,frameB,frameSize,vImage,miniDist.offsetVec[i-1],trans);
					cout << "makeInterpolation " << frameSize << endl;
					count += frameSize;
					for(int k=0;k<vImage.size();++k)
					{
						outVideo << vImage[k];
						cout << "Add interpolated image" << endl;
						
					}
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+1);
					backMovie >> frame;
					if(rect==cv::Rect()) outVideo << frame.clone();
					else outVideo << frame(rect).clone();
					backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].startFrame+2);
					backMovie >> frame;
					if(rect==cv::Rect()) frameA=frame.clone();
					else frameA=frame(rect).clone();
					cout << "Input A" << endl;
					frameSize=2;
				}
			}
			if(i==miniDist.result.size()-1){
				frameSize+=duration-1;
				cv::Mat frame;
				backMovie.set(CV_CAP_PROP_POS_FRAMES,(double)miniDist.result[i].endFrame);
				backMovie >> frame;
				if(rect==cv::Rect()) frameB=frame.clone();
				else frameB=frame(rect).clone();
				std::vector<cv::Mat> vImage;
				makeInterpFrame(frameA,frameB,frameSize,vImage,miniDist.offsetVec[i-1],trans);
				cout << "makeInterpolation " << frameSize << endl;
				count += frameSize;
				for(int k=0;k<vImage.size();++k)
				{
					outVideo << vImage[k];
					cout << "Add interpolated image" << endl;
						
				}
			}
			cout << "continue" << endl;
			frameSize+=duration;
		}					
		if(i!=0) trans += miniDist.offsetVec[i-1];
	}
	cout << count << endl;
	getchar();
}
