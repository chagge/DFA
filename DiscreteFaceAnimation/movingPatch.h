#include <iostream>
#include <iterator>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "MT.h"

using namespace std;

#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"opencv_core248d.lib")
    #pragma comment(lib,"opencv_imgproc248d.lib")
    #pragma comment(lib,"opencv_highgui248d.lib")
#else
    //Releaseモードの場合
    #pragma comment(lib,"opencv_core248.lib")
    #pragma comment(lib,"opencv_imgproc248.lib")
    #pragma comment(lib,"opencv_highgui248.lib")
#endif

namespace cv{

	int SSD(const cv::Mat& lhs,const cv::Mat& rhs)
	{
		int tmp=0;
		cv::Vec3i diff;
		for(int i=0;i<lhs.rows;++i)
		{
			const cv::Vec3b* lptr=lhs.ptr<cv::Vec3b>(i);
			const cv::Vec3b* rptr=rhs.ptr<cv::Vec3b>(i);
			for(int j=0;j<lhs.cols;++j)
			{
				diff=(cv::Vec3i)lptr[j]-(cv::Vec3i)rptr[j];
				tmp+=diff.dot(diff);
			}
		}
		return tmp;
	}

	void buildPyramidAnySize(const cv::Mat &src,vector<cv::Mat> &vTar,int maxLevel,double scale)
	{
		vTar.resize(maxLevel+1);

		vTar[0]=src.clone();
		for(int i=1;i<=maxLevel;++i)
		{
			cv::resize(vTar[i-1],vTar[i],cv::Size(),scale,scale,cv::INTER_LANCZOS4);	
		}
	}
};

//get -1.0~1.0
double getRand()
{
	int value=genrand_int32()%1000-500;
	return (double)value/500.0;
}

void generatePatch(int patchSize,const cv::Mat& src,vector<cv::Mat> &patchSrc,vector<cv::Point> &pointSrc)
{
	const int heightSrc=src.rows;
	const int widthSrc=src.cols;

	for(int i=0;i<=heightSrc-patchSize;++i)
	{
		for(int j=0;j<=widthSrc-patchSize;++j)
		{
			cv::Mat tmp=src(cv::Range(i,i+patchSize),cv::Range(j,j+patchSize));		
			patchSrc.push_back(tmp);
			pointSrc.push_back(cv::Point(j,i));
		}
	}

}

void localPatchMatch(int patchSize,int maxIterate,const cv::Mat& src,const vector<cv::Mat> &patchSrc,const vector<cv::Point> &pointSrc,
	const cv::Mat& tar,const vector<cv::Mat> &patchTar,const vector<cv::Point> &pointTar,vector<int> &_patchIndexSrc)
{
	const int Ns=patchSrc.size();
	const int Nt=patchTar.size();

	const int widthSrc=src.cols;
	const int heightSrc=src.rows;
	
	const int heightTar=tar.rows;
	const int widthTar=tar.cols;

	//initialize(set Patch randomly or )
	int size=Ns;
	vector<int> randomIndex;
	randomIndex.resize(size);
	for(int i=0;i<size;++i)
	{
		//need to restrict the patchs to be out of the mask
		if(i<Nt){randomIndex[i]=i;}
		else{
			randomIndex[i]=Nt-1;
		}			
	}

	_patchIndexSrc=randomIndex;

	//set Offset
	vector<cv::Point2i> offsetSrc;
	offsetSrc.resize(Ns);
	for(int i=0;i<Ns;++i)
	{
		offsetSrc[i]=pointTar[_patchIndexSrc[i]]-pointSrc[i];
	}

	//Search for src to tar
	for(int c=0;c<maxIterate;++c)
	{
		for(int i=0;i<Ns;++i)
		{
			int curIndexSrc,curIndexTar;
			int minD;
			if(c%2==0){
				curIndexSrc=i;											
				cv::Point2i curTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
				minD=cv::SSD(patchSrc[curIndexSrc],patchTar[curTar.y*(widthTar-patchSize+1)+curTar.x]);

				if(i/(widthSrc-patchSize+1)!=0){
					cv::Point2i upTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc-(widthSrc-patchSize+1)];
					if(upTar.x>=0&&upTar.x<=widthTar-patchSize&&upTar.y>=0&&upTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[upTar.y*(widthTar-patchSize+1)+upTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc-(widthSrc-patchSize+1)];}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					cv::Point2i leftTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc-1];
					if(leftTar.x>=0&&leftTar.x<=widthTar-patchSize&&leftTar.y>=0&&leftTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[leftTar.y*(widthTar-patchSize+1)+leftTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc-1];}
					}
				}
			}else{
				curIndexSrc=Ns-i-1;							

				cv::Point2i curTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
				minD=cv::SSD(patchSrc[curIndexSrc],patchTar[curTar.y*(widthTar-patchSize+1)+curTar.x]);

				if(i/(widthSrc-patchSize+1)!=0){
					cv::Point2i belowTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc+(widthSrc-patchSize+1)];

					if(belowTar.x>=0&&belowTar.x<=widthTar-patchSize&&belowTar.y>=0&&belowTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[belowTar.y*(widthTar-patchSize+1)+belowTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc+(widthSrc-patchSize+1)];}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					cv::Point2i rightTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc+1];

					if(rightTar.x>=0&&rightTar.x<=widthTar-patchSize&&rightTar.y>=0&&rightTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[rightTar.y*(widthTar-patchSize+1)+rightTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc+1];}
					}
				}
			}

			//ramdom step
			double alpha=0.5;
			cv::Point curPos=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];

			int w; heightTar>widthTar?w=heightTar*0.05:w=widthTar*0.05;
			while(w>1){
				cv::Point2i disp(w*getRand(),w*getRand());
				cv::Point2i randomTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc]+disp;

				if(randomTar.x>=0&&randomTar.x<=widthTar-patchSize&&randomTar.y>=0&&randomTar.y<=heightTar-patchSize){
					int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[randomTar.y*(widthTar-patchSize+1)+randomTar.x]);
					if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc]+disp;}					
				}
				w=w*alpha;
			}
			cv::Point2i resultTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
			_patchIndexSrc[curIndexSrc]=resultTar.y*(widthTar-patchSize+1)+resultTar.x;			
		}
	}
}

void patchMatch(int patchSize,int iterateNum,int pyramidLevel,const cv::Mat& src,const vector<cv::Mat> &patchSrc,const vector<cv::Point> &pointSrc,
	const cv::Mat& tar,const vector<cv::Mat> &patchTar,const vector<cv::Point> &pointTar,vector<int> &_patchIndexSrc)
{
	const int maxIterate = 2;

	const int Ns=patchSrc.size();
	const int Nt=patchTar.size();

	const int widthSrc=src.cols;
	const int heightSrc=src.rows;
	
	const int heightTar=tar.rows;
	const int widthTar=tar.cols;

	//initialize(set Patch randomly or )2回目以降のiterateは受け取ったやつを使う感じ
	if(pyramidLevel==0&&iterateNum==0){
		int size;
		Ns>Nt ? size=Ns : size=Nt;
		vector<int> randomIndex;
		randomIndex.resize(size);
		for(int i=0;i<size;++i)
		{
			//need to restrict the patchs to be out of the mask
			if(i<Nt){randomIndex[i]=i;}
			else{
				randomIndex[i]=genrand_int31()%Nt;
			}			
		}

		for(int i=0;i<size;++i)
		{
			int j=i+genrand_int31()%(size-i);
			swap(randomIndex[i],randomIndex[j]);
		}	

		randomIndex.resize(Ns);

		_patchIndexSrc=randomIndex;
	}
	else if(iterateNum==0){
		vector<int> vTmp=_patchIndexSrc;
		_patchIndexSrc.clear();
		_patchIndexSrc.resize(Ns);

		for(int i=0;i<vTmp.size();++i)
		{
			int tarIndex=vTmp[i];

			int X=i%(widthSrc/2-patchSize+1);
			int Y=i/(widthSrc/2-patchSize+1);

			int x=tarIndex%(widthTar/2-patchSize+1);
			int y=tarIndex/(widthTar/2-patchSize+1);

			_patchIndexSrc[2*Y*(widthSrc-patchSize+1)+2*X]=2*y*(widthTar-patchSize+1)+2*x;
			_patchIndexSrc[(2*Y+1)*(widthSrc-patchSize+1)+2*X+1]=(2*y+1)*(widthTar-patchSize+1)+2*x+1;
		}

		for(int i=0;i<Ns;++i)
		{
			int tmp=genrand_int31()%Nt;

			int tmpD=cv::SSD(patchSrc[i],patchTar[tmp]);
			int curD=cv::SSD(patchSrc[i],patchTar[_patchIndexSrc[i]]);
			if(tmpD<curD){_patchIndexSrc[i]=tmp;}
		}	
	}

	//set Offset
	vector<cv::Point2i> offsetSrc;
	offsetSrc.resize(Ns);
	for(int i=0;i<Ns;++i)
	{
		offsetSrc[i]=pointTar[_patchIndexSrc[i]]-pointSrc[i];
	}

	//Search for src to tar
	for(int c=0;c<maxIterate;++c)
	{
		for(int i=0;i<Ns;++i)
		{
			int curIndexSrc,curIndexTar;
			int minD;
			if(c%2==0){
				curIndexSrc=i;											
				cv::Point2i curTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
				minD=cv::SSD(patchSrc[curIndexSrc],patchTar[curTar.y*(widthTar-patchSize+1)+curTar.x]);

				if(i/(widthSrc-patchSize+1)!=0){
					cv::Point2i upTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc-(widthSrc-patchSize+1)];
					if(upTar.x>=0&&upTar.x<=widthTar-patchSize&&upTar.y>=0&&upTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[upTar.y*(widthTar-patchSize+1)+upTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc-(widthSrc-patchSize+1)];}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					cv::Point2i leftTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc-1];
					if(leftTar.x>=0&&leftTar.x<=widthTar-patchSize&&leftTar.y>=0&&leftTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[leftTar.y*(widthTar-patchSize+1)+leftTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc-1];}
					}
				}
			}else{
				curIndexSrc=Ns-i-1;							

				cv::Point2i curTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
				minD=cv::SSD(patchSrc[curIndexSrc],patchTar[curTar.y*(widthTar-patchSize+1)+curTar.x]);

				if(i/(widthSrc-patchSize+1)!=0){
					cv::Point2i belowTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc+(widthSrc-patchSize+1)];

					if(belowTar.x>=0&&belowTar.x<=widthTar-patchSize&&belowTar.y>=0&&belowTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[belowTar.y*(widthTar-patchSize+1)+belowTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc+(widthSrc-patchSize+1)];}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					cv::Point2i rightTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc+1];

					if(rightTar.x>=0&&rightTar.x<=widthTar-patchSize&&rightTar.y>=0&&rightTar.y<=heightTar-patchSize){
						int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[rightTar.y*(widthTar-patchSize+1)+rightTar.x]);
						if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc+1];}
					}
				}
			}

			//ramdom step
			double alpha=0.5;
			cv::Point curPos=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];

			int widthMax,heightMax;					
			if(curPos.y>heightTar-curPos.y-patchSize+1) heightMax=curPos.y;
			else heightMax=heightTar-patchSize+1-curPos.y;
			if(curPos.x>widthTar-curPos.x-patchSize+1) widthMax=curPos.x;
			else widthMax=widthTar-patchSize+1-curPos.x;

			int w; heightMax>widthMax?w=heightMax:w=widthMax;
			while(w>1){
				cv::Point2i disp(w*getRand(),w*getRand());
				cv::Point2i randomTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc]+disp;

				if(randomTar.x>=0&&randomTar.x<=widthTar-patchSize&&randomTar.y>=0&&randomTar.y<=heightTar-patchSize){
					int tmpD=cv::SSD(patchSrc[curIndexSrc],patchTar[randomTar.y*(widthTar-patchSize+1)+randomTar.x]);
					if(tmpD<minD){minD=tmpD;offsetSrc[curIndexSrc]=offsetSrc[curIndexSrc]+disp;}					
				}
				w=w*alpha;
			}
			cv::Point2i resultTar=pointSrc[curIndexSrc]+offsetSrc[curIndexSrc];
			_patchIndexSrc[curIndexSrc]=resultTar.y*(widthTar-patchSize+1)+resultTar.x;			
		}
	}
}

void minimizeDistBDS(int patchSize,int pyramidLevel,const cv::Mat &src,cv::Mat &tar)
{
	unsigned int Eprev=0;
	unsigned int Enow=0;
	unsigned int minE=0;

	vector<int> patchIndexSrc; vector<int> patchIndexTar;			
	vector<cv::Mat> tempSrc;vector<cv::Mat> tempTar;
	vector<cv::Mat> patchSrc; vector<cv::Point> pointSrc;
	vector<cv::Mat> patchTar; vector<cv::Point> pointTar;
	
	cv::buildPyramid(src,tempSrc,pyramidLevel);
	cv::buildPyramid(tar,tempTar,pyramidLevel);

	int maxIterate=4;
	for(int a=0;a<=pyramidLevel;++a)
	{
		const int heightTar=tempTar[pyramidLevel-a].rows;
		const int widthTar=tempTar[pyramidLevel-a].cols;

		cv::Mat img4Vote(tempTar[pyramidLevel-a].size(),CV_64FC3);
		cv::Mat voteNumTar(tempTar[pyramidLevel-a].size(),CV_32SC1);
		cv::Mat voteNumSrc(tempTar[pyramidLevel-a].size(),CV_32SC1);

		for(int c=0;c<maxIterate;++c)
		{		
			//Generate patch
			//cout << "Generating Source Patch" << endl;
			patchSrc.clear(); pointSrc.clear();
			generatePatch(patchSize,tempSrc[pyramidLevel-a],patchSrc,pointSrc);
			//cout << "Done..." << endl;

			//cout << "Generating Target Patch" << endl;
			patchTar.clear(); pointTar.clear();
			generatePatch(patchSize,tempTar[pyramidLevel-a],patchTar,pointTar);
			//cout << "Done..." << endl;
			
			//cout << "Patch Match Src to Tar" << endl;
			patchMatch(patchSize,c,a,tempSrc[pyramidLevel-a],patchSrc,pointSrc,tempTar[pyramidLevel-a],patchTar,pointTar,patchIndexSrc);
			//cout << "Patch Match Tar to Src" << endl;
			patchMatch(patchSize,c,a,tempTar[pyramidLevel-a],patchTar,pointTar,tempSrc[pyramidLevel-a],patchSrc,pointSrc,patchIndexTar);

			const int Ns = patchSrc.size();
			const int Nt = patchTar.size();

			img4Vote.setTo(cv::Vec3d(0,0,0));
			voteNumTar.setTo(0);
			voteNumSrc.setTo(0);

			unsigned int dComplete=0;
			unsigned int dCohere=0;

			//vote Src to Tar
			//cout << "Voting Src for Tar" << endl;
			for(int i=0;i<patchSrc.size();++i)
			{
				dComplete+=cv::SSD(patchSrc[i],patchTar[patchIndexSrc[i]]);

				for(int k=0;k<patchSize;++k)
				{
					for(int l=0;l<patchSize;++l)
					{						
						img4Vote.at<cv::Vec3d>(pointTar[patchIndexSrc[i]]+cv::Point(k,l))+=(cv::Vec3d)(patchSrc[i].at<cv::Vec3b>(cv::Point(k,l)))/(double)Ns;
						voteNumSrc.at<int>(pointTar[patchIndexSrc[i]]+cv::Point(k,l))+=1;
					}
				}
			}
			//cout << "Done..." << endl;
			dComplete/=Ns;

			//cout << "Voting Tar for Src" << endl;
			for(int i=0;i<patchTar.size();++i)
			{
				dCohere+=cv::SSD(patchSrc[patchIndexTar[i]],patchTar[i]);
				for(int k=0;k<patchSize;++k)
				{
					for(int l=0;l<patchSize;++l)
					{
						img4Vote.at<cv::Vec3d>(pointTar[i]+cv::Point(k,l))+=(cv::Vec3d)(patchSrc[patchIndexTar[i]].at<cv::Vec3b>(cv::Point(k,l)))/(double)Nt;
						voteNumTar.at<int>(pointTar[i]+cv::Point(k,l))+=1;
					}
				}
			}
			//cout << "Done..." << endl;
			dCohere/=Nt;
			Enow=dComplete+dCohere;

			//cout << "dComp: " << dComplete << " dCohere: " << dCohere << " dBDS: " << Enow << endl;

			Eprev=Enow;

			for(int i=0;i<heightTar;++i)
			{
				cv::Vec3b* imgPtr=tempTar[pyramidLevel-a].ptr<cv::Vec3b>(i);
				cv::Vec3d* votePtr=img4Vote.ptr<cv::Vec3d>(i);
				int* nPtr=voteNumSrc.ptr<int>(i);
				int* mPtr=voteNumTar.ptr<int>(i);
				for(int j=0;j<widthTar;++j)
				{
					imgPtr[j]=votePtr[j]/(double)(nPtr[j]/(double)Ns+mPtr[j]/(double)Nt);	
				}
			}
		}
		if(a!=pyramidLevel){ 
			cv::pyrUp(tempTar[pyramidLevel-a],tempTar[pyramidLevel-a-1],tempTar[pyramidLevel-a-1].size());
			maxIterate=18*(pyramidLevel-a-1)/pyramidLevel+2;
		}
	}
}

void minimizeDistComp(int patchSize,int pyramidLevel,const cv::Mat &src,cv::Mat &tar)
{
	unsigned int Eprev=0;
	unsigned int Enow=0;
	unsigned int minE=0;

	vector<int> patchIndexSrc;
	vector<cv::Mat> tempSrc; vector<cv::Mat> tempTar;
	vector<cv::Mat> patchSrc; vector<cv::Point> pointSrc;
	vector<cv::Mat> patchTar; vector<cv::Point> pointTar;

	cv::buildPyramid(tar,tempTar,pyramidLevel);
	cv::buildPyramid(src,tempSrc,pyramidLevel);

	int maxIterate=3;
	for(int a=0;a<=pyramidLevel;++a)
	{
		const int heightTar=tempTar[pyramidLevel-a].rows;
		const int widthTar=tempTar[pyramidLevel-a].cols;

		cv::Mat img4Vote(tempTar[pyramidLevel-a].size(),CV_64FC3);
		cv::Mat voteNumSrc(tempTar[pyramidLevel-a].size(),CV_32SC1);

		for(int c=0;c<maxIterate;++c)
		{
			//cout << "Generating Source Patch" << endl;
			patchSrc.clear(); pointSrc.clear();
			generatePatch(patchSize,tempSrc[pyramidLevel-a],patchSrc,pointSrc);
			//cout << "Done..." << endl;

			//cout << "Generating Target Patch" << endl;
			patchTar.clear(); pointTar.clear();
			generatePatch(patchSize,tempTar[pyramidLevel-a],patchTar,pointTar);
			//cout << "Done..." << endl;
			
			//cout << "Patch Match Src to Tar" << endl;
			patchMatch(patchSize,c,a,tempSrc[pyramidLevel-a],patchSrc,pointSrc,tempTar[pyramidLevel-a],patchTar,pointTar,patchIndexSrc);
		
			const int Ns = patchSrc.size();
			const int Nt = patchTar.size();

			img4Vote.setTo(cv::Vec3d(0,0,0));
			voteNumSrc.setTo(0);

			unsigned int dComplete=0;
			unsigned int dCohere=0;

			//vote Src to Tar
			//cout << "Voting Src for Tar" << endl;
			for(int i=0;i<patchSrc.size();++i)
			{
				dComplete+=cv::SSD(patchSrc[i],patchTar[patchIndexSrc[i]]);

				for(int k=0;k<patchSize;++k)
				{
					for(int l=0;l<patchSize;++l)
					{						
						img4Vote.at<cv::Vec3d>(pointTar[patchIndexSrc[i]]+cv::Point(k,l))+=(cv::Vec3d)(patchSrc[i].at<cv::Vec3b>(cv::Point(k,l)));
						voteNumSrc.at<int>(pointTar[patchIndexSrc[i]]+cv::Point(k,l))+=1;
					}
				}
			}
			//cout << "Done..." << endl;
			dComplete/=Ns;

			Enow=dComplete+dCohere;

			//cout << "dComp: " << dComplete << " dCohere: " << dCohere << " dBDS: " << Enow << endl;

			Eprev=Enow;

			for(int i=0;i<heightTar;++i)
			{
				cv::Vec3b* imgPtr=tempTar[pyramidLevel-a].ptr<cv::Vec3b>(i);
				cv::Vec3d* votePtr=img4Vote.ptr<cv::Vec3d>(i);
				int* nPtr=voteNumSrc.ptr<int>(i);
				for(int j=0;j<widthTar;++j)
				{
					if(nPtr[j]){
						imgPtr[j]=votePtr[j]/(double)nPtr[j];
					}
				}
			}
		}
		if(a!=pyramidLevel){
			cv::pyrUp(tempTar[pyramidLevel-a],tempTar[pyramidLevel-a-1],tempTar[pyramidLevel-a-1].size());
			maxIterate=18*(pyramidLevel-a-1)/pyramidLevel+2;
		}
	}
}

void minimizeDAlphaDisCohere(int patchSize,int pyramidLevel,double alpha,const cv::Mat &src1,const cv::Mat &src2,cv::Mat &tar)
{
	unsigned int Eprev=0;
	unsigned int Enow=0;
	unsigned int minE=0;

	vector<int> patchIndexTar1;vector<int> patchIndexTar2;
	vector<cv::Mat> tempTar;vector<cv::Mat> tempSrc1;vector<cv::Mat> tempSrc2;
	vector<cv::Mat> patchSrc1; vector<cv::Point> pointSrc1;
	vector<cv::Mat> patchSrc2; vector<cv::Point> pointSrc2;
	vector<cv::Mat> patchTar; vector<cv::Point> pointTar;

	cv::buildPyramid(src1,tempSrc1,pyramidLevel);
	cv::buildPyramid(src2,tempSrc2,pyramidLevel);
	cv::buildPyramid(tar,tempTar,pyramidLevel);

	int maxIterate=3;
	for(int a=0;a<=pyramidLevel;++a)
	{
		const int heightTar=tempTar[pyramidLevel-a].rows;
		const int widthTar=tempTar[pyramidLevel-a].cols;
		cv::Mat img4Vote(tempTar[pyramidLevel-a].size(),CV_64FC3);
		cv::Mat voteNumTar1(tempTar[pyramidLevel-a].size(),CV_32SC1);
		cv::Mat voteNumTar2(tempTar[pyramidLevel-a].size(),CV_32SC1);

		for(int c=0;c<maxIterate;++c)
		{
			//cout << "Generating Source1 Patch" << endl;
			patchSrc1.clear(); pointSrc1.clear();
			generatePatch(patchSize,tempSrc1[pyramidLevel-a],patchSrc1,pointSrc1);
			//cout << "Done..." << endl;

			//cout << "Generating Source2 Patch" << endl;
			patchSrc2.clear(); pointSrc2.clear();
			generatePatch(patchSize,tempSrc2[pyramidLevel-a],patchSrc2,pointSrc2);			
			//cout << "Done..." << endl;

			//Generate patch
			//cout << "Generating Target Patch" << endl;
			patchTar.clear(); pointTar.clear();
			generatePatch(patchSize,tempTar[pyramidLevel-a],patchTar,pointTar);
			//cout << "Done..." << endl;
			
			//cout << "Patch Match for Src1" << endl;
			patchMatch(patchSize,c,a,tempTar[pyramidLevel-a],patchTar,pointTar,tempSrc1[pyramidLevel-a],patchSrc1,pointSrc1,patchIndexTar1);
			//cout << "Patch Match for Src2" << endl;
			patchMatch(patchSize,c,a,tempTar[pyramidLevel-a],patchTar,pointTar,tempSrc2[pyramidLevel-a],patchSrc2,pointSrc2,patchIndexTar2);

			const int Ns1 = patchSrc1.size();
			const int Ns2 = patchSrc2.size();
			const int Nt = patchTar.size();

			img4Vote.setTo(cv::Vec3d(0,0,0));
			voteNumTar1.setTo(0);
			voteNumTar2.setTo(0);

			unsigned int dComplete=0;
			unsigned int dCohere=0;

			//Sorting
			multimap<int,int> SSD;
			for(int i=0;i<Nt;++i)
			{
				//comparion
				int src1_SSD=cv::SSD(patchTar[i],patchSrc1[patchIndexTar1[i]]);
				int src2_SSD=cv::SSD(patchTar[i],patchSrc2[patchIndexTar2[i]]);
				SSD.insert(multimap<int,int>::value_type(src1_SSD-src2_SSD,i));
			}
			
			//vote Src1 to Tar
			multimap<int,int>::iterator it=SSD.begin();
			int threIndex = Nt*alpha;
			for(int i=0;i<Nt;++i)
			{
				cv::Mat patchSrc;
				if(i<threIndex) patchSrc=patchSrc1[patchIndexTar1[it->second]];
				else patchSrc=patchSrc2[patchIndexTar2[it->second]];

				for(int k=0;k<patchSize;++k)
				{
					for(int l=0;l<patchSize;++l)
					{
						img4Vote.at<cv::Vec3d>(pointTar[it->second]+cv::Point(k,l))+=(cv::Vec3d)(patchSrc.at<cv::Vec3b>(cv::Point(k,l)));
						voteNumTar1.at<int>(pointTar[it->second]+cv::Point(k,l))+=1;
					}
				}
				++it;
			}

			dCohere/=Nt;
			Enow=dComplete+dCohere;

			Eprev=Enow;

			for(int i=0;i<heightTar;++i)
			{
				cv::Vec3b* imgPtr=tempTar[pyramidLevel-a].ptr<cv::Vec3b>(i);
				cv::Vec3d* votePtr=img4Vote.ptr<cv::Vec3d>(i);
				int* mPtr=voteNumTar1.ptr<int>(i);
				for(int j=0;j<widthTar;++j)
				{
					imgPtr[j]=votePtr[j]/(double)mPtr[j];
				}
			}
		}
		if(a!=pyramidLevel){
			cv::pyrUp(tempTar[pyramidLevel-a],tempTar[pyramidLevel-a-1],tempTar[pyramidLevel-a-1].size());
			maxIterate=18*(pyramidLevel-a-1)/pyramidLevel+2;
		}
	}
}

void smootingPatch(int patchSize,int maxIterate,const cv::Mat& src,const vector<cv::Mat> &patchSrc,const vector<cv::Point> &pointSrc,
	const vector<cv::Mat> &patchTar,const vector<cv::Point> &pointTar,vector<int> &_patchIndexSrc,vector<int> &_patchIndexTar)
{
	const int Ns=patchSrc.size();

	const int widthSrc=src.cols;
	const int heightSrc=src.rows;
	
	for(int c=0;c<maxIterate;++c)
	{
		for(int i=0;i<Ns;++i)
		{
			int curIndexSrc,curIndexTar;
			int minDistSrc;int minSSDSrc;
			int minDistTar;int minSSDTar;
			if(c%2==0){
				//propagation
				curIndexSrc=curIndexTar=i;								
		
				cv::Point2i curDiffSrc=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[_patchIndexSrc[curIndexSrc]]];
				minDistSrc=curDiffSrc.ddot(curDiffSrc);
				minSSDSrc=cv::SSD(patchSrc[curIndexSrc],patchTar[_patchIndexSrc[curIndexSrc]]);

				cv::Point2i curDiffTar=pointTar[curIndexTar]-pointTar[_patchIndexSrc[_patchIndexTar[curIndexTar]]];
				minDistTar=curDiffTar.ddot(curDiffTar);
				minSSDTar=cv::SSD(patchTar[curIndexTar],patchSrc[_patchIndexTar[curIndexTar]]);

				if(i/(widthSrc-patchSize+1)!=0){
					int tarIndexSrc=_patchIndexSrc[curIndexSrc-(widthSrc-patchSize+1)]+(widthSrc-patchSize+1);
					if(tarIndexSrc<Ns&&tarIndexSrc>=0){
						cv::Point2i upDiff=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[tarIndexSrc]];
						int tmpDist=upDiff.ddot(upDiff);
						int tmpSSD=cv::SSD(patchSrc[curIndexSrc],patchTar[tarIndexSrc]);
						if(/*tmpSSD<minSSDSrc&&*/tmpDist<minDistSrc){minDistSrc=tmpDist;minSSDSrc=tmpSSD;_patchIndexSrc[curIndexSrc]=tarIndexSrc;}
					}

					int tarIndexTar=_patchIndexTar[curIndexTar-(widthSrc-patchSize+1)]+(widthSrc-patchSize+1);
					if(tarIndexTar<Ns&&tarIndexTar>=0){
						cv::Point2i upDiff=pointTar[curIndexTar]-pointTar[_patchIndexSrc[tarIndexTar]];
						int tmpDist=upDiff.ddot(upDiff);
						int tmpSSD=cv::SSD(patchTar[curIndexTar],patchSrc[tarIndexTar]);
						if(/*tmpSSD<minSSDTar&&*/tmpDist<minDistTar){minDistTar=tmpDist;minSSDTar=tmpSSD;_patchIndexTar[curIndexTar]=tarIndexTar;}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					int	tarIndexSrc=_patchIndexSrc[curIndexSrc-1]+1;
					if(tarIndexSrc<Ns&&tarIndexSrc>=0){
						cv::Point2i leftDiff=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[tarIndexSrc]];
						int tmpDist=leftDiff.ddot(leftDiff);
						int tmpSSD=cv::SSD(patchSrc[curIndexSrc],patchTar[tarIndexSrc]);
						if(/*tmpSSD<minSSDSrc&&*/tmpDist<minDistSrc){minDistSrc=tmpDist;minSSDSrc=tmpSSD;_patchIndexSrc[curIndexSrc]=tarIndexSrc;}
					}
					
					int tarIndexTar=_patchIndexTar[curIndexTar-1]+1;
					if(tarIndexTar<Ns&&tarIndexTar>=0){
						cv::Point2i leftDiff=pointTar[curIndexTar]-pointTar[_patchIndexSrc[tarIndexTar]];
						int tmpDist=leftDiff.ddot(leftDiff);
						int tmpSSD=cv::SSD(patchTar[curIndexTar],patchSrc[tarIndexTar]);
						if(/*tmpSSD<minSSDTar&&*/tmpDist<minDistTar){minDistTar=tmpDist;minSSDTar=tmpSSD;_patchIndexTar[curIndexTar]=tarIndexTar;}
					}
				}
			}else{
				curIndexSrc=curIndexTar=Ns-i-1;							

				cv::Point2i curDiff=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[_patchIndexSrc[curIndexSrc]]];
				minDistSrc=curDiff.ddot(curDiff);
				minSSDSrc=cv::SSD(patchSrc[curIndexSrc],patchTar[_patchIndexSrc[curIndexSrc]]);

				if(i/(widthSrc-patchSize+1)!=0){
					int tarIndexSrc=_patchIndexSrc[curIndexSrc+(widthSrc-patchSize+1)]-(widthSrc-patchSize+1);
					if(tarIndexSrc<Ns&&tarIndexSrc>=0){
						cv::Point2i belowDiff=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[tarIndexSrc]];
						int tmpDist=belowDiff.ddot(belowDiff);
						int tmpSSD=cv::SSD(patchSrc[curIndexSrc],patchTar[tarIndexSrc]);
						if(/*tmpSSD<minSSDSrc&&*/tmpDist<minDistSrc){minDistSrc=tmpDist;minSSDSrc=tmpSSD;_patchIndexSrc[curIndexSrc]=tarIndexSrc;}
					}

					int tarIndexTar=_patchIndexTar[curIndexTar+(widthSrc-patchSize+1)]-(widthSrc-patchSize+1);
					if(tarIndexTar<Ns&&tarIndexTar>=0){
						cv::Point2i belowDiff=pointTar[curIndexTar]-pointTar[_patchIndexSrc[tarIndexTar]];
						int tmpDist=belowDiff.ddot(belowDiff);
						int tmpSSD=cv::SSD(patchTar[curIndexTar],patchSrc[tarIndexTar]);
						if(/*tmpSSD<minSSDTar&&*/tmpDist<minDistTar){minDistTar=tmpDist;minSSDTar=tmpSSD;_patchIndexTar[curIndexTar]=tarIndexTar;}
					}
				}
				if(i%(widthSrc-patchSize+1)!=0){
					int	tarIndexSrc=_patchIndexSrc[curIndexSrc+1]-1;
					if(tarIndexSrc<Ns&&tarIndexSrc>=0){
						cv::Point2i rightDiff=pointSrc[curIndexSrc]-pointSrc[_patchIndexTar[tarIndexSrc]];
						int tmpDist=rightDiff.ddot(rightDiff);
						int tmpSSD=cv::SSD(patchSrc[curIndexSrc],patchTar[tarIndexSrc]);
						if(/*tmpSSD<minSSDSrc&&*/tmpDist<minDistSrc){minDistSrc=tmpDist;minSSDSrc=tmpSSD;_patchIndexSrc[curIndexSrc]=tarIndexSrc;}
					}
				
					int tarIndexTar=_patchIndexTar[curIndexTar+1]-1;
					if(tarIndexTar<Ns&&tarIndexTar>=0){
						cv::Point2i rightDiff=pointTar[curIndexTar]-pointTar[_patchIndexSrc[tarIndexTar]];
						int tmpDist=rightDiff.ddot(rightDiff);
						int tmpSSD=cv::SSD(patchTar[curIndexTar],patchSrc[tarIndexTar]);
						if(/*tmpSSD<minSSDTar&&*/tmpDist<minDistTar){minDistTar=tmpDist;minSSDTar=tmpSSD;_patchIndexTar[curIndexTar]=tarIndexTar;}
					}
				}
			}
		}
	}
}

//Interpolation
void movingPatch(const int patchSize,const int frameNum,const cv::Mat &imgA,const cv::Mat &imgB,vector<cv::Mat> &vImg)
{
	vector<int> patchIndexA;
	vector<int> patchIndexB;
	vector<cv::Mat> patchA; vector<cv::Point> pointA;
	vector<cv::Mat> patchB; vector<cv::Point> pointB;

	//Generate patch A
	cout << "Generating Patch A" << endl;
	patchA.clear(); pointA.clear();
	generatePatch(patchSize,imgA,patchA,pointA);
	//cout << "Patch A Number:" << patchA.size() << endl;

	//Generate patch B
	cout << "Generating Patch B" << endl;		
	patchB.clear(); pointB.clear();
	generatePatch(patchSize,imgB,patchB,pointB);
	//cout << "Patch B Number:" << patchB.size() << endl;
		
	cout << "Patch Match" << endl;
	localPatchMatch(patchSize,2,imgA,patchA,pointA,imgB,patchB,pointB,patchIndexA);
	localPatchMatch(patchSize,2,imgB,patchB,pointB,imgA,patchA,pointA,patchIndexB);	

	const int Ns=(int)patchA.size();
	const int Nt=(int)patchB.size();

	//cout << "Smoothing" << endl;
	smootingPatch(patchSize,2,imgA,patchA,pointA,patchB,pointB,patchIndexA,patchIndexB);

	cout << "Converting Offset" << endl;
	vector<cv::Point> vOffsetA;
	vector<cv::Point> vOffsetB;
	vOffsetA.resize(Ns);
	vOffsetB.resize(Ns);
	for(int i=0;i<pointA.size();++i)
	{
		vOffsetA[i]=pointB[patchIndexA[i]]-pointA[i];
		vOffsetB[i]=pointA[patchIndexB[i]]-pointB[i];
	}

	cv::Mat img4Vote(imgA.size(),CV_64FC3);
	cv::Mat voteNumA(imgA.size(),CV_32SC1);
	cv::Mat voteNumB(imgA.size(),CV_32SC1);

	vImg.clear();
	vImg.resize(frameNum);
	vImg[0]=imgA.clone();
	vImg[frameNum-1]=imgB.clone();
	for(int a=1;a<frameNum-1;++a)
	{
		img4Vote.setTo(cv::Vec3d(0,0,0));
		voteNumA.setTo(0);
		voteNumB.setTo(0);

		vImg[a]=imgA.clone();
		//vote Src to Tar
		for(int i=0;i<patchA.size();++i)
		{
			cv::Point midOffset(a*vOffsetA[i].x/(frameNum-1),a*vOffsetA[i].y/(frameNum-1)); 
			int BIndex=(pointA[i].y+midOffset.y)*(vImg[a].cols-patchSize+1)+pointA[i].x+midOffset.x;

			for(int k=0;k<patchSize;++k)
			{
				for(int l=0;l<patchSize;++l)
				{						
					img4Vote.at<cv::Vec3d>(pointB[BIndex]+cv::Point(k,l))+=patchA[i].at<cv::Vec3b>(cv::Point(k,l));
					voteNumA.at<int>(pointB[BIndex]+cv::Point(k,l))+=1;
				}
			}
		}

		for(int i=0;i<patchB.size();++i)
		{
			cv::Point midOffset((frameNum-1-a)*vOffsetB[i].x/(frameNum-1),(frameNum-1-a)*vOffsetB[i].y/(frameNum-1)); 
			int AIndex=(pointB[i].y+midOffset.y)*(vImg[a].cols-patchSize+1)+pointB[i].x+midOffset.x;

			for(int k=0;k<patchSize;++k)
			{
				for(int l=0;l<patchSize;++l)
				{						
					img4Vote.at<cv::Vec3d>(pointA[AIndex]+cv::Point(k,l))+=patchB[i].at<cv::Vec3b>(cv::Point(k,l));
					voteNumB.at<int>(pointA[AIndex]+cv::Point(k,l))+=1;
				}
			}
		}

		for(int i=0;i<vImg[a].rows;++i)
		{
			cv::Vec3b* imgPtr=vImg[a].ptr<cv::Vec3b>(i);
			cv::Vec3d* votePtr=img4Vote.ptr<cv::Vec3d>(i);
			int* nPtr=voteNumA.ptr<int>(i);
			int* mPtr=voteNumB.ptr<int>(i);
			for(int j=0;j<vImg[a].cols;++j)
			{
				if(nPtr[j]+mPtr[j])imgPtr[j]=votePtr[j]/(double)(nPtr[j]+mPtr[j]);
			}
		}
	}
}
