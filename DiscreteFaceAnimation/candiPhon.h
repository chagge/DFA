/*
 *  candiPho.h
 *
 *  Created by Shunsuke Saito on 3/25/14.
 *  Copyright 2010 Shunsuke Saito. All rights reserved.
 *
 */

#ifndef __20140325CANDIPHON__
#define __20140325CANDIPHON__

#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <time.h>


#include "StrUtil.h"

#ifndef MAX_READ_LINE
#define MAX_READ_LINE 1024
#endif

static const std::string folder="pointers/";
static const std::string formerPart="roos_r";
static const std::string latterPart="_pointer.txt";

struct PHONSEQ
{
	int startFrame, endFrame;
	int actualSize;
	int duration;

	void showValue(){
		std::cout << "Start:" << startFrame << " End:" << endFrame << " ActualDuration:" << actualSize << " ModifiedDuration:" << duration << std::endl;
	}
	int getMiddle(){return (startFrame+endFrame)/2;}
};

struct CANDIPHON
{
	std::vector< std::vector<PHONSEQ> > vSetPhon;//[candidateOrder][phonemeNumber]

	CANDIPHON(const int numCandidate){
		readCandidate(numCandidate);
	}
	~CANDIPHON(){};
	bool readCandidate(const int numCandidate);
	bool readPhonemeFile(const std::string &filename);
};

#endif
