/*
 *  candiPho.cpp
 *
 *  Created by Shunsuke Saito on 3/25/14.
 *  Copyright 2014 Shunsuke Saito. All rights reserved.
 *
 */

#include "candiPhon.h"

bool CANDIPHON::readCandidate(const int numCandidate)
{	
	for(int i=0;i<numCandidate;++i)
	{
		std::ostringstream fname;
		fname << folder.c_str() << formerPart.c_str() << i << latterPart.c_str();

		std::string name=fname.str();

		readPhonemeFile(name);
	}

	return true;
}

bool CANDIPHON::readPhonemeFile(const std::string &filename)
{
	std::vector<PHONSEQ> sPhonSeq;

	//Open filestream
	std::ifstream in( filename.c_str() );
	if( !in ) {
		std::cerr << "Can't Open File" << std::endl; 
		return false;
	}

	// get number of vertecies, textures, normals 
	char *next;
	char rline[MAX_READ_LINE];
	std::string str;
	std::vector<std::string> token;

	bool isEven=true;
	while( !in.eof() ) {
		// read the line of the file
		in.getline(rline, MAX_READ_LINE);
		
		// forward to the first non-space character in rline
		for(next = rline; *next != '\0' && isspace(*next); next++ ){
		}

		// skip blank lines and comments
		if( *next == '\0' ) {
			continue;	// continue
		}
		if( *next == '#' || *next == '$') {
			continue;
		}
		
		str = next;
		SplitToken( str, token );

		std::vector<std::string> temp_token; // each value between "|"
		std::vector< std::vector<std::string> > sub_token; //[seg][value]

		PHONSEQ phoneme;
		if( token.size() != 0 ) {
			// Skip two line from the beginning
			if( token[0] == "name:" ) {		
				continue;
			}
			else if( token[0] == "score:" ) {
				continue;
			}

			SplitToken( token[0], '|', temp_token);
			sub_token.resize(2);
			for(int i=0;i<2;++i)
			{
				SplitToken( temp_token[i], ',', sub_token[i]);
			}

			phoneme.startFrame=atoi( sub_token[0][0].c_str())/2;//value is doubled 
			phoneme.endFrame=atoi( sub_token[0][1].c_str())/2;//value is doubled

			int duration=atoi(sub_token[1][0].c_str());
			if(duration%2==0) phoneme.duration=duration/2;
			else{
				if(isEven){ phoneme.duration=duration/2; isEven=false;}
				else{ phoneme.duration=1+duration/2; isEven=true;}
			}
			phoneme.actualSize=atoi( sub_token[1][1].c_str())/2;//value is doubled

			sPhonSeq.push_back(phoneme);
		}
	}

	vSetPhon.push_back(sPhonSeq);

	// Close filestream
	in.close();

	return true;
}
