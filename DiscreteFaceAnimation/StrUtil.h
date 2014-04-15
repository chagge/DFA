// Utility for string operation
// 2004.06.17
// (c) ATR SLT ASR 
// coded by Tatsuo Yotsukura
#ifndef __STR_UTIL_H__
#define __STR_UTIL_H__


#pragma warning(disable:4786)		// STL�g�p���̃��[�j���O����

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>

using namespace std;

// �g�[�N�������F�X�y�[�X�E�^�u�E���s
void SplitToken(const string &buf, vector<string> &vsToken);
// �g�[�N�������F�w�肵���ꕶ��
void SplitToken(const string &buf, const char &sep, vector<string> &vsToken);
// ������->�啶���i�ꕶ���j
char CapitalChar( const char c );
// ������->�啶���istring�j
string &CapitalString( const string &s1, string &s2 );
// �啶��->�������i�ꕶ���j
char SmallChar( const char c );
// �啶��->�������istring�j
string &SmallString( const string &s1, string &s2 );

#endif //__STR_UTIL_H__