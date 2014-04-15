// Utility for string operation
// 2004.06.17
// (c) ATR SLT ASR 
// coded by Tatsuo Yotsukura
#ifndef __STR_UTIL_H__
#define __STR_UTIL_H__


#pragma warning(disable:4786)		// STL使用時のワーニング制御

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>

using namespace std;

// トークン分割：スペース・タブ・改行
void SplitToken(const string &buf, vector<string> &vsToken);
// トークン分割：指定した一文字
void SplitToken(const string &buf, const char &sep, vector<string> &vsToken);
// 小文字->大文字（一文字）
char CapitalChar( const char c );
// 小文字->大文字（string）
string &CapitalString( const string &s1, string &s2 );
// 大文字->小文字（一文字）
char SmallChar( const char c );
// 大文字->小文字（string）
string &SmallString( const string &s1, string &s2 );

#endif //__STR_UTIL_H__