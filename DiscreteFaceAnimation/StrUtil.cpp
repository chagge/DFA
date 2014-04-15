#include "StrUtil.h"

// 機能：	トークン分割：スペース・タブ・改行
// 引数：	const string &buffer		入力
//			string <string> &vsToken	出力（分割されて入る）
// 戻値：	なし
void SplitToken(const string &buf, vector<string> &vsToken)
{
	istringstream isstr(buf);

	string str;
	
	// すべての有無を言わさず要素をすべて削除する (^^;
	vsToken.clear();
	
	while ( isstr >> str ) vsToken.push_back(str);

#if 0
	// 結果を出力する
	
	for( int i = 0; i < vsToken.size() ; i++ ) {
		printf( "Split Token. %02d = [%s]\n", i, vsToken[i].c_str() );
	}
#endif
}

// 機能：	トークン分割：指定した一文字
// 引数：	const string &buf			入力
//			const char &sep				分割用文字
//			string <string> &vsToken	出力（分割されて入る）
// 戻値：	なし
void SplitToken(const string &buf, const char &sep, vector<string> &vsToken)
{
	string buffer = buf;
	
	// すべての有無を言わさず要素をすべて削除する (^^;
	vsToken.clear();

	int pos = 0, prepos = 0;
	
	// cout << "[" << buf << "] " << endl;
	buffer = buffer + " " + sep;
	
	// 文字列の最後までトークンを検索する。
	while( buffer.npos != ( pos = (int)buffer.find_first_of( sep, prepos ) ))  {
		if( prepos != pos  ) { // 文字が空でなかったら配列に登録
			// 文字列を切り出す
			string str = buffer.substr( prepos,  pos - prepos );
			// 配列に追加
			vsToken.push_back( str );
			// 検索開始位置を変更
			prepos = pos + 1 ;
		} 
		else {
			// 検索位置を１文字進める
			prepos++ ;
		}
	}
	
	vsToken[vsToken.size() - 1].erase(vsToken[vsToken.size() - 1].size() - 1 );
	
#if 0
	// 結果を出力する
	
	for( int i = 0; i < vsToken.size() ; i++ ) {
		printf( "Split Token. %02d = [%s]\n", i, vsToken[i].c_str() );
	}
#endif
}


// 機能：	小文字->大文字（一文字）
// 引数：	const char c		入力（小文字）
// 戻値：	char				出力（大文字）
char CapitalChar( const char c )
{
	return static_cast<char>( toupper( c ) );
}

// 機能：	小文字->大文字（string）
// 引数：	const string &s1	入力（小文字）
//			string &s2			出力（大文字）
// 戻値：	string&				出力（大文字）
string &CapitalString( const string &s1, string &s2 )
{
	s2 = s1;
	transform( s1.begin(), s1.end(), s2.begin(), CapitalChar );
	return s2;
}

// 機能：	大文字->小文字（一文字）
// 引数：	const char c		入力（大文字）
// 戻値：	char				出力（小文字）
char SmallChar( const char c )
{
	return static_cast<char>( tolower( c ) );
}

// 機能：	大文字->小文字（string）
// 引数：	const string &s1	入力（大文字）
//			string &s2			出力（小文字）
// 戻値：	string&				出力（小文字）
string &SmallString( const string &s1, string &s2 )
{
	s2 = s1;
	transform( s1.begin(), s1.end(), s2.begin(), SmallChar );
	return s2;
}