#include "StrUtil.h"

// �@�\�F	�g�[�N�������F�X�y�[�X�E�^�u�E���s
// �����F	const string &buffer		����
//			string <string> &vsToken	�o�́i��������ē���j
// �ߒl�F	�Ȃ�
void SplitToken(const string &buf, vector<string> &vsToken)
{
	istringstream isstr(buf);

	string str;
	
	// ���ׂĂ̗L�������킳���v�f�����ׂč폜���� (^^;
	vsToken.clear();
	
	while ( isstr >> str ) vsToken.push_back(str);

#if 0
	// ���ʂ��o�͂���
	
	for( int i = 0; i < vsToken.size() ; i++ ) {
		printf( "Split Token. %02d = [%s]\n", i, vsToken[i].c_str() );
	}
#endif
}

// �@�\�F	�g�[�N�������F�w�肵���ꕶ��
// �����F	const string &buf			����
//			const char &sep				�����p����
//			string <string> &vsToken	�o�́i��������ē���j
// �ߒl�F	�Ȃ�
void SplitToken(const string &buf, const char &sep, vector<string> &vsToken)
{
	string buffer = buf;
	
	// ���ׂĂ̗L�������킳���v�f�����ׂč폜���� (^^;
	vsToken.clear();

	int pos = 0, prepos = 0;
	
	// cout << "[" << buf << "] " << endl;
	buffer = buffer + " " + sep;
	
	// ������̍Ō�܂Ńg�[�N������������B
	while( buffer.npos != ( pos = (int)buffer.find_first_of( sep, prepos ) ))  {
		if( prepos != pos  ) { // ��������łȂ�������z��ɓo�^
			// �������؂�o��
			string str = buffer.substr( prepos,  pos - prepos );
			// �z��ɒǉ�
			vsToken.push_back( str );
			// �����J�n�ʒu��ύX
			prepos = pos + 1 ;
		} 
		else {
			// �����ʒu���P�����i�߂�
			prepos++ ;
		}
	}
	
	vsToken[vsToken.size() - 1].erase(vsToken[vsToken.size() - 1].size() - 1 );
	
#if 0
	// ���ʂ��o�͂���
	
	for( int i = 0; i < vsToken.size() ; i++ ) {
		printf( "Split Token. %02d = [%s]\n", i, vsToken[i].c_str() );
	}
#endif
}


// �@�\�F	������->�啶���i�ꕶ���j
// �����F	const char c		���́i�������j
// �ߒl�F	char				�o�́i�啶���j
char CapitalChar( const char c )
{
	return static_cast<char>( toupper( c ) );
}

// �@�\�F	������->�啶���istring�j
// �����F	const string &s1	���́i�������j
//			string &s2			�o�́i�啶���j
// �ߒl�F	string&				�o�́i�啶���j
string &CapitalString( const string &s1, string &s2 )
{
	s2 = s1;
	transform( s1.begin(), s1.end(), s2.begin(), CapitalChar );
	return s2;
}

// �@�\�F	�啶��->�������i�ꕶ���j
// �����F	const char c		���́i�啶���j
// �ߒl�F	char				�o�́i�������j
char SmallChar( const char c )
{
	return static_cast<char>( tolower( c ) );
}

// �@�\�F	�啶��->�������istring�j
// �����F	const string &s1	���́i�啶���j
//			string &s2			�o�́i�������j
// �ߒl�F	string&				�o�́i�������j
string &SmallString( const string &s1, string &s2 )
{
	s2 = s1;
	transform( s1.begin(), s1.end(), s2.begin(), SmallChar );
	return s2;
}