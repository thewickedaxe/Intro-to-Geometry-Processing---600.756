/* -*- C++ -*-
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <cassert>
#include <string.h>

#if defined( WIN32 ) || defined( _WIN64 )
inline int strcasecmp( char* c1 , char* c2 ){ return _stricmp( c1 , c2 ); }
#endif // WIN32 || _WIN64

template< class Type > void CmdLineWriteValue( Type t , char* str );
template< class Type > void CmdLineCleanUp( Type* t );
template< class Type > Type CmdLineInitialize( void );
template< class Type > Type CmdLineCopy( Type t );
template< class Type > Type CmdLineStringToType( const char* str );

template< > void CmdLineCleanUp< int    >( int*    t ){ }
template< > void CmdLineCleanUp< float  >( float*  t ){ }
template< > void CmdLineCleanUp< double >( double* t ){ }
template< > void CmdLineCleanUp< char*  >( char** t ){ if( *t ) free( *t ) ; *t = NULL; }
template< > int    CmdLineInitialize< int    >( void ){ return 0; }
template< > float  CmdLineInitialize< float  >( void ){ return 0.f; }
template< > double CmdLineInitialize< double >( void ){ return 0.; }
template< > char*  CmdLineInitialize< char*  >( void ){ return NULL; }
template< > void CmdLineWriteValue< int    >( int    t , char* str ){ sprintf( str , "%d" , t ); }
template< > void CmdLineWriteValue< float  >( float  t , char* str ){ sprintf( str , "%f" , t ); }
template< > void CmdLineWriteValue< double >( double t , char* str ){ sprintf( str , "%f" , t ); }
template< > void CmdLineWriteValue< char*  >( char*  t , char* str ){ if( t ) sprintf( str , "%s" , t ) ; else str[0]=0; }
template< > int    CmdLineCopy( int    t ){ return t;  }
template< > float  CmdLineCopy( float  t ){ return t;  }
template< > double CmdLineCopy( double t ){ return t;  }
#if defined( WIN32 ) || defined( _WIN64 )
template< > char*  CmdLineCopy( char* t ){ return _strdup( t ); }
#else // !WIN32 && !_WIN64
template< > char*  CmdLineCopy( char* t ){ return strdup( t ); }
#endif // WIN32 || _WIN64
template< > int    CmdLineStringToType( const char* str ){ return atoi( str ); }
template< > float  CmdLineStringToType( const char* str ){ return float( atof( str ) ); }
template< > double CmdLineStringToType( const char* str ){ return double( atof( str ) ); }
#if defined( WIN32 ) || defined( _WIN64 )
template< > char*  CmdLineStringToType( const char* str ){ return _strdup( str ); }
#else // !WIN32 && !_WIN64
template< > char*  CmdLineStringToType( const char* str ){ return  strdup( str ); }
#endif // WIN32 || _WIN64


/////////////////////
// CmdLineReadable //
/////////////////////
#if defined( WIN32 ) || defined( _WIN64 )
inline CmdLineReadable::CmdLineReadable( const char *name ) : set(false) { this->name = _strdup( name ); }
#else // !WIN32 && !_WIN64
inline CmdLineReadable::CmdLineReadable( const char *name ) : set(false) { this->name =  strdup( name ); }
#endif // WIN32 || _WIN64

inline CmdLineReadable::~CmdLineReadable( void ){ if( name ) free( name ) ; name = NULL; }
inline int CmdLineReadable::read( char** , int ){ set = true ; return 0; }
inline void CmdLineReadable::writeValue( char* str ) const { str[0] = 0; }

//////////////////////
// CmdLineParameter //
//////////////////////
template< class Type > CmdLineParameter< Type >::~CmdLineParameter( void ) { CmdLineCleanUp( &value ); }
template< class Type > CmdLineParameter< Type >::CmdLineParameter( const char *name ) : CmdLineReadable( name ){ value = CmdLineInitialize< Type >(); }
template< class Type > CmdLineParameter< Type >::CmdLineParameter( const char *name , Type v ) : CmdLineReadable( name ){ value = CmdLineCopy< Type >( v ); }
template< class Type >
int CmdLineParameter< Type >::read( char** argv , int argc )
{
	if( argc>0 )
	{
		CmdLineCleanUp< Type >( &value ) , value = CmdLineStringToType< Type >( argv[0] );
		set = true;
		return 1;
	}
	else return 0;
}
template< class Type >
void CmdLineParameter< Type >::writeValue( char* str ) const { CmdLineWriteValue< Type >( value , str ); }


///////////////////////////
// CmdLineParameterArray //
///////////////////////////
template< class Type , int Dim >
CmdLineParameterArray< Type , Dim >::CmdLineParameterArray( const char *name , const Type* v ) : CmdLineReadable( name )
{
	if( v ) for( int i=0 ; i<Dim ; i++ ) values[i] = CmdLineCopy< Type >( v[i] );
	else    for( int i=0 ; i<Dim ; i++ ) values[i] = CmdLineInitialize< Type >();
}
template< class Type , int Dim >
CmdLineParameterArray< Type , Dim >::~CmdLineParameterArray( void ){ for( int i=0 ; i<Dim ; i++ ) CmdLineCleanUp< Type >( values+i ); }
template< class Type , int Dim >
int CmdLineParameterArray< Type , Dim >::read( char** argv , int argc )
{
	if( argc>=Dim )
	{
		for( int i=0 ; i<Dim ; i++ ) CmdLineCleanUp< Type >( values+i ) , values[i] = CmdLineStringToType< Type >( argv[i] );
		set = true;
		return Dim;
	}
	else return 0;
}
template< class Type , int Dim >
void CmdLineParameterArray< Type , Dim >::writeValue( char* str ) const
{
	char* temp=str;
	for( int i=0 ; i<Dim ; i++ )
	{
		CmdLineWriteValue< Type >( values[i] , temp );
		temp = str+strlen( str );
	}
}
///////////////////////
// CmdLineParameters //
///////////////////////
template< class Type >
CmdLineParameters< Type >::CmdLineParameters( const char* name ) : CmdLineReadable( name ) , values(NULL) , count(0) { }
template< class Type >
CmdLineParameters< Type >::~CmdLineParameters( void )
{
	if( values ) delete[] values;
	values = NULL;
	count = 0;
}
template< class Type >
int CmdLineParameters< Type >::read( char** argv , int argc )
{
	if( values ) delete[] values;
	values = NULL;

	if( argc>0 )
	{
		count = atoi(argv[0]);
		if( count <= 0 || argc <= count ) return 1;
		values = new Type[count];
		if( !values ) return 0;
		for( int i=0 ; i<count ; i++ ) values[i] = CmdLineStringToType< Type >( argv[i+1] );
		set = true;
		return count+1;
	}
	else return 0;
}
template< class Type >
void CmdLineParameters< Type >::writeValue( char* str ) const
{
	char* temp=str;
	for( int i=0 ; i<count ; i++ )
	{
		CmdLineWriteValue< Type >( values[i] , temp );
		temp = str+strlen( str );
	}
}


inline char* FileExtension( char* fileName )
{
	char* temp = fileName;
	for( int i=0 ; i<strlen(fileName) ; i++ ) if( fileName[i]=='.' ) temp = &fileName[i+1];
	return temp;
}

inline char* GetFileExtension( const char* fileName )
{
	char* fileNameCopy;
	char* ext=NULL;
	char* temp;

	fileNameCopy=new char[strlen(fileName)+1];
	assert(fileNameCopy);
	strcpy(fileNameCopy,fileName);
	temp=strtok(fileNameCopy,".");
	while(temp!=NULL)
	{
		if(ext!=NULL){delete[] ext;}
		ext=new char[strlen(temp)+1];
		assert(ext);
		strcpy(ext,temp);
		temp=strtok(NULL,".");
	}
	delete[] fileNameCopy;
	return ext;
}
inline char* GetLocalFileName( const char* fileName )
{
	char* fileNameCopy;
	char* name=NULL;
	char* temp;

	fileNameCopy=new char[strlen(fileName)+1];
	assert(fileNameCopy);
	strcpy(fileNameCopy,fileName);
	temp=strtok(fileNameCopy,"\\");
	while(temp!=NULL){
		if(name!=NULL){delete[] name;}
		name=new char[strlen(temp)+1];
		assert(name);
		strcpy(name,temp);
		temp=strtok(NULL,"\\");
	}
	delete[] fileNameCopy;
	return name;
}
inline char* LocalFileName( char* fileName )
{
	char* temp = fileName;
	for( int i=0 ; i<(int)strlen(fileName) ; i++ ) if( fileName[i] =='\\' ) temp = &fileName[i+1];
	return temp;
}
inline char* DirectoryName( char* fileName )
{
	for( int i=int( strlen(fileName) )-1 ; i>=0 ; i-- )
		if( fileName[i] =='\\' )
		{
			fileName[i] = 0;
			return fileName;
		}
	fileName[0] = 0;
	return fileName;
}

inline void CmdLineParse( int argc , char **argv , CmdLineReadable** params )
{
	while( argc>0 )
	{
        if( argv[0][0]=='-' && argv[0][1]=='-' )
		{
			CmdLineReadable* readable=NULL;
			for( int i=0 ; params[i]!=NULL && readable==NULL ; i++ ) if( !strcasecmp( params[i]->name , argv[0]+2 ) ) readable = params[i];
			if( readable )
			{
				int j = readable->read( argv+1 , argc-1 );
				argv += j , argc -= j;
			}
			else
			{
				fprintf( stderr , "[WARNING] Invalid option: %s\n" , argv[0] );
				for( int i=0 ; params[i]!=NULL ; i++ ) printf( "\t--%s\n" , params[i]->name );
			}
		}
		else fprintf( stderr , "[WARNING] Parameter name should be of the form --<name>: %s\n" , argv[0] );
		++argv , --argc;
	}
}

inline char** ReadWords(const char* fileName,int& cnt)
{
	char** names;
	char temp[500];
	FILE* fp;

	fp=fopen(fileName,"r");
	if(!fp){return NULL;}
	cnt=0;
	while(fscanf(fp," %s ",temp)==1){cnt++;}
	fclose(fp);

	names=new char*[cnt];
	if(!names){return NULL;}

	fp=fopen(fileName,"r");
	if(!fp){
		delete[] names;
		cnt=0;
		return NULL;
	}
	cnt=0;
	while(fscanf(fp," %s ",temp)==1){
		names[cnt]=new char[strlen(temp)+1];
		if(!names){
			for(int j=0;j<cnt;j++){delete[] names[j];}
			delete[] names;
			cnt=0;
			fclose(fp);
			return NULL;
		}
		strcpy(names[cnt],temp);
		cnt++;
	}
	fclose(fp);
	return names;
}

