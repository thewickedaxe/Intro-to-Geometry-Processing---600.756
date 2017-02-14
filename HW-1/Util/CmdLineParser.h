/*
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

#ifndef CMD_LINE_PARSER_INCLUDED
#define CMD_LINE_PARSER_INCLUDED
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

struct CmdLineReadable;

// Reads the command line and sets the parameter values.
// Inputs are assumed to be of the form --<parameter name> <parameter value(s)...>
// The parameter list must be NULL-terminated
void CmdLineParse( int argc , char **argv, CmdLineReadable** cmdLineParameters );

// The base command-line readable object class
struct CmdLineReadable
{
	bool set;										// Indicates if the parameter has been set
	char *name;										// The name of the parameter
	CmdLineReadable( const char *name );			// Constructs an instance with the specified name
	virtual ~CmdLineReadable( void );
	virtual int read( char** argv , int argc );		// Reads the arguments from the parameter list and returns the number of objects read
	virtual void writeValue( char* str ) const;		// Writes the value(s) (if any) to the string
};

// A readable object that takes a value
template< class Type >
struct CmdLineParameter : public CmdLineReadable
{
	Type value;										// The value of the object
	CmdLineParameter( const char *name );			// Constructs an instance with the specified name (value un-initialized beyond the default initialization)
	CmdLineParameter( const char *name , Type v );	// Constructs an instance with the specified name and value
	~CmdLineParameter( void );
	int read( char** argv , int argc );
	void writeValue( char* str ) const;
};

// A readable object that takes a fixed number of values
template< class Type , int Dim >
struct CmdLineParameterArray : public CmdLineReadable
{
	Type values[Dim];												// The values of the object
	CmdLineParameterArray( const char *name, const Type* v=NULL );	// Constructs an instance with the specified name (and copies value if non-NULL)
	~CmdLineParameterArray( void );
	int read( char** argv , int argc );
	void writeValue( char* str ) const;
};

// A readable object that takes a multiple number of values [the number is the first parameter]
template< class Type >
struct CmdLineParameters : public CmdLineReadable
{
	int count;								// The number of values
	Type *values;							// The values themselves
	CmdLineParameters( const char* name );	// Constructs and instance with the specified name
	~CmdLineParameters( void );
	int read( char** argv , int argc );
	void writeValue( char* str ) const;
};

#ifdef WIN32
int strcasecmp( char* c1 , char* c2 );
#endif // WIN32
void CmdLineParse( int argc , char **argv, CmdLineReadable** params );
char* FileExtension( char* fileName );
char* LocalFileName( char* fileName );
char* DirectoryName( char* fileName );
char* GetFileExtension( const char* fileName );
char* GetLocalFileName( const char* fileName );
char** ReadWords( const char* fileName , int& cnt );

#include "CmdLineParser.inl"
#endif // CMD_LINE_PARSER_INCLUDED
