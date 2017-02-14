/*

   Header for PLY polygon files.

   - Greg Turk, March 1994

   A PLY file contains a single polygonal _object_.

   An object is composed of lists of _elements_.  Typical elements are
   vertices, faces, edges and materials.

   Each type of element for a given object has one or more _properties_
   associated with the element type.  For instance, a vertex element may
   have as properties three floating-point values x,y,z and three unsigned
   chars for red, green and blue.

   ---------------------------------------------------------------

   Copyright (c) 1994 The Board of Trustees of The Leland Stanford
   Junior University.  All rights reserved.   

   Permission to use, copy, modify and distribute this software and its   
   documentation for any purpose is hereby granted without fee, provided   
   that the above copyright notice and this permission notice appear in   
   all copies of this software and that you do not sell the software.   

   THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,   
   EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY   
   WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.   

*/

// JP: Disable the "deprecated conversion from string constant to char *"
#if !defined( WIN32 ) && !defined( _WIN64 )
#pragma GCC diagnostic ignored "-Wwrite-strings"
#endif // !WIN32 && !_WIN64

#ifndef PLY_INCLUDED
#define PLY_INCLUDED
#include <vector>
#include "Geometry.h"
#include "PlyFile.h"

template< class Real >
class PlyVertex : public VectorSpace< Real , PlyVertex< Real > >
{
public:
	//////////////////////////
	// Vector space methods //
	void Add	(const PlyVertex& p)	{	position += p.position;	}
	void Scale	(float s)				{	position *= s;			}
	//////////////////////////

	const static int ReadComponents=3;
	const static int WriteComponents=3;
	static PlyProperty ReadProperties[];
	static PlyProperty WriteProperties[];

	Point3D< Real > position;

	operator Point3D< Real >& ()					{ return position; }
	operator const Point3D< Real >& () const		{ return position; }
	template< class Real2 > operator Point3D< Real2 > ( ) const { return Point3D< Real2 >( position ); }
	PlyVertex( void )								{ }
	PlyVertex( const Point3D< Real >& p )			{ position = p; }
};
template<>
PlyProperty PlyVertex< float >::ReadProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyVertex< float >::WriteProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,position.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyVertex< double >::ReadProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyVertex< double >::WriteProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,position.coords[2])), 0, 0, 0, 0}
};

template< class Real >
class PlyOrientedVertex : public VectorSpace< Real , PlyOrientedVertex< Real > >
{
public:
    typedef Real real_type;

	//////////////////////////
	// Vector space methods //
	void Add	(const PlyOrientedVertex& p)	{	position += p.position;	normal += p.normal;	}
	void Scale	( Real s)						{	position *= s;			normal *= s;			}
	//////////////////////////
	const static int ReadComponents=6;
	const static int WriteComponents=6;
	static PlyProperty ReadProperties[];
	static PlyProperty WriteProperties[];

	Point3D< Real > position , normal;

	operator Point3D<Real>& ()					{return position;}
	operator const Point3D<Real>& () const		{return position;}
	template< class Real2 > operator Point3D< Real2 > ( ) const { return Point3D< Real2 >( position ); }
	PlyOrientedVertex(void)						{ }
	PlyOrientedVertex(const Point3D<Real>& p)	{ position=p; }
};
template<>
PlyProperty PlyOrientedVertex< float >::ReadProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyOrientedVertex< float >::WriteProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,position.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyOrientedVertex< double >::ReadProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyOrientedVertex< double >::WriteProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,position.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};

template< class Real >
class PlyColorVertex : public VectorSpace< Real , PlyColorVertex< Real > >
{
public:
	//////////////////////////
	// Vector space methods //
	void Add	(const PlyColorVertex& p)	{	position += p.position;	color+=p.color;	}
	void Scale	(Real s)					{	position *= s;			color*=s;		}
	//////////////////////////
	const static int ReadComponents=9;
	const static int WriteComponents=6;
	static PlyProperty ReadProperties[];
	static PlyProperty WriteProperties[];

	Point3D<Real> position,color;

	operator Point3D<Real>& ()					{return position;}
	operator const Point3D<Real>& () const		{return position;}
	template< class Real2 > operator Point3D< Real2 > ( ) const { return Point3D< Real2 >( position ); }
	PlyColorVertex(void)						{ }
	PlyColorVertex(const Point3D<Real>& p)		{ position=p; }
};
template<>
PlyProperty PlyColorVertex< float >::ReadProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0},
	{"diffuse_red",		PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"diffuse_green",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"diffuse_blue",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0},
};
template<>
PlyProperty PlyColorVertex< float >::WriteProperties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,position.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_FLOAT, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0}
};
template<>
PlyProperty PlyColorVertex< double >::ReadProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0},
	{"diffuse_red",		PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"diffuse_green",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"diffuse_blue",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0},
};
template<>
PlyProperty PlyColorVertex< double >::WriteProperties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,position.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[0])),0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[1])),0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_DOUBLE, int(offsetof(PlyColorVertex,color.coords[2])),0, 0, 0, 0}
};

///////////////////////////////////////////////////////////////////

int PlyDefaultFileType( void ){ return PLY_ASCII; }

template< class Vertex >
int PlyReadTriangles( const char* fileName ,
					  std::vector<Vertex>& vertices , std::vector< TriangleIndex >& triangles ,
					  PlyProperty* properties , bool* propertyFlags , int propertyNum ,
					  char*** comments=NULL , int* commentNum=NULL );

template< class Vertex >
int PlyWriteTriangles( const char* fileName ,
					   const std::vector<Vertex>& vertices , const std::vector< TriangleIndex >& triangles ,
					   PlyProperty* properties , int propertyNum ,
					   int file_type = PlyDefaultFileType() ,
					   char** comments=NULL , const int& commentNum=0);

#include "Ply.inl"

#endif // PLY_INCLUDED
