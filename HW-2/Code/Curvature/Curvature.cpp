#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <unordered_map>

#define SOLUTION

#ifdef _OPENMP
#include <omp.h>
#else // !_OPENMP
int  omp_get_num_procs  ( void ){ return 1; }
int  omp_get_max_threads( void ){ return 1; }
int  omp_get_thread_num ( void ){ return 0; }
void omp_set_num_threads( int  ){ }
void omp_set_num_threads( int  ){ }
#endif // _OPENMP

#include "Util/Ply.h"
#include "Util/Geometry.h"
#include "Util/CmdLineParser.h"
#include "Util/Timer.h"

// The command line parameters
enum
{
	CURVATURE_MEAN ,
	CURVATURE_GAUSSIAN ,
	CURVATURE_UMBILIC ,
	CURVATURE_FULL ,
	CURVATURE_COUNT
};
const char* CurvatureNames[] = { "mean" , "Gaussian" , "umbilic" , "full" };

CmdLineParameter< char* > In( "in" ) , Out( "out" );
CmdLineParameter< int > CurvatureType( "k" , CURVATURE_MEAN ) , Refine( "refine" , 0 );
CmdLineReadable ASCII( "ascii" );

CmdLineReadable* cmdLineParameters[] = { &In, &Out , &CurvatureType , &Refine , &ASCII , NULL };

void Usage( const char* ex )
{
	printf( "Usage %s:\n" , ex );
	printf( "\t --%s <input mesh>\n" , In.name );
	printf( "\t[--%s <output mesh>]\n", Out.name );
	printf( "\t[--%s <curvature type>=%d]\n" , CurvatureType.name , CurvatureType.value );
	for( int i=0 ; i<CURVATURE_COUNT ; i++ ) printf( "\t\t%d] %s\n" , i , CurvatureNames[i] );
	printf( "\t[--%s <1-to-4 refinement iterations>=%d]\n" , Refine.name , Refine.value );
	printf( "\t[--%s]\n" , ASCII.name );
}

struct CurvatureInfo
{
	CurvatureInfo( void ) : k1(0) , k2(0) { }
	float k1 , k2;
	Point3D< float > dir1 , dir2;
	static const int Components = 8;
	static PlyProperty Properties[];
	CurvatureInfo normalize( void ) const
	{
		CurvatureInfo ci;
		ci.k1 = k1 , ci.k2 = k2;
		ci.dir1 = dir1 / (float)sqrt( Point3D< float >::SquareNorm( dir1 ) );
		ci.dir2 = dir2 / (float)sqrt( Point3D< float >::SquareNorm( dir2 ) );
		return ci;
	}
};
PlyProperty CurvatureInfo::Properties[] =
{
	{ "x1" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir1.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
	{ "y1" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir1.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
	{ "z1" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir1.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
	{ "x2" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir2.coords[0] ) ) , 0 , 0 , 0 , 0 } ,
	{ "y2" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir2.coords[1] ) ) , 0 , 0 , 0 , 0 } ,
	{ "z2" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , dir2.coords[2] ) ) , 0 , 0 , 0 , 0 } ,
	{ "k1" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , k1 ) ) , 0 , 0 , 0 , 0 } ,
	{ "k2" , PLY_FLOAT , PLY_FLOAT , int(offsetof( CurvatureInfo , k2 ) ) , 0 , 0 , 0 , 0 }
};


void RefineMesh( std::vector< PlyOrientedVertex< float > >& plyVertices , std::vector< TriangleIndex >& triangles )
{
	std::vector< TriangleIndex > newTriangles;
	std::vector< PlyOrientedVertex< float > > newPlyVertices;
	newPlyVertices.reserve( plyVertices.size()*4 );
	newTriangles.reserve( triangles.size()*4 );

	auto MapIndex = []( unsigned int v1 , unsigned int v2 )
	{
		if( v1<v2 ) return ( (unsigned long long)v1<<32 ) | (unsigned long long)v2;
		else        return ( (unsigned long long)v2<<32 ) | (unsigned long long)v1;
	};

	std::unordered_map< unsigned long long , int > vMap;
	for( int i=0 ; i<triangles.size() ; i++ )
	{
		unsigned int v[] = { triangles[i][0] , triangles[i][1] , triangles[i][2] , triangles[i][0] };
		int vIdx[3] , eIdx[3];
		// Add vertices to the map
		for( int j=0 ; j<3 ; j++ )
		{
			long long idx;

			// Old vertices
			idx = MapIndex( v[j] , v[j] );
			if( vMap.find( idx )==vMap.end() ) vMap[idx] = (int)newPlyVertices.size() , newPlyVertices.push_back( plyVertices[v[j]] );
			vIdx[j] = vMap[idx];

			// New vertices
			idx = MapIndex( v[j] , v[j+1] );
			if( vMap.find( idx )==vMap.end() )
			{
				vMap[idx] = (int)newPlyVertices.size() , newPlyVertices.push_back( ( plyVertices[ v[j] ] + plyVertices[ v[j+1] ] )/2 );
				newPlyVertices.back().normal /= (float)sqrt( Point3D< float >::SquareNorm( newPlyVertices.back().normal ) );
			}
			eIdx[j] = vMap[idx];
		}

		// Add the triangles
		newTriangles.push_back( TriangleIndex( eIdx[0] , eIdx[1] , eIdx[2] ) );
		newTriangles.push_back( TriangleIndex( vIdx[0] , eIdx[0] , eIdx[2] ) );
		newTriangles.push_back( TriangleIndex( vIdx[1] , eIdx[1] , eIdx[0] ) );
		newTriangles.push_back( TriangleIndex( vIdx[2] , eIdx[2] , eIdx[1] ) );
	}
	triangles = newTriangles;
	plyVertices = newPlyVertices;
}

void SetNormals( std::vector< PlyOrientedVertex< float > >& plyVertices , const std::vector< TriangleIndex >& triangles )
{
	// Compute the vertex normals
}

std::vector< float > GetCurvatures( const std::vector< PlyOrientedVertex< float > >& plyVertices , const std::vector< TriangleIndex >& triangles , int curvatureType )
{
	std::vector< float > vCurvatures( plyVertices.size() , 0 ) , tCurvatures( triangles.size() , 0 );

	// Compute the mean/Gaussian curvatures per triangle (tCurvatures)
	{
#pragma message ( "[WARNING] Missing code here" )
	}
	// Average the per-triangle curvature values into the vertices (tCurvatures -> vCurvatures)
	{
#pragma message ( "[WARNING] Missing code here" )
	}
	return vCurvatures;
}

std::vector< CurvatureInfo > GetCurvatures( const std::vector< PlyOrientedVertex< float > >& plyVertices , const std::vector< TriangleIndex >& triangles )
{
	std::vector< CurvatureInfo > curvatures( triangles.size() );

	// Compute the principal curvature directions and values
	// (The first principal curvature value/direction should be the one with the largest value)
#pragma message ( "[WARNING] Missing code here" )

	return curvatures;
}

int main( int argc , char* argv[] )
{
	// Read the command line arguments
	CmdLineParse( argc-1 , argv+1 , cmdLineParameters );

	// Check that an input mesh has been specified
	if( !In.set )
	{
		Usage( argv[0] );
		return EXIT_FAILURE;
	}

	// Storage for the vertices and triangles of the mesh
	std::vector< PlyOrientedVertex< float > > plyVertices;
	std::vector< TriangleIndex > triangles;

	{
		// Flags for tracking which ply parameters were read in
		bool vFlags[ PlyOrientedVertex< float >::ReadComponents ];
		PlyReadTriangles( In.value , plyVertices , triangles , PlyOrientedVertex< float >::ReadProperties , vFlags , PlyOrientedVertex< float >::ReadComponents );

		// Check if normals were provided
		if( !vFlags[3] || !vFlags[4] || !vFlags[5] ) SetNormals( plyVertices , triangles );

		// Just in case they aren't, make sure the normals have unit length
		for( int i=0 ; i<plyVertices.size() ; i++ ) plyVertices[i].normal /= (float)sqrt( Point3D< float >::SquareNorm( plyVertices[i].normal ) );

		// Refine the mesh
		for( int r=0 ; r<Refine.value ; r++ ) RefineMesh( plyVertices , triangles );

		printf( "Read mesh: %d vertices , %d triangles\n" , (int)plyVertices.size() , (int)triangles.size() );
	}

	// Compute the curvatures
	if( CurvatureType.value==CURVATURE_FULL )
	{
		Timer timer;
		std::vector< CurvatureInfo > curvatures = GetCurvatures( plyVertices , triangles );
		printf( "Elapsed: %.2f(s)\n" , timer.elapsed() );

		// Output the curvatures as color values
		if( Out.set )
		{
			std::vector< TriangleIndexWithData< CurvatureInfo > > _triangles( triangles.size() );
			for( int i=0 ; i<triangles.size() ; i++ )
			{
				for( int j=0 ; j<3 ; j++ ) _triangles[i][j] = triangles[i][j];
				_triangles[i].data = curvatures[i].normalize();
			}
			PlyWriteTriangles< PlyOrientedVertex< float > , CurvatureInfo >( Out.value , plyVertices , _triangles , PlyVertex< float >::WriteProperties , PlyVertex< float >::WriteComponents , CurvatureInfo::Properties , CurvatureInfo::Components , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );
		}
	}
	else
	{
		Timer timer;
		std::vector< float > curvatures = GetCurvatures( plyVertices , triangles , CurvatureType.value );
		printf( "Elapsed: %.2f(s)\n" , timer.elapsed() );

		// Output the curvatures as color values
		if( Out.set )
		{
			// The lambda function turning curvature into color
			auto Color = []( float v )
			{
				// Clamp the curvature value to the range [-1,1]
				float s = std::max< float >( 0.f , std::min< float >( 1.f , (float)fabs( v ) ) );
				Point3D< float > gray( 192.f , 192.f , 192.f ) , red( 255.f , 0.f , 0.f ) , blue( 0.f , 0.f , 255.f );
				if( v<0 ) return gray * (1.f-s) +  red * s;
				else      return gray * (1.f-s) + blue * s; 
			};

			// Normalize the curvature to within a few standard deviations
			float scale = 0;
			if( CurvatureType.value!=CURVATURE_GAUSSIAN )
			{
				for( int i=0 ; i<curvatures.size() ; i++ ) scale += curvatures[i]*curvatures[i];
				scale = (float)sqrt( scale / curvatures.size() ) * 8;
			}
			else
			{
				for( int i=0 ; i<curvatures.size() ; i++ ) scale += fabs( curvatures[i] );
				scale = scale / curvatures.size();
			}

			std::vector< PlyColorVertex< float > > _plyVertices( plyVertices.size() );
			for( int i=0 ; i<plyVertices.size() ; i++ )
			{
				_plyVertices[i].position = plyVertices[i].position;
				_plyVertices[i].color = Color( 16.f * curvatures[i] / scale );
			}
			PlyWriteTriangles( Out.value , _plyVertices , triangles , PlyColorVertex< float >::WriteProperties , PlyColorVertex< float >::WriteComponents , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );
		}
	}
	return EXIT_SUCCESS;
}
