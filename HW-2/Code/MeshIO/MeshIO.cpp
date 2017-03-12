#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

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
CmdLineParameter< char* > In( "in" ) , Out( "out" );
CmdLineReadable ASCII( "ascii" );

CmdLineReadable* cmdLineParameters[] = { &In, &Out , &ASCII , NULL };

void Usage( const char* ex )
{
	printf( "Usage %s:\n" , ex );
	printf( "\t --%s <input mesh>\n" , In.name );
	printf( "\t[--%s <output mesh>]\n", Out.name);
	printf( "\t[--%s]\n" , ASCII.name );
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
	std::vector< PlyVertex< float > > plyVertices;
	std::vector< TriangleIndex > triangles;

	// This read function takes:
	// -- The input file name
	// -- A reference to a vector of vertices
	// -- A reference to a vector of triangles
	// -- The number of properties stored with a vertex
	// -- A (writeable) array to indicate which properties were successfully read
	// -- A description of the vertex properties
	PlyReadTriangles( In.value , plyVertices , triangles , PlyVertex< float >::ReadProperties , NULL , PlyVertex< float >::ReadComponents );
	printf( "Read mesh: %d vertices , %d triangles\n" , (int)plyVertices.size() , (int)triangles.size() );

	Timer timer;

	// Compute the bounding-box containing the shape
	Point3D< float > min = plyVertices[0].position , max = plyVertices[0].position;
	for( int i=1 ; i<plyVertices.size() ; i++ ) for( int d=0 ; d<3 ; d++ ) min[d] = std::min< float >( min[d] , plyVertices[i].position[d] ) , max[d] = std::max< float >( max[d] , plyVertices[i].position[d] );

	printf( "Elapsed: %.2f(s)\n" , timer.elapsed() );

	printf( "Bounding Box: [ %.2f , %.2f ] x [ %.2f , %.2f ] x [ %.2f , %.2f ]\n" , min[0] , max[0] , min[1] , max[1] , min[2] , max[2] );

	if( Out.set ) PlyWriteTriangles( Out.value , plyVertices , triangles , PlyVertex< float >::WriteProperties , PlyVertex< float >::WriteComponents , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );

	return EXIT_SUCCESS;
}