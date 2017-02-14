#include "Helpers.cpp"

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

void RefineMesh(std::vector< PlyVertex< float > > &plyVertices, std::vector< TriangleIndex > &triangles) {	
	// A map of vertex to index
	std::map < Point3D< float > , int64_t , PointCompare> site_index_map;
	// A map of index to vertex
	std::map < int64_t, Point3D< float > > index_site_map;

	Put_In_Map(site_index_map, index_site_map, plyVertices, triangles);
	Determine_New_Centers(site_index_map, index_site_map, plyVertices, triangles);
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
	
	// Allow for mesh smoothing and refinement to run independently
	std::vector< PlyVertex< float > > plyVertices_clone;
	std::vector< TriangleIndex > triangles_clone;

	// This read function takes:
	// -- The input file name
	// -- A reference to a vector of vertices
	// -- A reference to a vector of triangles
	// -- The number of properties stored with a vertex
	// -- A (writeable) array to indicate which properties were successfully read
	// -- A description of the vertex properties
	PlyReadTriangles( In.value , plyVertices , triangles , PlyVertex< float >::ReadProperties , NULL , PlyVertex< float >::ReadComponents );
	printf( "Read mesh: %d vertices , %d triangles\n" , (int)plyVertices.size() , (int)triangles.size() );

	RefineMesh(plyVertices, triangles);

	Timer timer;

	// Compute the bounding-box containing the shape
	Point3D< float > min = plyVertices[0].position , max = plyVertices[0].position;
	for( int i=1 ; i<plyVertices.size() ; i++ ) for( int d=0 ; d<3 ; d++ ) min[d] = std::min< float >( min[d] , plyVertices[i].position[d] ) , max[d] = std::max< float >( max[d] , plyVertices[i].position[d] );

	printf( "Elapsed: %.2f(s)\n" , timer.elapsed() );

	printf( "Bounding Box: [ %.2f , %.2f ] x [ %.2f , %.2f ] x [ %.2f , %.2f ]\n" , min[0] , max[0] , min[1] , max[1] , min[2] , max[2] );

	if( Out.set ) PlyWriteTriangles( Out.value , plyVertices , triangles , PlyVertex< float >::WriteProperties , PlyVertex< float >::WriteComponents , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );

	return EXIT_SUCCESS;
}