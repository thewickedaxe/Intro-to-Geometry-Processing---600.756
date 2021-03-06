#include "Helpers.cpp"
#include <string>

#include <omp.h>

#define NUM_THREADS 1 // Determined to be 3 for my pc by trial and error

// The command line parameters
CmdLineParameter< char* > In( "in" ) , Out( "out" ), BW( "bw" ), Iterations( "i" );
CmdLineReadable ASCII( "ascii" );

CmdLineReadable* cmdLineParameters[] = { &In, &Out , &ASCII , &BW, &Iterations, NULL };

void Usage( const char* ex )
{
	printf( "Usage %s:\n" , ex );
	printf( "\t --%s <input mesh>\n" , In.name );
	printf( "\t[--%s <output mesh>]\n", Out.name);
	printf( "\t[--%s <bledning weight> must be betweeen 0 and 1]\n", BW.name);
	printf( "\t[--%s <iterations> must be +ve]\n", Iterations.name);
	printf( "\t[--%s]\n" , ASCII.name );
}

void RefineMesh(
				std::vector< PlyVertex< float > > &plyVertices,
				std::vector< TriangleIndex > &triangles,
				std::map < int64_t, std::unordered_set< int64_t > > &associated_vertices,
				std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map
				) {
	Put_In_Map(site_index_map, index_site_map, plyVertices, triangles);
	Determine_New_Centers(site_index_map, index_site_map, plyVertices, triangles, associated_vertices);
}

void SmoothenMesh(
				std::map < int64_t, std::unordered_set< int64_t > > &associated_vertices,
                std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map,
                std::vector< PlyVertex< float > > &plyVertices,
                int iterations,
                float blending_weight
                ) {
    for (int j = 0; j < iterations; j++) {
		// Two clones of the site maps
		std::map < Point3D< float > , int64_t , PointCompare> site_index_clone = site_index_map;
		std::map < int64_t, Point3D< float > > index_site_clone = index_site_map;

		// Iterate over the indices, find neighboring vertices, blend
		int64_t i = 0;
		omp_lock_t writelock;
		omp_init_lock(&writelock);

		#pragma omp parallel num_threads(NUM_THREADS)
		{
			i = omp_get_thread_num();
			while(i < plyVertices.size()) {
				Point3D< float > point = index_site_clone[i];
				std::vector< Point3D < float > > neighbors;
				for (auto neighborindex : associated_vertices[i]) {
					neighbors.push_back(index_site_clone[neighborindex]);
				}
				Point3D< float > newpoint = blend(point, neighbors, blending_weight);
				omp_set_lock(&writelock);
				site_index_map.erase(point);
				index_site_map.erase(i);
				site_index_map[newpoint] = i;
				index_site_map[i] = newpoint;
				omp_unset_lock(&writelock);
				i += NUM_THREADS;
			}
		}
	}
	std::cout << "Size: " << index_site_map.size() << std::endl;
	plyVertices.clear();
	for (int i = 0; i < index_site_map.size(); i++) {
		plyVertices.push_back(index_site_map[i]);
	}
}

int main( int argc , char* argv[] )
{
	// Read the command line arguments
	CmdLineParse( argc-1 , argv+1 , cmdLineParameters );

	// Check that an input mesh has been specified
	if( !In.set || !BW.set || !Iterations.set)
	{
		Usage( argv[0] );
		return EXIT_FAILURE;
	}


	// Storage for the vertices and triangles of the mesh
	std::vector< PlyVertex< float > > plyVertices;
	std::vector< TriangleIndex > triangles;

	// A map of vertex to index
	std::map < Point3D< float > , int64_t , PointCompare> site_index_map;
	// A map of index to vertex
	std::map < int64_t, Point3D< float > > index_site_map;

	// A map of ints to sets to hold associated vertices
	std::map < int64_t, std::unordered_set< int64_t > > associated_vertices;

	// This read function takes:
	// -- The input file name
	// -- A reference to a vector of vertices
	// -- A reference to a vector of triangles
	// -- The number of properties stored with a vertex
	// -- A (writeable) array to indicate which properties were successfully read
	// -- A description of the vertex properties
	PlyReadTriangles( In.value , plyVertices , triangles , PlyVertex< float >::ReadProperties , NULL , PlyVertex< float >::ReadComponents );
	printf( "Read mesh: %d vertices , %d triangles\n" , (int)plyVertices.size() , (int)triangles.size() );
	std::cout << "Blending Weight: " << BW.value << std::endl;
	std::cout << "Iterations: " << Iterations.value << std::endl;


	RefineMesh(plyVertices, triangles, associated_vertices, site_index_map, index_site_map);
	std::cout << "Size: " << index_site_map.size() << std::endl;
	std::cout << "Size: " << index_site_map.size() << std::endl;
	SmoothenMesh(associated_vertices,
			     site_index_map,
				 index_site_map,
				 plyVertices,
				 std::stoi(Iterations.value),
				 std::stof(BW.value));

	Timer timer;

	// Compute the bounding-box containing the shape
	Point3D< float > min = plyVertices[0].position , max = plyVertices[0].position;
	for( int i=1 ; i<plyVertices.size() ; i++ ) for( int d=0 ; d<3 ; d++ ) min[d] = std::min< float >( min[d] , plyVertices[i].position[d] ) , max[d] = std::max< float >( max[d] , plyVertices[i].position[d] );

	printf( "Elapsed: %.2f(s)\n" , timer.elapsed() );

	printf( "Bounding Box: [ %.2f , %.2f ] x [ %.2f , %.2f ] x [ %.2f , %.2f ]\n" , min[0] , max[0] , min[1] , max[1] , min[2] , max[2] );

	if( Out.set ) PlyWriteTriangles( Out.value , plyVertices , triangles , PlyVertex< float >::WriteProperties , PlyVertex< float >::WriteComponents , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );

	return EXIT_SUCCESS;
}