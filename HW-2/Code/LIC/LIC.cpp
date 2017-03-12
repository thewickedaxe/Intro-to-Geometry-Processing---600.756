#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <unordered_map>

#define USE_EIGEN

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
#include "Util/LinearSolvers.h"
#include "Util/FEM.h"

CmdLineParameter< char* > In( "in" ) , Out( "out" );
CmdLineParameter< float > StepSize( "stepSize" , 1e-7f ) , Brighten( "brighten" , 1.f );
CmdLineReadable Smallest( "smallest" ) , Verbose( "verbose" ) , ASCII( "ascii" );

void Usage( const char* ex )
{
	printf( "Usage %s:\n" , ex );
	printf( "\t --%s <input mesh>\n" , In.name );
	printf( "\t[--%s <output mesh>]\n" , Out.name );
	printf( "\t[--%s <step size>=%g]\n" , StepSize.name , StepSize.value );
	printf( "\t[--%s <brightness multiplier>=%f]\n" , Brighten.name , Brighten.value );
	printf( "\t[--%s]\n" , Smallest.name );
	printf( "\t[--%s]\n" , Verbose.name );
	printf( "\t[--%s]\n" , ASCII.name );
}
CmdLineReadable* cmdLineParameters[] = { &In , &Out , &StepSize , &Brighten , &Smallest , &Verbose , &ASCII , NULL };

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif // M_PI
template< class Real >
void HSV2RGB( const Real* _hsv , Real* rgb )
{
	Real hsv[3];
	hsv[0] = _hsv[0];
	hsv[1] = _hsv[1];
	hsv[2] = _hsv[2];
	int i;
	Real f, p, q, t;
	if( hsv[1] == 0 )
	{
		rgb[0] = rgb[1] = rgb[2] = hsv[2];
		return;
	}
	else if( hsv[1]<0 )
	{
		fprintf( stderr , "[Error] Saturation can't be negative\n" );
		return;
	}
	while( hsv[0]<0 ) hsv[0] += (Real)( 2. * M_PI );
	hsv[0] /= (Real)( M_PI / 3. );
	i = (int)floor( hsv[0] );
	f = (Real)(hsv[0] - i);
	p = (Real)(hsv[2] * ( 1 - hsv[1] ));
	q = (Real)(hsv[2] * ( 1 - hsv[1] * f ));
	t = (Real)(hsv[2] * ( 1 - hsv[1] * ( 1 - f ) ));
	switch( i ) {
	case 0:
		rgb[0] = hsv[2] , rgb[1] = t , rgb[2] = p;
		break;
	case 1:
		rgb[0] = q , rgb[1] = hsv[2] , rgb[2] = p;
		break;
	case 2:
		rgb[0] = p , rgb[1] = hsv[2] , rgb[2] = t;
		break;
	case 3:
		rgb[0] = p , rgb[1] = q , rgb[2] = hsv[2];
		break;
	case 4:
		rgb[0] = t , rgb[1] = p , rgb[2] = hsv[2];
		break;
	default:
		rgb[0] = hsv[2] , rgb[1] = p , rgb[2] = q;
		break;
	}
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
	std::vector< PlyColorVertex< float > > plyVertices;
	std::vector< TriangleIndex > triangles;
	std::vector< CurvatureInfo > curvatures;

	{
		// Flags for tracking which ply parameters were read 
		std::vector< TriangleIndexWithData< CurvatureInfo > > _triangles;
		bool dFlags[ CurvatureInfo::Components ];
		PlyReadTriangles( In.value , plyVertices , _triangles , PlyVertex< float >::ReadProperties , NULL , PlyVertex< float >::ReadComponents , CurvatureInfo::Properties , dFlags , CurvatureInfo::Components );
		for( int i=0 ; i<CurvatureInfo::Components ; i++ ) if( !dFlags[i] ) fprintf( stderr , "[ERROR] Failed to read curvature into component %d\n" , i ) , exit( 0 );

		// Just in case they aren't, make sure the principal curvature directions have unit 
		triangles.resize( _triangles.size() );
		curvatures.resize( _triangles.size() );
		for( int i=0 ; i<_triangles.size() ; i++ )
		{
			for( int j=0 ; j<3 ; j++ ) triangles[i][j] = _triangles[i][j];
			curvatures[i] = _triangles[i].data.normalize();
		}
		printf( "Read mesh: %d vertices , %d triangles\n" , (int)plyVertices.size() , (int)triangles.size() );
	}

	// Initialize the vertices with a random color
	for( int i=0 ; i<plyVertices.size() ; i++ )
	{
		rand() , rand() , rand();
		Point3D< float > hsv( (float)( Random< double >() *2. * M_PI ) , 1.f , 1.f ) , rgb;
		HSV2RGB( &hsv[0] , &rgb[0] );
		plyVertices[i].color = rgb * 255.f;
	}

	// Construct the Riemannian mesh
	FEM::RiemannianMesh< double > mesh( GetPointer( triangles ) , triangles.size() );
	{
		Timer t;
		for( int i=0 ; i<triangles.size() ; i++ )
		{
			// The tangent directions
			Point3D< double > t[] = { Point3D< double >( plyVertices[ triangles[i][1] ].position - plyVertices[ triangles[i][0] ].position  ) , Point3D< double >( plyVertices[ triangles[i][2] ].position - plyVertices[ triangles[i][0] ].position  ) };

			// Compute the coefficients of the tangent directions in the basis of the principal curvature directions
			Point2D< double > _t[2];
			for( int j=0 ; j<2 ; j++ ) _t[j] = Point2D< double >( Point3D< double >::Dot( t[j] , Point3D< double >( curvatures[i].dir1 ) ) , Point3D< double >::Dot( t[j] , Point3D< double >( curvatures[i].dir2 ) ) ); 

			// Scale the coefficients
			if( Smallest.set ) for( int j=0 ; j<2 ; j++ ) _t[j][1] = 0 , _t[j][0] *= fabs( curvatures[i].k1 - curvatures[i].k2 );
			else               for( int j=0 ; j<2 ; j++ ) _t[j][0] = 0 , _t[j][1] *= fabs( curvatures[i].k1 - curvatures[i].k2 );

			SquareMatrix< double , 2 > g;
			for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ ) g( j , k ) = Point2D< double >::Dot( _t[j] , _t[k] );
			mesh.g(i) = mesh.g(i) * 1e-8 + g;
		}
		mesh.makeUnitArea();
		if( Verbose.set ) printf( "Set metric: %.1f(s)\n" , t.elapsed() );
	}

	// Compute the diffusion system
	SparseMatrix< double , int > A , M , S;
	{
		Timer t;
		M = mesh.template massMatrix< FEM::BASIS_0_WHITNEY >();
		S = mesh.template stiffnessMatrix< FEM::BASIS_0_WHITNEY >();
		A = M + S * StepSize.value;
		if( Verbose.set ) printf( "Set system matrix: %.1f(s)\n" , t.elapsed() );
	}

	// Solve the system
	{
		Timer t;
		EigenSolverCholeskyLLt< double , ConstPointer( MatrixEntry< double , int > ) > solver( A );
		if( Verbose.set ) printf( "Factored system matrix: %.1f(s)\n" , t.elapsed() );

		Pointer( double ) b = AllocPointer< double >( plyVertices.size() );
		Pointer( double ) x = AllocPointer< double >( plyVertices.size() );

		t.reset();
		for( int c=0 ; c<3 ; c++ )
		{
			for( int i=0 ; i<plyVertices.size() ; i++ ) x[i] = plyVertices[i].color[c];
			M.Multiply( x , b );
			solver.solve( b , x );
			for( int i=0 ; i<plyVertices.size() ; i++ ) plyVertices[i].color[c] = (float)x[i];
		}
		if( Verbose.set ) printf( "Solved: %.1f(s)\n" , t.elapsed() );
		FreePointer( b );
		FreePointer( x );
		if( Brighten.set ) for( int i=0 ; i<plyVertices.size() ; i++ ) for( int c=0 ; c<3 ; c++ ) plyVertices[i].color[c] = std::min< float >( 255.f , std::max< float >( 0.f , 128.f + ( plyVertices[i].color[c] - 128.f )* Brighten.value ) );
	}
	if( Out.set ) PlyWriteTriangles( Out.value , plyVertices , triangles , PlyColorVertex< float >::ReadProperties , PlyColorVertex< float >::ReadComponents , ASCII.set ? PLY_ASCII : PLY_BINARY_NATIVE );
	return EXIT_SUCCESS;
}