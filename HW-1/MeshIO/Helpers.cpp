#include "Util/Ply.h"
#include "Util/Geometry.h"
#include "Util/CmdLineParser.h"
#include "Util/Timer.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <iostream>
#include <map>

#include <omp.h>

#define TRIANGLE_VERTEX_COUNT 3
#define NUM_THREADS 1 // Determined to be 4 for my pc by trial and error

bool areEqualRel(float a, float b, float epsilon) {
    return (fabs(a - b) <= epsilon * std::max(1.0f, std::max(a, b)));
}
struct PointCompare
{
   bool operator() (const Point3D< float >& lhs, const Point3D< float >& rhs) const
   {
       if (lhs[0] < rhs[0]) {
           return true;
       } else if(lhs[0] == rhs[0]) {
           if (lhs[1] < rhs[1]) {
               return true;
           } else if (lhs[1] == rhs[1]) {
               if (lhs[2] < rhs[2]) {
                   return true;
               } else {
                   return false;
               }
           } else {
               return false;
           }
       } else {
           return false;
       }
   }
};

void IdentifyVertex(const Point3D< float > point, const int64_t index = -1) {
    if (index != -1) {
        printf("Vertex %d: [%f, %f, %f]\n", index, point[0], point[1], point[2]);
    } else {
        printf("Vertex: [%f, %f, %f]\n", point[0], point[1], point[2]);
    }
}

void IdentifyTriangle(TriangleIndex t,
                      std::map < int64_t, Point3D< float > > &index_site_map) {
    IdentifyVertex(index_site_map[t[0]], t[0]);
    IdentifyVertex(index_site_map[t[1]], t[1]);
    IdentifyVertex(index_site_map[t[2]], t[2]);
    std::cout << "---\n";
}

Point3D< float > Determine_Midpoint(Point3D< float > p1, Point3D< float > p2) {
    Point3D< float > ans;
    ans[0] = (p1[0] + p2[0]) / 2;
    ans[1] = (p1[1] + p2[1]) / 2;
    ans[2] = (p1[2] + p2[2]) / 2;
    return ans;
}

void Safe_Insert_Vertex(
    std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
    std::map < int64_t, Point3D< float > > &index_site_map,
    Point3D< float > point
    ) {
    int64_t i = site_index_map.size();
    if (site_index_map.count(point) == 0) {
        site_index_map[point] = i;
        index_site_map[i] = point;
    }
}

int find_site_index(Point3D < float > point, std::map < int64_t, Point3D< float > > index_site_map) {
    for (int i = 0; i < index_site_map.size(); i++) {
        Point3D <float> comp_point = index_site_map[i];
        if (
            (areEqualRel(comp_point[0], point[0], FLT_EPSILON))&&
            (areEqualRel(comp_point[1], point[1], FLT_EPSILON))&&
            (areEqualRel(comp_point[2], point[2], FLT_EPSILON))
        ) {
            return i;
        }
    }
}

void Safe_Insert_Triangle(
                            Point3D< float > p1,
                            Point3D< float > p2,
                            Point3D< float > p3,
                            Point3D< float > p4,
                            Point3D< float > p5,
                            Point3D< float > p6,
                            std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                            std::map < int64_t, Point3D< float > > &index_site_map,
                            std::vector< TriangleIndex > &triangles
                        ) {
    TriangleIndex t1, t2, t3, t4;
    t1[0] = site_index_map[p1];
    t1[1] = site_index_map[p4];
    t1[2] = site_index_map[p6];

    t2[0] = site_index_map[p2];
    t2[1] = site_index_map[p5];
    t2[2] = site_index_map[p4];

    t3[0] = site_index_map[p3];
    t3[1] = site_index_map[p6];
    t3[2] = site_index_map[p5];

    t4[0] = site_index_map[p4];
    t4[1] = site_index_map[p5];
    t4[2] = site_index_map[p6];

    triangles.push_back(t1);
    triangles.push_back(t2);
    triangles.push_back(t3);
    triangles.push_back(t4);
}

void Determine_New_Centers(
                std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map,
                std::vector< PlyVertex< float > > &plyVertices,
                std::vector< TriangleIndex > &triangles
                ) {
    std::vector< TriangleIndex > new_triangles; // Will hold the new triangles
    // Write lock to ensure data writes are single threaded
	omp_lock_t writelock;
	omp_init_lock(&writelock);
    
    std::cout << "Current Vertices Size: " << site_index_map.size() << std::endl;
    std::cout << "Current Triangles Count: " << triangles.size() << std::endl;
    
    #pragma omp parallel num_threads(NUM_THREADS)
	{
        int64_t i = omp_get_thread_num();
        while (i < triangles.size()) {
            TriangleIndex t = triangles[i];
            Point3D< float > v0,v1,v2,m01,m12,m20;
            v0 = index_site_map[t[0]];
            v1 = index_site_map[t[1]];
            v2 = index_site_map[t[2]];
            m01 = Determine_Midpoint(v0, v1);
            m12 = Determine_Midpoint(v1, v2);
            m20 = Determine_Midpoint(v2, v0);
            
            omp_set_lock(&writelock);
            Safe_Insert_Vertex(site_index_map, index_site_map, m01);
            Safe_Insert_Vertex(site_index_map, index_site_map, m12);
            Safe_Insert_Vertex(site_index_map, index_site_map, m20);
            Safe_Insert_Triangle(v0, v1, v2,
                                 m01, m12, m20,
                                 site_index_map,
                                 index_site_map,
                                 new_triangles);
            omp_unset_lock(&writelock);
            i = i + NUM_THREADS;
        }
    }
   
    plyVertices.clear();
    triangles.clear();
    for (int i = 0; i < index_site_map.size(); i++) {
        plyVertices.push_back(index_site_map[i]);
    }
    triangles.clear();
    triangles = new_triangles;
    
    std::cout << "Current Vertices Size: " << site_index_map.size() << std::endl;
    std::cout << "Current Triangles Count: " << triangles.size() << std::endl;
}

void Put_In_Map(
                std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map,
                std::vector< PlyVertex< float > > &plyVertices,
                std::vector< TriangleIndex > &triangles
                ) {
    // Write lock to ensure data writes are single threaded
	omp_lock_t writelock;
	omp_init_lock(&writelock);

    // I know parallelization achieves nothing here, but just having it anyway
	#pragma omp parallel num_threads(NUM_THREADS)
	{
		int i = omp_get_thread_num();
		while ( i < plyVertices.size()) {
			omp_set_lock(&writelock);
			site_index_map[plyVertices[i]] = i;
			index_site_map[i] = plyVertices[i];
            Point3D< float > p = index_site_map[i];
			omp_unset_lock(&writelock);
			i = i + NUM_THREADS;
		}
	}
}
