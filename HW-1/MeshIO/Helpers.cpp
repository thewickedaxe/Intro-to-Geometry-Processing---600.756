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
#include <unordered_set>


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

bool AreEqual(Point3D< float > a, Point3D< float> b) {
    if((a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2])) {
        return true;
    } else {
        return false;
    }
}


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

Point3D< float > blend (Point3D <float> point, std::vector< Point3D< float > > neighbors, float blending_weight) {
    float sum_x, sum_y, sum_z;
    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    for (int i = 0; i < neighbors.size(); i++) {
        sum_x += neighbors[i][0];
        sum_y += neighbors[i][1];
        sum_z += neighbors[i][2];
    }

    sum_x = sum_x / neighbors.size();
    sum_y = sum_y / neighbors.size();
    sum_z = sum_z / neighbors.size();

    sum_x *= blending_weight;
    sum_y *= blending_weight;
    sum_z *= blending_weight;

    Point3D< float > ans;
    ans[0] = sum_x + (1 - blending_weight) * point[0];
    ans[1] = sum_y + (1 - blending_weight) * point[1];
    ans[2] = sum_z + (1 - blending_weight) * point[2];

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
                            std::vector< TriangleIndex > &triangles,
                            std::map < int64_t, std::unordered_set< int64_t > > &associated_vertices
                        ) {
    TriangleIndex t1, t2, t3, t4;
    t1[0] = site_index_map[p1];
    t1[1] = site_index_map[p4];
    t1[2] = site_index_map[p6];
    associated_vertices[t1[0]].insert(t1[1]); associated_vertices[t1[0]].insert(t1[2]);
    associated_vertices[t1[1]].insert(t1[0]); associated_vertices[t1[1]].insert(t1[2]);
    associated_vertices[t1[2]].insert(t1[0]); associated_vertices[t1[2]].insert(t1[1]);

    t2[0] = site_index_map[p2];
    t2[1] = site_index_map[p5];
    t2[2] = site_index_map[p4];
    associated_vertices[t2[0]].insert(t2[1]); associated_vertices[t2[0]].insert(t2[2]);
    associated_vertices[t2[1]].insert(t2[0]); associated_vertices[t2[1]].insert(t2[2]);
    associated_vertices[t2[2]].insert(t2[0]); associated_vertices[t2[2]].insert(t2[1]);

    t3[0] = site_index_map[p3];
    t3[1] = site_index_map[p6];
    t3[2] = site_index_map[p5];
    associated_vertices[t3[0]].insert(t3[1]); associated_vertices[t3[0]].insert(t3[2]);
    associated_vertices[t3[1]].insert(t3[0]); associated_vertices[t3[1]].insert(t3[2]);
    associated_vertices[t3[2]].insert(t3[0]); associated_vertices[t3[2]].insert(t3[1]);

    t4[0] = site_index_map[p4];
    t4[1] = site_index_map[p5];
    t4[2] = site_index_map[p6];
    associated_vertices[t4[0]].insert(t4[1]); associated_vertices[t4[0]].insert(t4[2]);
    associated_vertices[t4[1]].insert(t4[0]); associated_vertices[t4[1]].insert(t4[2]);
    associated_vertices[t4[2]].insert(t4[0]); associated_vertices[t4[2]].insert(t4[1]);

    triangles.push_back(t1);
    triangles.push_back(t2);
    triangles.push_back(t3);
    triangles.push_back(t4);
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

void Determine_New_Centers(
                std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map,
                std::vector< PlyVertex< float > > &plyVertices,
                std::vector< TriangleIndex > &triangles,
                std::map < int64_t, std::unordered_set< int64_t > > &associated_vertices
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
            Safe_Insert_Triangle(
                                 v0, v1, v2,
                                 m01, m12, m20,
                                 site_index_map,
                                 index_site_map,
                                 new_triangles,
                                 associated_vertices
                                 );
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
