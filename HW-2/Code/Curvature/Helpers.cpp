#include "Util/Ply.h"
#include "Util/Geometry.h"
#include "Util/CmdLineParser.h"
#include "Util/Timer.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <unordered_set>

#define TRIANGLE_VERTEX_COUNT 3

double max(double a, double b) {
	if (a > b) {
		return a;
	} else {
		return b;
	}
}

double min(double a, double b) {
    if (a < b) {
        return a;
    } else {
        return b;
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

class SmartFace
{
public:
	Point3D< float> v1, n1;
    Point3D< float> v2, n2;
    Point3D< float> v3, n3;
    Point3D < float > normal;
    double curvature;
    double mean_curvature;
    double gaussian_curvature;
    double area;
    double pc_1, pc_2;
    SquareMatrix< float , 2 > shape;
    Point3D< float > dir1;
    Point3D< float > dir2;
    int64_t index;

	void Calc_Area(void) {
		double t1[3],t2[3],t[3];
		for(int d=0;d<3;d++){
			t1[d]=v2[d]-v1[d];
			t2[d]=v3[d]-v1[d];
		}
		t[0]= t1[1]*t2[2]-t1[2]*t2[1];
		t[1]=-t1[0]*t2[2]+t1[2]*t2[0];
		t[2]= t1[0]*t2[1]-t1[1]*t2[0];
        area = sqrt(t[0]*t[0]+t[1]*t[1]+t[2]*t[2]) / 2; // Multiplied by 100000 to handle precision problems later
        //std::cout<<"Area: " << area << std::endl;
	}
    void Calc_Normal() {
        Point3D< float > vec1 = v2 - v1;
        Point3D< float > vec2 = v3 - v1;
        normal = CrossProduct(vec1, vec2);
        //IdentifyVertex(normal);
    }
    void Calc_Curvature(int curvature_type) {
        SquareMatrix< float , 2 > first_form;
        SquareMatrix< float , 2 > second_form_temp;
        SquareMatrix< float , 2 > second_form;
        Point3D< float > v1_v0, v2_v0, n1_n0, n2_n0;
        v1_v0 = v2 - v1;
        v2_v0 = v3 - v1;
        n1_n0 = n2 - n1;
        n2_n0 = n3 - n1;
        first_form(0, 0) = DotProduct(v1_v0, v1_v0);
        first_form(0, 1) = DotProduct(v2_v0, v1_v0);
        first_form(1, 0) = DotProduct(v1_v0, v2_v0);
        first_form(1, 1) = DotProduct(v2_v0, v2_v0);

        second_form_temp(0, 0) = DotProduct(n1_n0, v1_v0);
        second_form_temp(0, 1) = DotProduct(n2_n0, v1_v0);
        second_form_temp(1, 0) = DotProduct(n1_n0, v2_v0);
        second_form_temp(1, 1) = DotProduct(n2_n0, v2_v0);

        second_form = (second_form_temp + second_form_temp.transpose()) / 2;

        shape = (first_form.inverse()) * second_form;
        
        if (curvature_type == 0) { // Mean
            curvature = shape.trace() / 2;
            mean_curvature = curvature;
        } else { // Gaussian
            curvature = shape.determinant();
            gaussian_curvature = curvature;
        }
    }
    double Calc_Principal_Curvatures() {
        pc_1 = mean_curvature + sqrt(pow(mean_curvature, 2) - gaussian_curvature);
		pc_2 = mean_curvature - sqrt(pow(mean_curvature, 2) - gaussian_curvature);
    }
    double Calc_Principal_Directions(double k1, double k2) {
        SquareMatrix< float , 2 > curv_mat;
        curv_mat(0,0) = k1;
        curv_mat(1,1) = k2;
        curv_mat(0,1) = 0;
        curv_mat(1,0) = 0;

        SquareMatrix< float , 2 > dir_pre = shape * (curv_mat.inverse());
        dir1 = ((v2 - v1) * dir_pre(0, 0)) + ((v3 - v1) * dir_pre(1, 0));
        dir2 = ((v2 - v1) * dir_pre(0, 1)) + ((v3 - v1) * dir_pre(1, 1));
    }
    Point3D< float > &operator[]( unsigned int idx ) {
        if (idx == 0) {
            return v1;
        } else if (idx == 1) {
            return v2;
        } else if (idx == 2) {
            return v3;
        }
    }
	Point3D< float >  operator[]( unsigned int idx ) const {
        if (idx == 0) {
            return v1;
        } else if (idx == 1) {
            return v2;
        } else if (idx == 2) {
            return v3;
        }
    }
};

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

void Put_In_Map(
                std::map < Point3D< float > , int64_t , PointCompare> &site_index_map,
                std::map < int64_t, Point3D< float > > &index_site_map,
                std::vector< PlyOrientedVertex< float > > plyVertices,
                std::vector< TriangleIndex > triangles
                ) {
	for (int64_t i = 0; i < plyVertices.size(); i++) {
			site_index_map[plyVertices[i]] = i;
			index_site_map[i] = plyVertices[i];
	}
}
