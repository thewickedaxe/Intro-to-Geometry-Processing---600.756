#include "Util/Ply.h"
#include "Util/Geometry.h"
#include "Util/CmdLineParser.h"
#include "Util/Timer.h"
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <map>

#define TRIANGLE_VERTEX_COUNT 3

void IdentifyVertex(const Point3D<float> point, const int index = -1) {
    if (index != -1) {
        printf("Vertex %d: [%f, %f, %f]\n", index, point[0], point[1], point[2]);
    } else {
        printf("Vertex: [%f, %f, %f]\n", point[0], point[1], point[2]);
    }
}

/*void IdentifyTriangle(TriangleIndex triangle, ) {

}*/