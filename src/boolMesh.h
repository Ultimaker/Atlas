#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3

#include "Kernel.h"

#include "AABB_Tree.h"
#include "mesh/Mesh.h"
#include "mesh/FVMesh.h"
#include "mesh/HalfEdgeMesh.h"

// enable/disable debug output
#define BOOL_MESH_DEBUG 1

#if BOOL_MESH_DEBUG == 1
#   define BOOL_MESH_DEBUG_DO(x) do { x } while (0);
#   define BOOL_MESH_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#   define BOOL_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define BOOL_MESH_DEBUG_DO(x)
#   define BOOL_MESH_DEBUG_SHOW(x)
#   define BOOL_MESH_DEBUG_PRINTLN(x)
#endif


struct IntersectionEdge {
    Point p0;
    Point p1;
    HE_EdgeHandle edge_from;
    HE_EdgeHandle edge_to;
    IntersectionEdge(Point& p0_, Point& p1_, HE_EdgeHandle& edge_from, HE_EdgeHandle& edge_to) : p0(p0_), p1(p1_), edge_from(edge_from), edge_to(edge_to) {};
};

typedef double xType; //!< type of x in the line equation L = ab * x + d

class BooleanFVMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result);
    static void intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2);

    static void test();
    //static SimpleEdge intersect()
protected:

    struct IntersectingLine
    {
        Point3* from = nullptr;
        Point3* to = nullptr;
        HE_Vertex* intersection = nullptr;
    };
    struct IntersectionEnv //!< warning! uses pointers to the objects to which the parameters are references!
    {
//        Point3* line1from = nullptr; // the two intersecting lines from triangle 2
//        Point3* line1to = nullptr;
//        Point3* line2from = nullptr;
//        Point3* line2to = nullptr;
//
//        HE_Vertex* intersection1 = nullptr; // reference to existing vertex OR
//        HE_Vertex* intersection2 = nullptr; // ... to new placeholder vertex not yet inserted into the mesh (still a nullpointer after computeIntersectingEdges)

        IntersectingLine line1;
        IntersectingLine line2;

        Point* O = nullptr; // any point on the line of the intersection between the two planes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

        void computeIntersectingEdges(Point3& a, Point3& b, Point3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    };

    static IntersectionEnv getIntersectingEdges(Point3& a, Point3& b, Point3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    static xType divide(Point a, Point& b);
};




#endif // BOOL_MESH_H
