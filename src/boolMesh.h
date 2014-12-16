#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3

#include "Kernel.h"

#include "AABB_Tree.h"
#include "FVMesh.h"
#include "halfEdgeMesh.h"

// enable/disable debug output
#define BOOL_MESH_DEBUG 1

#if BOOL_MESH_DEBUG == 1
#   define BOOL_MESH_DEBUG_DO(x) do { x } while (0);
#   define BOOL_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#   define BOOL_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define BOOL_MESH_DEBUG_DO(x)
#   define BOOL_MESH_DEBUG_SHOW(x)
#   define BOOL_MESH_DEBUG_PRINTLN(x)
#endif

struct SimpleEdge {
    Point p0;
    Point p1;
    SimpleEdge(Point& p0_, Point& p1_) : p0(p0_), p1(p1_) {};
};

class BooleanFVMeshOps
{
public:
    static void subtract(FVMesh& keep, HE_Mesh& subtracted, FVMesh& result);
    //static SimpleEdge intersect()
protected:

};




#endif // BOOL_MESH_H
