#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include "triangleIntersect.h"

#include <unordered_map> // ==hash_map

#include <string>       // std::string
#include <sstream>      // std::stringstream,

#include <cstdlib> // exit (debug only)

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

typedef FPoint3 FPoint;

class FractureLineSegment
{
    //std::graph<IntersectionPoint> fracture;
    //Edge firstHalfEdgeOfOuterPart;

};

class BooleanFVMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result); //!< subtract one volume from another ([subtracted] from [keep])


protected:
    std::unordered_map<HE_FaceHandle, FractureLineSegment> face2fractureline_mesh1;
    std::unordered_map<HE_FaceHandle, FractureLineSegment> face2fractureline_mesh2;
    void followIntersectionsAlongFacet(HE_FaceHandle& triangle, TriangleIntersection first); //!< adds all intersections connected to the first which intersect with the triangle to the mapping of the triangle
    void completeFractureLine(TriangleIntersection first); //!< walks along (each) fracture line segment recording all fracture line segemnts in the maps, until whole fracture is explored (a fracture line can split)
};




#endif // BOOL_MESH_H
