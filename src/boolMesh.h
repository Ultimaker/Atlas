#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include "triangleIntersect.h"


#include <list> // == double-linked list, used as queue

#include <unordered_map> // == hash_map

#include <string>       // std::string
#include <sstream>      // std::stringstream,

#include <cstdlib> // exit (debug only)

#include "Kernel.h"

#include "AABB_Tree.h"
#include "mesh/Mesh.h"
#include "mesh/FVMesh.h"
#include "mesh/HalfEdgeMesh.h"

#include "utils/graph.h"

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

struct IntersectionSegment
{
    TriangleIntersection lineSegment; //! the intersection as computed by triangleIntersect.cpp
    HE_FaceHandle otherFace; //! the other face than the one for which this IntersectionSegment is used
    IntersectionSegment(TriangleIntersection lineSegment_, HE_FaceHandle otherFace_) : lineSegment(lineSegment_), otherFace(otherFace_) {};
};

//! intersection line a.k.a. fracture line is a polyline
typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;
typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;

struct FractureLinePart
{
    // when two triangles are coplanar, the intersection is non-linear ( => graph-like)
    // also where a vertex lies exactly on a triangle of the other mesh, the intersection can be non-linear ( => graph-like)
    Graph<IntersectionPoint, IntersectionSegment> fracture;
    Graph<IntersectionPoint, IntersectionSegment>::Arrow* start;
    std::vector<Graph<IntersectionPoint, IntersectionSegment>::Arrow*> endPoints;
};

class BooleanFVMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result); //!< subtract one volume from another ([subtracted] from [keep])

protected:


    HE_Mesh& keep, subtracted;
    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh1;
    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh2;
    void getFacetIntersectionlineSegment(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLinePart& result); //!< adds all intersections connected to the first which intersect with the triangle to the mapping of the triangle
    void completeFractureLine(std::shared_ptr<TriangleIntersection> first); //!< walks along (each) fracture line segment recording all fracture line segemnts in the maps, until whole fracture is explored (a fracture line can split)

    void addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*> todo);
};




#endif // BOOL_MESH_H
