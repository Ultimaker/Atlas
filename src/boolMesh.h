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


#include "MACROS.h" // debug
// enable/disable debug output
#define BOOL_MESH_DEBUG 1

#if BOOL_MESH_DEBUG == 1
#   define BOOL_MESH_DEBUG_DO(x) DEBUG_DO(x)
#   define BOOL_MESH_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define BOOL_MESH_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
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

    void debugOutput()
    {
        std::cerr << "starting node: " << std::endl;
        if (start != nullptr) start->from->data.debugOutput();
        std::cerr << "fracture nodes: " << std::endl;
        for (Node* node : fracture.nodes)
        {
            node->data.debugOutput();
        }
        std::cerr << "fracture node locations: " << std::endl;
        for (Node* node : fracture.nodes)
        {
            BOOL_MESH_DEBUG_PRINTLN(node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z);
        }
        std::cerr << "endPoints: " << std::endl;
        for (Arrow* a : endPoints)
            a->to->data.debugOutput();
        std::cerr << "connections: " << std::endl;
        for (Arrow* a : fracture.arrows)
            std::cerr << " "<< a->from->data.getLocation() << "=" << a->data.lineSegment.from->getLocation() << " -> " << a->data.lineSegment.to->getLocation() << "=" << a->to->data.getLocation() << std::endl;

    }
};

class BooleanMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result); //!< subtract one volume from another ([subtracted] from [keep])

    static void test_getFacetIntersectionlineSegment(PrintObject* model);
protected:


    HE_Mesh& keep, subtracted;
    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh1;
    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh2;
    static void getFacetIntersectionlineSegment(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLinePart& result); //!< adds all intersections connected to the first which intersect with the triangle to the mapping of the triangle
    static void addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*>& todo);

    void completeFractureLine(std::shared_ptr<TriangleIntersection> first); //!< walks along (each) fracture line segment recording all fracture line segemnts in the maps, until whole fracture is explored (a fracture line can split)

};




#endif // BOOL_MESH_H
