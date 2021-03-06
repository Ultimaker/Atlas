#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include "triangleIntersect.h"


#include <list> // == double-linked list, used as queue

#include <unordered_map> // == hash_map
#include <unordered_set> // == hash_set

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
    TriangleIntersection lineSegment; //!< the intersection as computed by triangleIntersect.cpp
    HE_FaceHandle otherFace; //!< the other face than the one for which this IntersectionSegment is used
    bool otherFace_is_second_triangle; //!< whether [otherFace] is triangle2 of the intersection
    IntersectionSegment(TriangleIntersection lineSegment_, HE_FaceHandle otherFace_, bool otherFace_is_second_triangle)
    : lineSegment(lineSegment_)
    , otherFace(otherFace_)
    , otherFace_is_second_triangle(otherFace_is_second_triangle)
    { };
    bool isDirectionOfInnerPartOfOtherTriangle()
    {
        if (otherFace_is_second_triangle)
            return lineSegment.isDirectionOfInnerPartOfTriangle2;
        else
            return lineSegment.isDirectionOfInnerPartOfTriangle1;
    }
    bool isDirectionOfInnerPartOfMainTriangle()
    {
        if (otherFace_is_second_triangle)
            return lineSegment.isDirectionOfInnerPartOfTriangle1;
        else
            return lineSegment.isDirectionOfInnerPartOfTriangle2;
    }
    boost::optional<HE_EdgeHandle> edgeOfMainTouchingOtherTriangle()
    {
        if (otherFace_is_second_triangle)
            return lineSegment.edgeOfTriangle1TouchingTriangle2;
        else
            return lineSegment.edgeOfTriangle2TouchingTriangle1;
    }
    boost::optional<HE_EdgeHandle> edgeOfOtherTouchingMainTriangle()
    {
        if (otherFace_is_second_triangle)
            return lineSegment.edgeOfTriangle2TouchingTriangle1;
        else
            return lineSegment.edgeOfTriangle1TouchingTriangle2;
    }
};

//! intersection line a.k.a. fracture line is a polyline
typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;
typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;

struct FractureLinePart
{
    // when two triangles are coplanar, the intersection is non-linear ( => graph-like)
    // also where a vertex lies exactly on a triangle of the other mesh, the intersection can be non-linear ( => graph-like)
    HE_FaceHandle face; //!< the face on which this fracture line part lies
    Graph<IntersectionPoint, IntersectionSegment> fracture;
    Arrow* start;
    std::vector<Arrow*> endPoints; //!< does this include start?!

    FractureLinePart(HE_FaceHandle face) : face(face) {};

//    void combine(FractureLinePart& b)
//    {
//        if (face != b.face)
//            ERROR("fracture line parts cannot be combined!");
//        for (Node* node : b.fracture.nodes)
//            if (node->data.
//            fracture.
//    };

    void endPointsFaces(std::unordered_set<HE_FaceHandle>& result)
    {
        for (Arrow* a : endPoints)
            result.emplace(a->data.otherFace);
        if (start==nullptr)
            BOOL_MESH_DEBUG_PRINTLN("start of fracture is NULL!");
        result.emplace(start->data.otherFace);
    }
    void endPointsFacesWithoutFirst(std::unordered_set<HE_FaceHandle>& result)
    {
        for (Arrow* a : endPoints)
            result.emplace(a->data.otherFace);
    }


    void debugOutput()
    {
        std::cerr << "starting node: " << std::endl;
        if (start != nullptr) start->from->data.debugOutput();
        std::cerr << "fracture nodes: " << std::endl;
        for (Node* node : fracture.nodes)
        {
            node->data.debugOutput();
        }
        std::cerr << "endPoints: " << std::endl;
        for (Arrow* a : endPoints)
            a->to->data.debugOutput();
        std::cerr << "connections: " << std::endl;
        for (Arrow* a : fracture.arrows)
            std::cerr << " "<< a->from->data.getLocation() << "=" << a->data.lineSegment.from->getLocation() << " -> " << a->data.lineSegment.to->getLocation() << "=" << a->to->data.getLocation() << std::endl;

    }
    void debugOutputNodePoints()
    {
        std::cerr << "fracture node locations: " << std::endl;
        for (Node* node : fracture.nodes)
        {
            BOOL_MESH_DEBUG_PRINTLN(node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z);
        }
    }
};

ENUM(BoolOpType, UNION, INTERSECTION, DIFFERENCE);

class BooleanMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result); //!< subtract one volume from another ([subtracted] from [keep])

protected:

    HE_Mesh& keep, subtracted;
    BoolOpType boolOpType;

    bool useCoplanarFaceIntersection(HE_FaceHandle fh1, HE_FaceHandle fh2)
    {
        bool sameNormals = fh1.normal().dot(fh2.normal()) > 0;
        switch(boolOpType)
        {
        case BoolOpType::UNION:         return sameNormals;
        case BoolOpType::INTERSECTION:  return sameNormals;
        case BoolOpType::DIFFERENCE:    return !sameNormals;
        }
    };
    bool useAboveFracture(HE_FaceHandle fh)
    {
        switch(boolOpType)
        {
        case BoolOpType::UNION:         return true;
        case BoolOpType::INTERSECTION:  return false;
        case BoolOpType::DIFFERENCE:    return fh.m == &keep;
        }
    };


    BooleanMeshOps(HE_Mesh& keep, HE_Mesh& subtracted, BoolOpType boolOpType)
    : keep(keep), subtracted(subtracted), boolOpType(boolOpType)
    { } ;


//    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh1;
//    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fracturelines_mesh2;
    std::shared_ptr<TriangleIntersection> getIntersection(HE_FaceHandle f1, HE_FaceHandle f2);

    void perform(HE_Mesh& result); //!< subtract one volume from another ([subtracted] from [keep])

    void getFacetFractureLinePart(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, TriangleIntersection& first, FractureLinePart& result); //!< adds all intersections connected to the first which intersect with the triangle to the mapping of the triangle
    void addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*>& todo);

    void completeFractureLine(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, TriangleIntersection& first, std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>>& face2fracture); //!< walks along (each) fracture line part recording all fracture line segemnts in the maps, until whole fracture is explored (a fracture line can split)

    void getNextFacesOnFracture(HE_FaceHandle triangle1, Arrow* current, std::unordered_set<HE_FaceHandle>& checked_faces, std::vector<std::tuple<HE_FaceHandle, TriangleIntersection>>& result);



public:
    static void test_subtract();
    static void test_subtract(PrintObject* model);
    static void test_subtract(HE_Mesh& keep, HE_Mesh& subtracted);
    static void test_getFacetFractureLinePart(PrintObject* model);
    static void test_completeFractureLine();
    static void test_completeFractureLine(PrintObject* model);
    static void test_completeFractureLine(HE_Mesh& keep, HE_Mesh& subtracted);

private:
    void debug_export_problem(HE_FaceHandle triangle1, std::shared_ptr<TriangleIntersection> triangleIntersection, HE_FaceHandle newFace, IntersectionPoint& connectingPoint);
    void debug_export_difference_mesh(HE_FaceHandle originalFace, IntersectionPoint& connectingPoint, TriangleIntersection& triangleIntersection, HE_FaceHandle newFace);

    void debug_csv(std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> & face2fractures, std::string filename = "WHOLE.csv");



};




#endif // BOOL_MESH_H
