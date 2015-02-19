#ifndef BOOL_MESH_OPS_H
#define BOOL_MESH_OPS_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include "triangleIntersect.h"

#include "utils/BucketGrid3D.h"

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
#define BOOL_MESH_OPS_DEBUG 1

#if BOOL_MESH_OPS_DEBUG == 1
#   define BOOL_MESH_OPS_DEBUG_DO(x) DEBUG_DO(x)
#   define BOOL_MESH_OPS_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define BOOL_MESH_OPS_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#else
#   define BOOL_MESH_OPS_DEBUG_DO(x)
#   define BOOL_MESH_OPS_DEBUG_SHOW(x)
#   define BOOL_MESH_OPS_DEBUG_PRINTLN(x)
#endif


namespace boolOps {

typedef FPoint3 FPoint;

//! intersection line a.k.a. fracture line is a polyline
typedef Graph<Point, TriangleIntersection>::Arrow Arrow;
typedef Graph<Point, TriangleIntersection>::Node Node;

struct FractureLinePart
{
    // when two triangles are coplanar, the intersection is non-linear ( => graph-like)
    // also where a vertex lies exactly on a triangle of the other mesh, the intersection can be non-linear ( => graph-like)
    HE_FaceHandle face; //!< the face on which this fracture line part lies
    Graph<Point, TriangleIntersection> fracture;
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

//    void endPointsFaces(std::unordered_set<HE_FaceHandle>& result)
//    {
//        for (Arrow* a : endPoints)
//            result.emplace(a->data.otherFace);
//    }

//
//    void debugOutput()
//    {
//        std::cerr << "fracture nodes: " << std::endl;
//        for (Node* node : fracture.nodes)
//        {
//            node->data.debugOutput();
//        }
//        std::cerr << "endPoints: " << std::endl;
//        for (Arrow* a : endPoints)
//            a->to->data.debugOutput();
//        std::cerr << "connections: " << std::endl;
//        for (Arrow* a : fracture.arrows)
//            std::cerr << " "<< a->from->data.getLocation() << "=" << a->data.lineSegment.from->getLocation() << " -> " << a->data.lineSegment.to->getLocation() << "=" << a->to->data.getLocation() << std::endl;
//
//    }
//    void debugOutputNodePoints()
//    {
//        std::cerr << "fracture node locations: " << std::endl;
//        for (Node* node : fracture.nodes)
//        {
//            BOOL_MESH_OPS_DEBUG_PRINTLN(node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z);
//        }
//    }
};


ENUM(BoolOpType, UNION, INTERSECTION, DIFFERENCE);



struct IntersectionPointHasher
{
    static inline uint32_t pointHash(const Point3& point)
    {
        Point p = point/MELD_DISTANCE;
        return p.x ^ (p.y << 10) ^ (p.z << 20);
    };
    uint32_t operator()(const std::pair<TriangleIntersection,bool>& p) const
    {
        if (p.second)
            return pointHash(p.first.from->p_const());
        else
            return pointHash(p.first.to->p_const());
    };
    uint32_t operator()(const IntersectionPoint& p) const
    {
        return pointHash(p.p_const());
    };
};



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

    typedef std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>> Face2Soup;

    void createIntersectionSegmentSoup(Face2Soup& fracture_soup_keep, Face2Soup& fracture_soup_subtracted, AABB_Tree<HE_FaceHandle>& keep_aabb, std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle>>& coplanarKeepToSubtracted);

    void hashMapInsert(std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>>& fractures, HE_FaceHandle tri1, HE_FaceHandle tri2, TriangleIntersection& triangleIntersection);

    void getFace2fractures(
        const std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>>& fracture_soup_keep,
        std::unordered_map<HE_FaceHandle, FractureLinePart>& face2fractures_keep,
        bool keep,
        const std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle>>& coplanarKeepToSubtracted);

    void connectNodesInFracture (
        std::unordered_map<HE_FaceHandle, TriangleIntersection>& segments,
        std::unordered_map<IntersectionPoint , Node*, IntersectionPointHasher>& point2fracNode,
        const std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle>>& coplanarKeepToSubtracted,
        FractureLinePart& frac,
        const HE_FaceHandle tri_main,
        const bool keep
        );

public:
    static void test_subtract();
    static void test_subtract(PrintObject* model);
    static void test_subtract(HE_Mesh& keep, HE_Mesh& subtracted);

private:
    void debug_export_problem(HE_FaceHandle triangle1, std::shared_ptr<TriangleIntersection> triangleIntersection, HE_FaceHandle newFace, IntersectionPoint& connectingPoint);
    void debug_export_difference_mesh(HE_FaceHandle originalFace, IntersectionPoint& connectingPoint, TriangleIntersection& triangleIntersection, HE_FaceHandle newFace);

    void debug_csv(std::unordered_map<HE_FaceHandle, FractureLinePart> & face2fractures, std::string filename = "WHOLE.csv");



};


} // namespace boolOps


#endif // BOOL_MESH_OPS_H
