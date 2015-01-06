#ifndef BOOL_MESH_H
#define BOOL_MESH_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

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


typedef double xType; //!< type of x in the line equation L = ab * x + d

enum IntersectionPointType { EXISTING, NEW };

class IntersectionPoint
{
public:
    virtual Point getLocation() = 0;
    Point p() { return getLocation(); };
    virtual IntersectionPointType getType() = 0;
    virtual IntersectionPoint* copy() = 0;
    virtual std::string toString() = 0;
    virtual ~IntersectionPoint() {
//        BOOL_MESH_DEBUG_PRINTLN(" base destructor called! for IntersectionPoint at " << long(this));
    };
};

class ExistingVertexIntersectionPoint : public IntersectionPoint
{
public:

    HE_VertexHandle vh;

    ExistingVertexIntersectionPoint(HE_VertexHandle vh_) : vh(vh_) {};
    Point getLocation() { return vh.p(); };
    IntersectionPointType getType() { return EXISTING; };
    virtual ~ExistingVertexIntersectionPoint() {
//        BOOL_MESH_DEBUG_PRINTLN(" ~ExistingVertexIntersectionPoint for " << long(this));
    };
    ExistingVertexIntersectionPoint* copy() { return new ExistingVertexIntersectionPoint(*this); }
    std::string toString() { return vh.vertex().p.toString(); };
};

class NewIntersectionPoint : public IntersectionPoint
{
public:
    Point location;
    HE_EdgeHandle edge;
    NewIntersectionPoint(Point loc, HE_EdgeHandle eh) : location(loc), edge(eh) {};
    Point getLocation() { return location; };
    IntersectionPointType getType() { return NEW; };
    virtual ~NewIntersectionPoint() {
//        BOOL_MESH_DEBUG_PRINTLN(" ~NewIntersectionPoint for "<< long(this));
    };
    NewIntersectionPoint* copy() { return new NewIntersectionPoint(*this); }
    std::string toString() { return location.toString(); };
};

/*!
The intersection between two triangles: generally a line segment.
Cannot represent an area intersection (when the triangles are coplanar)

In case the intersection consists of a single point either from or to is nullptr .
*/
class TriangleIntersection
{
public:
    IntersectionPoint* from;
    IntersectionPoint* to;

    bool isDirectionOfInnerPartOfTriangle1;
    bool isDirectionOfInnerPartOfTriangle2;

    TriangleIntersection(IntersectionPoint* from, IntersectionPoint* to, bool inMesh2, bool inMesh1)
    : from(from)
    , to(to)
    , isDirectionOfInnerPartOfTriangle1(inMesh2)
    , isDirectionOfInnerPartOfTriangle2(inMesh1) {};
};





class BooleanFVMeshOps
{
public:
    static void subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result);
    static TriangleIntersection* intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2);

    static void test();
    //static SimpleEdge intersect()
protected:
    struct IntersectingLine //!< edge of one triangle intersecting the plane of the other triangle
    {
        FPoint3* from = nullptr;
        FPoint3* to = nullptr;
//        HE_Vertex* intersection = nullptr; //!< intersection point of an edge with the plane of another triangle (a new point on the edge, or an existing vertex)
//        bool intersectionIsNewVertex = false;
        IntersectionPoint* intersection;

        IntersectingLine(const IntersectingLine& old) {
            *this = old;
        }
        IntersectingLine& operator=(const IntersectingLine& old) {
            from = new FPoint3(*old.from);
            to = new FPoint3(*old.to);
            intersection = old.intersection->copy();
        };

        ~IntersectingLine() {
            delete from;
            delete to;
            delete intersection;
        };
        IntersectingLine() : intersection(nullptr) {};
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

        IntersectingLine line1; //!< one edge of the triangle crossing the intersection line between the two triangles planes
        IntersectingLine line2; //!< one edge of the triangle crossing the intersection line between the two triangles planes

        bool isDirectionOfInnerFacePart; //!< whether the direction of the line segment from the intersection of line1 to the intersection of line2 is in the direction of the halfEdge belonging to part of the face which is inside the other mesh

        bool isCorrect; //!< whether the triangles are in a position such that they intersect

        FPoint* O = nullptr; //!< any point on the line of the intersection between the two planes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

        IntersectionEnv() { };

        IntersectionEnv(const IntersectionEnv& old) {
        if (old.O != nullptr)
            O = new FPoint(*old.O);
        };
        ~IntersectionEnv() { if (O != nullptr) delete O; };

        void computeIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    };

    static IntersectionEnv getIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    static xType divide(FPoint a, FPoint& b);
};




#endif // BOOL_MESH_H
