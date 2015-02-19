#ifndef TRIANGLE_INTERSECT_H
#define TRIANGLE_INTERSECT_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include <boost/optional.hpp> // ==maybe

#include <string>       // std::string
#include <sstream>      // std::stringstream,

#include <memory> // unique_ptr

#include <cstdlib> // exit (debug only)

#include "MACROS.h" // ENUM

#include "Kernel.h"

#include "AABB_Tree.h"
#include "mesh/Mesh.h"
#include "mesh/FVMesh.h"
#include "mesh/HalfEdgeMesh.h"

#include <boost/optional.hpp> // ==maybe


#include "MACROS.h" // debug
// enable/disable debug output
#define TRIANGLE_INTERSECT_DEBUG 0

#if TRIANGLE_INTERSECT_DEBUG == 1
#   define TRIANGLE_INTERSECT_DEBUG_DO(x) DEBUG_DO(x)
#   define TRIANGLE_INTERSECT_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define TRIANGLE_INTERSECT_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#else
#   define TRIANGLE_INTERSECT_DEBUG_DO(x)
#   define TRIANGLE_INTERSECT_DEBUG_SHOW(x)
#   define TRIANGLE_INTERSECT_DEBUG_PRINTLN(x)
#endif

typedef FPoint3 FPoint;

/*
TODO:
make intersection relative to point a
so that float precision is maximal
*/

typedef double xType; //!< type of x in the line equation L = ab * x + d

//! the type of an endpoint of a tri-tri intersection line segment: either an existing vertex or a new point to be made into a new vertex
ENUM(IntersectionPointType , VERTEX, NEW );


/*!
The interface for the two types of endpoint of a tri-tri intersection line segment (ExistingVertexIntersectionPoint and NewIntersectionPoint)
*/
class IntersectionPoint
{
public:
    Point location;
    HE_EdgeHandle edge; //!< the handle of the edge which gave rise to the location, when intersecting the edge with the halfplane of the other triangle

    HE_VertexHandle vertex; //!< the handle of the vertex coincident with the endpoint of the intersection line segment

    IntersectionPointType type;

    bool operator==(const IntersectionPoint& other) const
    {
        switch (type) {
        case IntersectionPointType::NEW:
            if (other.type != IntersectionPointType::NEW) return false;
            return edge == other.edge && location==other.location;
        case IntersectionPointType::VERTEX:
            if (other.type != IntersectionPointType::VERTEX) return false;
            return vertex == other.vertex;
        }
        return false;
    }

    Point getLocation_const() const//!< the location of the point
    {
        switch (type) {
        case IntersectionPointType::NEW: return location;
        case IntersectionPointType::VERTEX: return vertex.p_const();
        }
        return location;
    };
    Point& getLocation() //!< the location of the point
    {
        switch (type) {
        case IntersectionPointType::NEW: return location;
        case IntersectionPointType::VERTEX: return vertex.p();
        }
        return location;
    };
    Point& p() { return getLocation(); }; //!< the location of the point
    Point p_const() const { return getLocation_const(); }; //!< the location of the point
    IntersectionPointType getType() { return type; }; //!< the type of endpoint: existing vertex or new point

    IntersectionPoint(HE_VertexHandle vertex)               : type(IntersectionPointType::VERTEX), vertex(vertex),    edge(*vertex.m, -1) {};
    IntersectionPoint(Point loc, HE_EdgeHandle edge)    : type(IntersectionPointType::NEW), location(loc), edge(edge),     vertex(*edge.m, -1) {};
    IntersectionPoint(Point loc, HE_EdgeHandle edge, HE_VertexHandle vertex, IntersectionPointType type) : location(loc), edge(edge), vertex(vertex), type(type) {};

/*
    IntersectionPoint& operator=(const IntersectionPoint& other)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" IntersectionPoint location reassignment: " << location << " becomes" << other.location);
        location = other.location;
        edge = other.edge;
        vertex = other.vertex;
        return *this;
    }
*/
    IntersectionPoint* clone() { return new IntersectionPoint(location, edge, vertex, type); };

    void debugOutput()
    {
        std::cerr << getLocation() << "\t " << (type) << ",\t idx = " << ((type == IntersectionPointType::NEW)? edge.idx : vertex.idx)<< "\t face :" << ((type == IntersectionPointType::NEW)? edge.face().idx : -1) << std::endl;
    };

    HE_Mesh* getSourceMesh()
    {
        switch (type) {
        case IntersectionPointType::NEW: return edge.m;
        case IntersectionPointType::VERTEX: return vertex.m;
        }
        return edge.m;
    };
    bool compareSource(const IntersectionPoint& other)
    {
        if (type != other.type) return false;
        switch (type)
        {
        case IntersectionPointType::NEW:
            return edge == other.edge;
        case IntersectionPointType::VERTEX:
            return vertex == other.vertex;
        }
    }
    bool compareSourceConverse(const IntersectionPoint& other)
    {
        if (type != other.type) return false;
        switch (type)
        {
        case IntersectionPointType::NEW:
            return edge.converse() == other.edge;
        case IntersectionPointType::VERTEX:
            return vertex == other.vertex;
        }
    }
    bool belongsToSource(const HE_FaceHandle fh)
    {
        switch(type) {
        case IntersectionPointType::NEW:
            return edge.face() == fh || edge.converse().face() == fh;
        case IntersectionPointType::VERTEX:
            return fh.hasVertex(vertex);
        }
    }
};


//! type of (non-)intersection between triangles
ENUM(IntersectionType , LINE_SEGMENT, COPLANAR, TOUCHING_POINT, NON_TOUCHING, NON_TOUCHING_PLANES, PARALLEL, UNKNOWN );


/*!
The intersection between two triangles: a line segment.
Cannot represent an area intersection (when the triangles are coplanar)
or a point intersection (when triangles only touch).

Information on the origin of the intersection is also represented.
We keep track of which elements of the original meshes gave rise to the endpoints of the line segment
and we keep information on what way the intersection divides the triangle in parts;
the line segment knows which part of the first triangle falls below the second triangle and vice versa.
*/
struct TriangleIntersection
{
    boost::optional<IntersectionPoint> from; //!< the first endpoint of the intersection line segment
    boost::optional<IntersectionPoint> to;//!< the second endpoint of the intersection line segment

    bool isDirectionOfInnerPartOfTriangle1; //!< whether the direction  from -> to  is the same direction as the halfedge belonging the part of the first triangle which is below the second triangle
    bool isDirectionOfInnerPartOfTriangle2; //!< whether the direction  from -> to  is the same direction as the halfedge belonging the part of the second triangle which is below the first triangle

    boost::optional<HE_EdgeHandle> edgeOfTriangle1TouchingTriangle2; //!< either no edge or the edge of triangle1 with which the line segment is coincident (and so the triangle1 touches the plane of the other with an edge)
    boost::optional<HE_EdgeHandle> edgeOfTriangle2TouchingTriangle1; //!< either no edge or the edge of triangle2 with which the line segment is coincident (and so the triangle2 touches the plane of the other with an edge)

    IntersectionType intersectionType;

    TriangleIntersection(boost::optional<IntersectionPoint> from_, boost::optional<IntersectionPoint> to_
        , bool inMesh2, bool inMesh1
        , boost::optional<HE_EdgeHandle>& edgeOfTriangle1TouchingTriangle2, boost::optional<HE_EdgeHandle>& edgeOfTriangle2TouchingTriangle1
        , IntersectionType intersectionType_)
    : from(from_)
    , to(to_)
    , isDirectionOfInnerPartOfTriangle1(inMesh2)
    , isDirectionOfInnerPartOfTriangle2(inMesh1)
    , intersectionType(intersectionType_)
    , edgeOfTriangle1TouchingTriangle2(edgeOfTriangle1TouchingTriangle2)
    , edgeOfTriangle2TouchingTriangle1(edgeOfTriangle2TouchingTriangle1)
    { };
    TriangleIntersection(boost::optional<IntersectionPoint> from_, boost::optional<IntersectionPoint> to_, bool inMesh2, bool inMesh1, IntersectionType intersectionType_)
    : from(from_)
    , to(to_)
    , isDirectionOfInnerPartOfTriangle1(inMesh2)
    , isDirectionOfInnerPartOfTriangle2(inMesh1)
    , intersectionType(intersectionType_)
    , edgeOfTriangle1TouchingTriangle2(boost::none)
    , edgeOfTriangle2TouchingTriangle1(boost::none)
    { };

    bool operator==(const TriangleIntersection& other) const
    {
        return intersectionType == other.intersectionType && from == other.from && to == other.to;
    }

    void reverse()
    {
        std::swap(from, to);
        isDirectionOfInnerPartOfTriangle1 = ! isDirectionOfInnerPartOfTriangle1;
        isDirectionOfInnerPartOfTriangle2 = ! isDirectionOfInnerPartOfTriangle2;
    };
};





/*!
Class for computing the intersection between two triangles.


*/
class TriangleIntersectionComputation
{
public:
    static std::shared_ptr<TriangleIntersection> intersect(HE_FaceHandle fh1, HE_FaceHandle fh2);
    static std::shared_ptr<TriangleIntersection> intersect(HE_FaceHandle fh1, HE_FaceHandle fh2, boost::optional<Point> some_point_on_planes_intersection_line);

    static void test();
protected:
    /*!
    A line segment corresponding to an edge of the one triangle crossing the halfplane of the other triangle.
    The intersection point at which the line intersects the halfplane is also stored.
    */
    struct LinePlaneIntersection //!< edge of one triangle intersecting the halfplane of the other triangle
    {
        boost::optional<FPoint3> from;
        boost::optional<FPoint3> to;

        boost::optional<IntersectionPoint> intersection;

        LinePlaneIntersection()
        : from(boost::none)
        , to(boost::none)
        , intersection(boost::none) {};
    };

    /*!
    Intersecting a triangle with a halfplane (in which another triangle lies).

    Stores the intersections of the line segments corresponding to the (generally two) edges of the triangle crossing the halfplane:
    stores the line segments and with it the intersections of these line segments with the halfplane.

    Keeps track of which side of the triangle falls below the halfplane.

    */
    class TrianglePlaneIntersection
    {
        friend class TriangleIntersectionComputation; // make computeIntersectingEdges visible
    public:
        LinePlaneIntersection line1; //!< one edge of the triangle crossing the intersection line between the two triangles planes
        LinePlaneIntersection line2; //!< the other edge of the triangle crossing the plane of the second triangle [equivalent description]

        bool isDirectionOfInnerFacePart; //!< whether the direction of the line segment from the intersection of line1 to the intersection of line2 is in the direction of the halfEdge belonging to part of the face which is below the halfplane

        boost::optional<HE_EdgeHandle> edgeOfTriangleTouchesPlane; //!< whether and which edge of the triangle is touching the plane

        bool isCorrect; //!< whether the triangles are in a position such that they intersect

        IntersectionType intersectionType; //!< information on the intersection type between the two triangles, if already known.

        boost::optional<FPoint3> O; //!< any point on the line of the intersection between the two halfplanes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

        TrianglePlaneIntersection()
        : isCorrect(false)
        , intersectionType(IntersectionType::UNKNOWN)
        , O(boost::none)
        , edgeOfTriangleTouchesPlane(boost::none)
        {  };


    protected:
        /*!
        Set the [from] and [to] of [line1] and [line2], and if at hand, set the information of O and the actual intersection points.
        When a vertex intersects the triangle, don't set the [from] and [to], but set the vertex as intersections point itself instead.
        In such a case we can use the vertex as [O].

        The parameters are the vertex points, the orientation of these points wrt the halfplane and the face handle of the original triangle.
         */
        void computeIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    };
    /*!
    Returns a new TrianglePlaneIntersection on which computeIntersectingEdges(.) is performed.

    The parameters are the vertex points, the orientation of these points wrt the halfplane and the face handle of the original triangle.
    */
    static TrianglePlaneIntersection getIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh);
    static xType divide(FPoint a, FPoint& b); //!< divide two vectors, assuming they are in the same direction
};




#endif // TRIANGLE_INTERSECT_H
