#ifndef TRIANGLE_INTERSECT_H
#define TRIANGLE_INTERSECT_H

#include "utils/intpoint.h" // Point3
#include "utils/floatpoint.h" // Point3

#include <boost/optional.hpp> // ==maybe

#include <string>       // std::string
#include <sstream>      // std::stringstream,

#include <memory> // unique_ptr

#include <cstdlib> // exit (debug only)

#include "Kernel.h"

#include "AABB_Tree.h"
#include "mesh/Mesh.h"
#include "mesh/FVMesh.h"
#include "mesh/HalfEdgeMesh.h"

#include <boost/optional.hpp> // ==maybe

// enable/disable debug output
#define TRIANGLE_INTERSECT_DEBUG 1

#if TRIANGLE_INTERSECT_DEBUG == 1
#   define TRIANGLE_INTERSECT_DEBUG_DO(x) do { x } while (0);
#   define TRIANGLE_INTERSECT_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#   define TRIANGLE_INTERSECT_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define TRIANGLE_INTERSECT_DEBUG_DO(x)
#   define TRIANGLE_INTERSECT_DEBUG_SHOW(x)
#   define TRIANGLE_INTERSECT_DEBUG_PRINTLN(x)
#endif

typedef FPoint3 FPoint;


typedef double xType; //!< type of x in the line equation L = ab * x + d

enum IntersectionPointType { EXISTING, NEW }; //!< the type of an endpoint of a tri-tri intersection line segment: either an existing vertex or a new point to be made into a new vertex

/*!
The interface for the two types of endpoint of a tri-tri intersection line segment (ExistingVertexIntersectionPoint and NewIntersectionPoint)
*/
class IntersectionPoint
{
public:
    Point location;
    HE_EdgeHandle edge; //!< the handle of the edge which gave rise to the location, when intersecting the edge with the halfplane of the other triangle

    HE_VertexHandle vh; //!< the handle of the vertex coincident with the endpoint of the intersection line segment

    IntersectionPointType type;
    Point getLocation()
    {
        switch (type) {
        case NEW: return location;
        case EXISTING: return vh.p();
        }
        return location;
    }; //!< the location of the point
    Point p() { return getLocation(); }; //!< the location of the point
    IntersectionPointType getType() { return type; }; //!< the type of endpoint: existing vertex or new point

    IntersectionPoint(HE_VertexHandle vh)               : type(EXISTING), vh(vh),    edge(*vh.m, -1) {};
    IntersectionPoint(Point loc, HE_EdgeHandle edge)    : type(NEW), location(loc), edge(edge),     vh(*edge.m, -1) {};
    IntersectionPoint(Point loc, HE_EdgeHandle edge, HE_VertexHandle vh, IntersectionPointType type) : location(loc), edge(edge), vh(vh), type(type) {};

    IntersectionPoint* clone() { return new IntersectionPoint(location, edge, vh, type); };


};



enum IntersectionType { LINE_SEGMENT, COPLANAR, TOUCHING, NON_TOUCHING, NON_TOUCHING_PLANES, PARALLEL, UNKNOWN }; //!< type of (non-)intersection between triangles
static std::string toString(IntersectionType t)
{
    switch(t)
    {
	case LINE_SEGMENT: return "LINE_SEGMENT";
	case COPLANAR: return "COPLANAR";
	case TOUCHING: return "TOUCHING";
	case NON_TOUCHING: return "NON_TOUCHING";
	case NON_TOUCHING_PLANES: return "NON_TOUCHING_PLANES";
	case PARALLEL: return "PARALLEL";
	case UNKNOWN: return "UNKNOWN";
    }
};

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

    IntersectionType intersectionType;

    TriangleIntersection(boost::optional<IntersectionPoint> from_, boost::optional<IntersectionPoint> to_, bool inMesh2, bool inMesh1, IntersectionType intersectionType_)
    : from(from_)
    , to(to_)
    , isDirectionOfInnerPartOfTriangle1(inMesh2)
    , isDirectionOfInnerPartOfTriangle2(inMesh1)
    , intersectionType(intersectionType_)
    { };
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

        bool isCorrect; //!< whether the triangles are in a position such that they intersect

        IntersectionType intersectionType; //!< information on the intersection type between the two triangles, if already known.

        boost::optional<FPoint3> O; //!< any point on the line of the intersection between the two halfplanes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

        TrianglePlaneIntersection()
        : isCorrect(false)
        , intersectionType(UNKNOWN)
        , O(boost::none)
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
