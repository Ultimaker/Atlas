#include "triangleIntersect.h"


void TriangleIntersectionComputation::test()
{
    HE_Mesh mesh;

    mesh.vertices.emplace_back(Point(0  ,0  ,00), -1);
    mesh.vertices.emplace_back(Point(100000,100000,00), -1);
    mesh.vertices.emplace_back(Point(100000,0  ,00), -1);

    mesh.vertices.emplace_back(Point(0  ,0  ,-1000), -1);
    mesh.vertices.emplace_back(Point(100000,100000,1000), -1);
    mesh.vertices.emplace_back(Point(100000,0  ,1000), -1);


    mesh.createFace(0,1,2);
    mesh.createFace(3,4,5);

    HE_FaceHandle fh1(mesh, 0);
    HE_FaceHandle fh2(mesh, 1);

    TRIANGLE_INTERSECT_DEBUG_DO(mesh.debugOutputWholeMesh();)

    TriangleIntersection intersection = TriangleIntersectionComputation::intersect(fh1, fh2);
    TRIANGLE_INTERSECT_DEBUG_PRINTLN(" test finished ");

    TRIANGLE_INTERSECT_DEBUG_DO(
        if (intersection.intersectionType == LINE_SEGMENT)
        {
            TRIANGLE_INTERSECT_DEBUG_SHOW(EXISTING);
            TRIANGLE_INTERSECT_DEBUG_SHOW(NEW);
            if (intersection.from != nullptr)
            {
                TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.from->getType());
                TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.from->getLocation());
                if (intersection.from->getType() == NEW)
                    TRIANGLE_INTERSECT_DEBUG_SHOW(static_cast<NewIntersectionPoint*>(intersection.from.get())->edge.from_vert().p() );
            }

            if (intersection.to != nullptr)
            {
                TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.to->getType());
                TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.to->getLocation());
                if (intersection.to->getType() == NEW)
                    TRIANGLE_INTERSECT_DEBUG_SHOW(static_cast<NewIntersectionPoint*>(intersection.to.get())->edge.from_vert().p() );
            }
            TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.isDirectionOfInnerPartOfTriangle1);
            TRIANGLE_INTERSECT_DEBUG_SHOW(intersection.isDirectionOfInnerPartOfTriangle2);
        }
    )
    TRIANGLE_INTERSECT_DEBUG_PRINTLN("..======================================================================");
    TRIANGLE_INTERSECT_DEBUG_PRINTLN("|| type of triangle - triangle intersection : " << toString(intersection.intersectionType));
    TRIANGLE_INTERSECT_DEBUG_PRINTLN("''======================================================================");
}

TriangleIntersection TriangleIntersectionComputation::intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2)
{
    return intersect(fh1, fh2, boost::none);
}
TriangleIntersection TriangleIntersectionComputation::intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2, boost::optional<Point> some_point_on_planes_intersection_line)
{
    TRIANGLE_INTERSECT_DEBUG_PRINTLN("intersecting");
    //! see Tomas Moller - A Fast Triangle-Triangle Intersection Test



    FPoint a1(fh1.p0());
    FPoint b1(fh1.p1());
    FPoint c1(fh1.p2());
    FPoint a2(fh2.p0());
    FPoint b2(fh2.p1());
    FPoint c2(fh2.p2());

    TRIANGLE_INTERSECT_DEBUG_PRINTLN("init finished");

    FPoint ab1 = b1-a1;
    FPoint ac1 = c1-a1;

    FPoint ab2 = b2-a2;
    FPoint ac2 = c2-a2;

    FPoint n1 = ab1.cross(ac1).normalized();
    FPoint n2 = ab2.cross(ac2).normalized();

    TRIANGLE_INTERSECT_DEBUG_SHOW(n1);
    TRIANGLE_INTERSECT_DEBUG_SHOW(n2);
    TRIANGLE_INTERSECT_DEBUG_SHOW(a1);
    TRIANGLE_INTERSECT_DEBUG_SHOW(a2);

    auto resizeNormal = [](FPoint& n) { while (n.vSize2() > 100000 )    n *= .5; };


    if (n1==n2)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("parallel triangles! (or coplanar) ");
        return TriangleIntersection(nullptr, nullptr, false, false, PARALLEL); // parallel triangles! (also in the coplanar case we don't do anything)
    }


    float d1 = n1.dot(a1) * -1;
    TRIANGLE_INTERSECT_DEBUG_SHOW(d1);
    float d2 = n2.dot(a2) * -1;
    TRIANGLE_INTERSECT_DEBUG_SHOW(d2);

    auto dp1 = [&](FPoint& a) { return n1.dot(a) + d1; }; // distance to plane 2
    auto dp2 = [&](FPoint& a) { return n2.dot(a) + d2; }; // distance to plane 1

    auto sign = [](float a) { return char((0<a) - (a<0)); };

    char sa1 = sign(dp2(a1)); // sign of the distance
    char sb1 = sign(dp2(b1));
    char sc1 = sign(dp2(c1));

    char sa2 = sign(dp1(a2));
    char sb2 = sign(dp1(b2));
    char sc2 = sign(dp1(c2));

    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sa1));
    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sb1));
    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sc1));
    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sa2));
    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sb2));
    TRIANGLE_INTERSECT_DEBUG_SHOW(int(sc2));

    if (sa1 == sb1 && sb1 == sc1)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" no intersection, or coplanar! " << sa1);
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(dp2(a1) << "," << dp2(b1) << ","<< dp2(c1));
        return TriangleIntersection(nullptr, nullptr, false, false, NON_TOUCHING_PLANES); // no intersection (sign >0 or <0), or coplanar (sign=0)!
    }
    if (sa2 == sb2 && sb2 == sc2)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" no intersection! ");
        return TriangleIntersection(nullptr, nullptr, false, false, NON_TOUCHING_PLANES); // no intersection
    }

    FPoint O;


    TrianglePlaneIntersection tri1_plane2_ints = getIntersectingEdges(a1,b1,c1,sa1,sb1,sc1, fh1);
    if (! tri1_plane2_ints.isCorrect) return TriangleIntersection(nullptr, nullptr, false, false, tri1_plane2_ints.intersectionType);

    TrianglePlaneIntersection tri2_plane1_ints = getIntersectingEdges(a2,b2,c2,sa2,sb2,sc2, fh2);
    if (! tri2_plane1_ints.isCorrect) return TriangleIntersection(nullptr, nullptr, false, false, tri2_plane1_ints.intersectionType);



    FPoint n3u = n1.cross(n2); // unnormalized
    FPoint n3 = n3u.normalized(); // normal of a plane through the origin and perpendicular to plane 1 and 2. >> as 'D' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    // also: direction of the intersection between the two halfplanes

    TRIANGLE_INTERSECT_DEBUG_SHOW(n3.vSize2());
//    resizeNormal(n3);

TRIANGLE_INTERSECT_DEBUG_SHOW(n3);


    TRIANGLE_INTERSECT_DEBUG_PRINTLN(" O1 exists " << (bool(tri1_plane2_ints.O)));
    TRIANGLE_INTERSECT_DEBUG_PRINTLN(" O2 exists " << (bool(tri2_plane1_ints.O)));

    if (some_point_on_planes_intersection_line) O = *some_point_on_planes_intersection_line;
    else if (tri1_plane2_ints.O)                O = *tri1_plane2_ints.O;
    else if (tri2_plane1_ints.O)                O = *tri2_plane1_ints.O;
    else
    {
        FPoint d2n1 = d2 * n1 ;
        FPoint d1n2 = d1 * n2 ;
        TRIANGLE_INTERSECT_DEBUG_SHOW(d2n1);
        TRIANGLE_INTERSECT_DEBUG_SHOW(d1n2);
        FPoint p0 = (d2n1 - d1n2  ).cross(n3u) / n3u.vSize2() ; // as 'p0' in http://geomalgorithms.com/a05-_intersect-1.html : Intersection of 2 Planes . (C)
        O = p0; // as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    }

TRIANGLE_INTERSECT_DEBUG_PRINTLN("O = " << O);
TRIANGLE_INTERSECT_DEBUG_PRINTLN(" ");


    auto pL = [&](FPoint& a)
        {
            return n3.dot(a - O);
        };

    auto i1 = [&](FPoint& a, FPoint& b)
        {
            float pLa = pL(a);
            float pLb = pL(b);
            return pLa + (pLb - pLa) * dp2(a) / xType(dp2(a) - dp2(b));
        }; // intersect line from plane 1 with plane 2
    auto i2 = [&](FPoint& a, FPoint& b)
        {
            TRIANGLE_INTERSECT_DEBUG_PRINTLN("\n i2 for "<<&a << ","<<&b);
            TRIANGLE_INTERSECT_DEBUG_PRINTLN("\n i2 for "<<a << ","<<b);
            float pLa = pL(a);
            float pLb = pL(b);
            return pLa + (pLb - pLa) * dp1(a) / xType(dp1(a) - dp1(b));
        }; // intersect line from plane 2 with plane 1


    xType x11;
    xType x12;
    xType x21;
    xType x22;

    if (tri2_plane1_ints.line1.to.get() == tri2_plane1_ints.line2.from.get() && tri2_plane1_ints.line1.to)
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("WTF!");

    if (tri1_plane2_ints.line1.intersection->getType() == NEW) {
        x11 = i1(*tri1_plane2_ints.line1.from, *tri1_plane2_ints.line1.to);
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(O << " + " << x11 << " * "<<n3<<" = "<< (O + x11 * n3));
        static_cast<NewIntersectionPoint*> (tri1_plane2_ints.line1.intersection.get() )->location = (O + x11 * n3 /n3.vSize()).toPoint3();
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(tri1_plane2_ints.line1.intersection->p());
    } else {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection x11 from given vertex ");
        x11 = divide(FPoint(tri1_plane2_ints.line1.intersection->p()) - O , n3);
    }
    if (tri1_plane2_ints.line2.intersection->getType() == NEW) {
        x12 = i1(*tri1_plane2_ints.line2.from, *tri1_plane2_ints.line2.to);
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(O << " + " << x12 << " * "<<n3<<" = "<< (O + x12 * n3));
        static_cast<NewIntersectionPoint*> (tri1_plane2_ints.line2.intersection.get() )->location = (O + x12 * n3 /n3.vSize()).toPoint3();
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(tri1_plane2_ints.line2.intersection->p());
    } else {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection x12 from given vertex ");
        x12 = divide(FPoint(tri1_plane2_ints.line2.intersection->p()) - O , n3);
    }
    TRIANGLE_INTERSECT_DEBUG_SHOW(tri2_plane1_ints.line1.intersection.get());
    if (tri2_plane1_ints.line1.intersection == nullptr)
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" is null pointer!!!!@!! OMFG");

    if (tri2_plane1_ints.line1.intersection->getType() == NEW) {
        x21 = i2(*tri2_plane1_ints.line1.from, *tri2_plane1_ints.line1.to);
        static_cast<NewIntersectionPoint*> (tri2_plane1_ints.line1.intersection.get() )->location = (O + x21 * n3 /n3.vSize()).toPoint3();
    } else {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection x21 from given vertex ");
        x21 = divide(FPoint(tri2_plane1_ints.line1.intersection->p()) - O , n3);
    }
    if (tri2_plane1_ints.line2.intersection->getType() == NEW) {
        x22 = i2(*tri2_plane1_ints.line2.from, *tri2_plane1_ints.line2.to);
        static_cast<NewIntersectionPoint*> (tri2_plane1_ints.line2.intersection.get() )->location = (O + x22 * n3 /n3.vSize()).toPoint3();
    } else {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection x22 from given vertex ");
        x22 = divide(FPoint(tri2_plane1_ints.line2.intersection->p()) - O , n3);
    }


TRIANGLE_INTERSECT_DEBUG_PRINTLN(" ");
TRIANGLE_INTERSECT_DEBUG_PRINTLN("x11 = " << x11);
TRIANGLE_INTERSECT_DEBUG_PRINTLN("p11 = " << tri1_plane2_ints.line1.intersection->p());
TRIANGLE_INTERSECT_DEBUG_PRINTLN("x12 = " << x12);
TRIANGLE_INTERSECT_DEBUG_PRINTLN("p12 = " << tri1_plane2_ints.line2.intersection->p());
TRIANGLE_INTERSECT_DEBUG_PRINTLN("x21 = " << x21);
TRIANGLE_INTERSECT_DEBUG_PRINTLN("p21 = " << tri2_plane1_ints.line1.intersection->p());
TRIANGLE_INTERSECT_DEBUG_PRINTLN("x22 = " << x22);
TRIANGLE_INTERSECT_DEBUG_PRINTLN("p22 = " << tri2_plane1_ints.line2.intersection->p());


    if (x11 > x12)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("first intersections swapped");
        std::swap(x11,x12);
        std::swap(tri1_plane2_ints.line1, tri1_plane2_ints.line2);
        tri1_plane2_ints.isDirectionOfInnerFacePart = ! tri1_plane2_ints.isDirectionOfInnerFacePart;
    }
    if (x21 > x22)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("second intersections swapped");
        std::swap(x21,x22);
        std::swap(tri2_plane1_ints.line1, tri2_plane1_ints.line2);
        tri2_plane1_ints.isDirectionOfInnerFacePart = ! tri2_plane1_ints.isDirectionOfInnerFacePart;
    }

    if (x12 < x21 || x22 < x11)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("no overlap between line segments of intersections of triangles in the plane of the other !");
        return TriangleIntersection(nullptr, nullptr, false, false, NON_TOUCHING); // no overlap!
    }



    //return TriangleIntersection((x11 > x21)? tri1_plane2_ints.line1.intersection : tri2_plane1_ints.line1.intersection, (x12 < x22)? tri1_plane2_ints.line2.intersection : tri2_plane1_ints.line2.intersection);
    TriangleIntersection ret(
              std::move(( (x11 > x21)? tri1_plane2_ints : tri2_plane1_ints ).line1.intersection)
            , std::move(( (x12 < x22)? tri1_plane2_ints : tri2_plane1_ints ).line2.intersection)
            , tri1_plane2_ints.isDirectionOfInnerFacePart
            , tri2_plane1_ints.isDirectionOfInnerFacePart
            , LINE_SEGMENT
        );

    if ( ( ret.from->p() - ret.to->p() ) .testLength(MELD_DISTANCE))
    { // only return resulting line segment if it contains a vertex and another point (which is not the same vertex)
        if (ret.to->getType() == NEW && ret.from->getType() == NEW)
            return TriangleIntersection(nullptr, nullptr, false, false, TOUCHING);
        if (ret.to->getType() == EXISTING && ret.from->getType() == EXISTING)
            if (static_cast<ExistingVertexIntersectionPoint*>(ret.to.get() )->vh == static_cast<ExistingVertexIntersectionPoint*>(ret.from.get() )->vh  )
                return TriangleIntersection(nullptr, nullptr, false, false, TOUCHING);

    }

    TRIANGLE_INTERSECT_DEBUG_PRINTLN("finished!");

    return ret;
}



xType TriangleIntersectionComputation::divide(FPoint a, FPoint& b) //!< assumes the two vectors are in the same direction
{
    xType ret;
    spaceTypeD xx = b.x*b.x;
    spaceTypeD yy = b.y*b.y;
    spaceTypeD zz = b.z*b.z;
    if (xx >= yy && xx >= zz)
        ret = float(a.x) / b.x;
    else if (yy >= zz)
        ret =  float(a.y) / b.y;
    else
        ret = float(a.z) / b.z;

    TRIANGLE_INTERSECT_DEBUG_PRINTLN("\n dividing (" << a<<","<<b <<") = "<< ret);
    return ret;
};










TriangleIntersectionComputation::TrianglePlaneIntersection TriangleIntersectionComputation::getIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{

    TrianglePlaneIntersection ret;
    ret.computeIntersectingEdges(a,b,c,sa,sb,sc, fh);
    return ret;
}
void TriangleIntersectionComputation::TrianglePlaneIntersection::computeIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{
    isCorrect=false;

    if (sa == sb && sb == sc)
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN("Triangle doesn't intersect plane of other triangle, or is coplanar!");
        if (sa > 0)
        {
            intersectionType = NON_TOUCHING_PLANES;
            TRIANGLE_INTERSECT_DEBUG_PRINTLN(" ... triangle lies above");
        } else if (sa < 0)
        {
            intersectionType = NON_TOUCHING_PLANES;
            TRIANGLE_INTERSECT_DEBUG_PRINTLN(" ... triangle lies below");
        } else {
            intersectionType = COPLANAR;
            TRIANGLE_INTERSECT_DEBUG_PRINTLN(" ... triangle is coplanar");
        }
        return; // triangle doesn't intersect the plane of the other
    }

    // check which two lines of triangle  cross the plane of triangle , and check if the plane is crossed in a vertex
    if (sa == sb)
    {
        if (sa==0) // whole line lies on segment
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v0()));
            line2.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v1()));
            O = FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sc < 0;
            std::cerr<< "use line segment ab below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sc == 0)
            {
                TRIANGLE_INTERSECT_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                intersectionType = TOUCHING;
                return; // triangle doesn't cross the plane (only touches it)
            }
            line1.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge1()));
            line1.from  = FPoint3(b);
            line1.to    = FPoint3(c);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge2()));
            line2.from  = FPoint3(c);
            line2.to    = FPoint3(a);
            isDirectionOfInnerFacePart = sc > 0;
        }

    }
    else if (sb == sc)
    {
        if (sb==0) // whole line lies on segment
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v1()));
            line2.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v2()));
            O = FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sa < 0;
            std::cerr<< "use line segment bc below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sa == 0)
            {
                TRIANGLE_INTERSECT_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                intersectionType = TOUCHING;
                return; // triangle doesn't cross the plane (only touches it)
            }
            line1.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge2()));
            line1.from  = FPoint3(c);
            line1.to    = FPoint3(a);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge0()));
            line2.from  = FPoint3(a);
            line2.to    = FPoint3(b);
            isDirectionOfInnerFacePart = sa > 0;
        }

    }
    else if (sc == sa)
    {
        if (sc==0) // whole line lies on segment
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v2()));
            line2.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v0()));
            O = FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sb < 0;
            std::cerr<< "use line segment ac below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sb == 0)
            {
                TRIANGLE_INTERSECT_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                intersectionType = TOUCHING;
                return; // triangle doesn't cross the plane (only touches it)
            }
            line1.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge0()));
            line1.from  = FPoint3(a);
            line1.to    = FPoint3(b);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge1()));
            line2.from  = FPoint3(b);
            line2.to    = FPoint3(c);
            isDirectionOfInnerFacePart = sb > 0;
        }

    } else // sa =/= sb =/= sc =/= sa  : all unequal, so one point must lie in the middle ON the plane
    {
        if (sa == 0)
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v0()));
            TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection from given vertex " << fh.v0().idx);
            O = FPoint(line1.intersection->p());
            line2.from = FPoint3(b);
            line2.to   = FPoint3(c);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge1()));
            isDirectionOfInnerFacePart = sb > 0;
        }
        else if (sb == 0)
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v1()));
            TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection from given vertex " << fh.v1().idx);
            O = FPoint(line1.intersection->p());
            line2.from  = FPoint3(c);
            line2.to    = FPoint3(a);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge2()));
            isDirectionOfInnerFacePart = sc > 0;
        }
        else if (sc == 0)
        {
            line1.intersection = std::unique_ptr<IntersectionPoint>(new ExistingVertexIntersectionPoint(fh.v2()));
            TRIANGLE_INTERSECT_DEBUG_PRINTLN("using intersection from given vertex " << fh.v2().idx);
            O = FPoint(line1.intersection->p());
            line2.from  = FPoint3(a);
            line2.to    = FPoint3(b);
            line2.intersection = std::unique_ptr<IntersectionPoint>(new NewIntersectionPoint(Point(0,0,0), fh.edge0()));
            isDirectionOfInnerFacePart = sa > 0;
        } else
        {
            TRIANGLE_INTERSECT_DEBUG_PRINTLN(" uncaught case! end of boolMesh.cpp...");
        }
    }

    if (!line1.intersection)
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" no line1.intersection !!!!!! ");
    else if (!line2.intersection)
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" no line2.intersection !!!!!! ");
    else
    {
        TRIANGLE_INTERSECT_DEBUG_PRINTLN(" we got intersections :) ");
        isCorrect = true;
    }

}
