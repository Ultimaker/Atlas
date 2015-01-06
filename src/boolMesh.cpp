#include "boolMesh.h"


void BooleanFVMeshOps::subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result)
{
//! is more efficient when keep is smaller than subtracted.

/*
1) Find every pair of triangles that intersect
2) Compute the intersections as segments
3) For each intersected triangle, store the intersection segments and orient them
4) Triangulate the intersected facets and add to the solution the triangles that belong to it.
5) Propagate the result to the whole polyhedra, using the connectivity of the facets
*/

    // : \/ construct an AABB tree from the keep-mesh
    typedef AABB_Tree<HE_FaceHandle>::Node Node;

    std::vector<Node*> leaves;
    for (int f = 0; f < keep.faces.size(); f++)
    {
        HE_FaceHandle fh(keep, f);
        leaves.push_back(new Node(keep.computeFaceBbox(f), &fh));
    }

    AABB_Tree<HE_FaceHandle> keep_aabb(leaves);

    for (int f = 0 ; f < subtracted.faces.size() ; f++)
    {
        BoundingBox tribbox = subtracted.computeFaceBbox(f);
        std::vector<HE_FaceHandle*> intersectingFaces;
        keep_aabb.getIntersections(tribbox, intersectingFaces);

        HE_FaceHandle face(subtracted, f);

    }

}


void BooleanFVMeshOps::test()
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

    BOOL_MESH_DEBUG_DO(mesh.debugOutputWholeMesh();)

    TriangleIntersection* intersection = BooleanFVMeshOps::intersect(fh1, fh2);
    BOOL_MESH_DEBUG_PRINTLN(" test finished ");

    BOOL_MESH_DEBUG_DO(
        if (intersection != nullptr)
        {
            BOOL_MESH_DEBUG_SHOW(EXISTING);
            BOOL_MESH_DEBUG_SHOW(NEW);
            if (intersection->from != nullptr)
            {
                BOOL_MESH_DEBUG_SHOW(intersection->from->getType());
                BOOL_MESH_DEBUG_SHOW(intersection->from->getLocation());
                if (intersection->from->getType() == NEW)
                    BOOL_MESH_DEBUG_SHOW(static_cast<NewIntersectionPoint*>(intersection->from)->edge.from_vert().p() );
            }

            if (intersection->to != nullptr)
            {
                BOOL_MESH_DEBUG_SHOW(intersection->to->getType());
                BOOL_MESH_DEBUG_SHOW(intersection->to->getLocation());
                if (intersection->to->getType() == NEW)
                    BOOL_MESH_DEBUG_SHOW(static_cast<NewIntersectionPoint*>(intersection->to)->edge.from_vert().p() );
            }
            BOOL_MESH_DEBUG_SHOW(intersection->isDirectionOfInnerPartOfTriangle1);
            BOOL_MESH_DEBUG_SHOW(intersection->isDirectionOfInnerPartOfTriangle2);
        }
    )
}

TriangleIntersection* BooleanFVMeshOps::intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2)
{
    BOOL_MESH_DEBUG_PRINTLN("intersecting");
    //! see Tomas Moller - A Fast Triangle-Triangle Intersection Test



    FPoint a1(fh1.p0());
    FPoint b1(fh1.p1());
    FPoint c1(fh1.p2());
    FPoint a2(fh2.p0());
    FPoint b2(fh2.p1());
    FPoint c2(fh2.p2());

    BOOL_MESH_DEBUG_PRINTLN("init finished");

    FPoint ab1 = b1-a1;
    FPoint ac1 = c1-a1;

    FPoint ab2 = b2-a2;
    FPoint ac2 = c2-a2;

    FPoint n1 = ab1.cross(ac1).normalized();
    FPoint n2 = ab2.cross(ac2).normalized();

    BOOL_MESH_DEBUG_SHOW(n1);
    BOOL_MESH_DEBUG_SHOW(n2);
    BOOL_MESH_DEBUG_SHOW(a1);
    BOOL_MESH_DEBUG_SHOW(a2);

    //auto resizeNormal = [](FPoint& n) { while (n.vSize2() > std::numeric_limits<spaceType>::max()/4 )    n /=2; };
    auto resizeNormal = [](FPoint& n) { while (n.vSize2() > 100000 )    n *= .5; };
    //auto resizeNormal = [](Point& n) { return; };

//    resizeNormal(n1);
//    resizeNormal(n2);

    if (n1==n2)
    {
        BOOL_MESH_DEBUG_PRINTLN("parallel triangles! (or coplanar) ");
        return nullptr; // parallel triangles! (also in the coplanar case we don't do anything)
    }


    float d1 = n1.dot(a1) * -1;
    BOOL_MESH_DEBUG_SHOW(d1);
    float d2 = n2.dot(a2) * -1;
    BOOL_MESH_DEBUG_SHOW(d2);

    auto dp1 = [&](FPoint& a) { return n1.dot(a) + d1; }; // distance to plane 2
    auto dp2 = [&](FPoint& a) { return n2.dot(a) + d2; }; // distance to plane 1

    auto sign = [](float a) { return char((0<a) - (a<0)); };

    char sa1 = sign(dp2(a1));
    char sb1 = sign(dp2(b1));
    char sc1 = sign(dp2(c1));

    char sa2 = sign(dp1(a2));
    char sb2 = sign(dp1(b2));
    char sc2 = sign(dp1(c2));

    BOOL_MESH_DEBUG_SHOW(int(sa1));
    BOOL_MESH_DEBUG_SHOW(int(sb1));
    BOOL_MESH_DEBUG_SHOW(int(sc1));
    BOOL_MESH_DEBUG_SHOW(int(sa2));
    BOOL_MESH_DEBUG_SHOW(int(sb2));
    BOOL_MESH_DEBUG_SHOW(int(sc2));

    if (sa1 == sb1 && sb1 == sc1)
    {
        BOOL_MESH_DEBUG_PRINTLN(" no intersection, or coplanar! " << sa1);
        BOOL_MESH_DEBUG_PRINTLN(dp2(a1) << "," << dp2(b1) << ","<< dp2(c1));
        return nullptr; // no intersection (sign >0 or <0), or coplanar (sign=0)!
    }
    if (sa2 == sb2 && sb2 == sc2)
    {
        BOOL_MESH_DEBUG_PRINTLN(" no intersection! ");
        return nullptr; // no intersection
    }

    //Point* O = nullptr; // any point on the line of the intersection between the two planes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    FPoint O;




    BOOL_MESH_DEBUG_PRINTLN("edges1 getIntersectingEdges");
    IntersectionEnv edges1 = getIntersectingEdges(a1,b1,c1,sa1,sb1,sc1, fh1);
    if (! edges1.isCorrect) return nullptr;
    BOOL_MESH_DEBUG_PRINTLN("edges2 getIntersectingEdges");
    IntersectionEnv edges2 = getIntersectingEdges(a2,b2,c2,sa2,sb2,sc2, fh2);
    if (! edges2.isCorrect) return nullptr;

    BOOL_MESH_DEBUG_PRINTLN("getIntersectingEdges finished");



    FPoint n3u = n1.cross(n2); // unnormalized
    FPoint n3 = n3u.normalized(); // normal of a plane through the origin and perpendicular to plane 1 and 2. >> as 'D' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    BOOL_MESH_DEBUG_SHOW(n3.vSize2());
//    resizeNormal(n3);

BOOL_MESH_DEBUG_SHOW(n3);


    BOOL_MESH_DEBUG_PRINTLN(" O1 exists " << (edges1.O != nullptr));
    BOOL_MESH_DEBUG_PRINTLN(" O2 exists " << (edges2.O != nullptr));

    if (edges1.O != nullptr)        O = *edges1.O;
    else if (edges2.O != nullptr)   O = *edges2.O;
    else
    {
        FPoint d2n1 = d2 * n1 ;
        FPoint d1n2 = d1 * n2 ;
        BOOL_MESH_DEBUG_SHOW(d2n1);
        BOOL_MESH_DEBUG_SHOW(d1n2);
        FPoint p0 = (d2n1 - d1n2  ).cross(n3u) / n3u.vSize2() ; // as 'p0' in http://geomalgorithms.com/a05-_intersect-1.html : Intersection of 2 Planes . (C)
        O = p0; // as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    }

BOOL_MESH_DEBUG_PRINTLN("O = " << O);
BOOL_MESH_DEBUG_PRINTLN(" ");


    auto pL = [&](FPoint& a)
        {
            return n3.dot(a - O);
        };

    //spaceTypeD x = pL(a1) + (pL(b1) - pL(a1)) * dp2(a1) / (dp2(a1) - dp2(b1)); // as 't1' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

    // typedef double xType; // => class member typedef..

    auto i1 = [&](FPoint& a, FPoint& b)
        {
            float pLa = pL(a);
            float pLb = pL(b);
            return pLa + (pLb - pLa) * dp2(a) / xType(dp2(a) - dp2(b));
        }; // intersect line from plane 1 with plane 2
    auto i2 = [&](FPoint& a, FPoint& b)
        {
            BOOL_MESH_DEBUG_PRINTLN("\n i2 for "<<&a << ","<<&b);
            BOOL_MESH_DEBUG_PRINTLN("\n i2 for "<<a << ","<<b);
            float pLa = pL(a);
            float pLb = pL(b);
            return pLa + (pLb - pLa) * dp1(a) / xType(dp1(a) - dp1(b));
        }; // intersect line from plane 2 with plane 1


    xType x11;
    xType x12;
    xType x21;
    xType x22;

    if (edges2.line1.to == edges2.line2.from && edges2.line1.to != nullptr)
        BOOL_MESH_DEBUG_PRINTLN("WTF!");

    if (edges1.line1.intersection->getType() == NEW) {
        x11 = i1(*edges1.line1.from, *edges1.line1.to);
        BOOL_MESH_DEBUG_PRINTLN(O << " + " << x11 << " * "<<n3<<" = "<< (O + x11 * n3));
        static_cast<NewIntersectionPoint*> (edges1.line1.intersection )->location = (O + x11 * n3 /n3.vSize()).toPoint3();
        BOOL_MESH_DEBUG_PRINTLN(edges1.line1.intersection->p());
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x11 from given vertex ");
        x11 = divide(FPoint(edges1.line1.intersection->p()) - O , n3);
    }
    if (edges1.line2.intersection->getType() == NEW) {
        x12 = i1(*edges1.line2.from, *edges1.line2.to);
        BOOL_MESH_DEBUG_PRINTLN(O << " + " << x12 << " * "<<n3<<" = "<< (O + x12 * n3));
        static_cast<NewIntersectionPoint*> (edges1.line2.intersection )->location = (O + x12 * n3 /n3.vSize()).toPoint3();
        BOOL_MESH_DEBUG_PRINTLN(edges1.line2.intersection->p());
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x12 from given vertex ");
        x12 = divide(FPoint(edges1.line2.intersection->p()) - O , n3);
    }
    BOOL_MESH_DEBUG_SHOW(edges2.line1.intersection);
    if (edges2.line1.intersection == nullptr)
        BOOL_MESH_DEBUG_PRINTLN(" is null pointer!!!!@!! OMFG");

    if (edges2.line1.intersection->getType() == NEW) {
        x21 = i2(*edges2.line1.from, *edges2.line1.to);
        static_cast<NewIntersectionPoint*> (edges2.line1.intersection )->location = (O + x21 * n3 /n3.vSize()).toPoint3();
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x21 from given vertex ");
        x21 = divide(FPoint(edges2.line1.intersection->p()) - O , n3);
    }
    if (edges2.line2.intersection->getType() == NEW) {
        x22 = i2(*edges2.line2.from, *edges2.line2.to);
        static_cast<NewIntersectionPoint*> (edges2.line2.intersection )->location = (O + x22 * n3 /n3.vSize()).toPoint3();
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x22 from given vertex ");
        x22 = divide(FPoint(edges2.line2.intersection->p()) - O , n3);
    }


BOOL_MESH_DEBUG_PRINTLN(" ");
BOOL_MESH_DEBUG_PRINTLN("x11 = " << x11);
BOOL_MESH_DEBUG_PRINTLN("p11 = " << edges1.line1.intersection->p());
BOOL_MESH_DEBUG_PRINTLN("x12 = " << x12);
BOOL_MESH_DEBUG_PRINTLN("p12 = " << edges1.line2.intersection->p());
BOOL_MESH_DEBUG_PRINTLN("x21 = " << x21);
BOOL_MESH_DEBUG_PRINTLN("p21 = " << edges2.line1.intersection->p());
BOOL_MESH_DEBUG_PRINTLN("x22 = " << x22);
BOOL_MESH_DEBUG_PRINTLN("p22 = " << edges2.line2.intersection->p());


    if (x11 > x12)
    {
        BOOL_MESH_DEBUG_PRINTLN("first intersections swapped");
        std::swap(x11,x12);
        std::swap(edges1.line1, edges1.line2);
        edges1.isDirectionOfInnerFacePart = ! edges1.isDirectionOfInnerFacePart;
    }
    if (x21 > x22)
    {
        BOOL_MESH_DEBUG_PRINTLN("second intersections swapped");
        std::swap(x21,x22);
        std::swap(edges2.line1, edges2.line2);
        edges2.isDirectionOfInnerFacePart = ! edges2.isDirectionOfInnerFacePart;
    }

    if (x12 < x21 || x22 < x11)
    {
        BOOL_MESH_DEBUG_PRINTLN("no overlap between line segments of intersections of triangles in the plane of the other !");
        return nullptr; // no overlap!
    }


//    if (x11 > x21)
//    {
//        std::cerr << "from first = "<< edges1.line1.intersection->p() <<std::endl;
//    } else
//    {
//        std::cerr << "from second = "<< edges2.line1.intersection->p() <<std::endl;
//    }
//
//    if (x12 < x22)
//    {
//        std::cerr << "to first = "<< edges1.line2.intersection->p() <<std::endl;
//    } else
//    {
//        std::cerr << "to second = "<< edges2.line2.intersection->p() <<std::endl;
//    }
//


    //return TriangleIntersection((x11 > x21)? edges1.line1.intersection : edges2.line1.intersection, (x12 < x22)? edges1.line2.intersection : edges2.line2.intersection);
    TriangleIntersection* ret = new TriangleIntersection(
            ( (x11 > x21)? edges1 : edges2 ).line1.intersection->copy()
            , ( (x12 < x22)? edges1 : edges2 ).line2.intersection->copy()
            , edges1.isDirectionOfInnerFacePart
            , edges2.isDirectionOfInnerFacePart
        );

    if ( ( ret->from->p() - ret->to->p() ) .testLength(MELD_DISTANCE))
    { // only return resulting line segment if it contains a vertex and another point (which is not the same vertex)
        if (ret->to->getType() == NEW && ret->from->getType() == NEW)
            return nullptr;
        if (ret->to->getType() == EXISTING && ret->from->getType() == EXISTING)
            if (static_cast<ExistingVertexIntersectionPoint*>(ret->to)->vh == static_cast<ExistingVertexIntersectionPoint*>(ret->from)->vh  )
                return nullptr;

    }
    BOOL_MESH_DEBUG_PRINTLN("finished!");

    return ret;
}



xType BooleanFVMeshOps::divide(FPoint a, FPoint& b) //!< assumes the two vectors are in the same direction
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

    BOOL_MESH_DEBUG_PRINTLN("\n dividing (" << a<<","<<b <<") = "<< ret);
    return ret;
}












BooleanFVMeshOps::IntersectionEnv BooleanFVMeshOps::getIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{

    IntersectionEnv ret;
    ret.computeIntersectingEdges(a,b,c,sa,sb,sc, fh);
    return ret;
}
void BooleanFVMeshOps::IntersectionEnv::computeIntersectingEdges(FPoint3& a, FPoint3& b, FPoint3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{
    isCorrect=false;

    if (sa == sb && sb == sc)
    {
        BOOL_MESH_DEBUG_PRINTLN("Triangle doesn't intersect plane of other triangle, or is coplanar!");
        if (sa > 0)
            BOOL_MESH_DEBUG_PRINTLN(" ... triangle lies above");
        else if (sa < 0)
            BOOL_MESH_DEBUG_PRINTLN(" ... triangle lies below");
        else
            BOOL_MESH_DEBUG_PRINTLN(" ... triangle is coplanar");
        return; // triangle doesn't intersect the plane of the other
    }

    // check which two lines of triangle  cross the plane of triangle , and check if the plane is crossed in a vertex
    if (sa == sb)
    {
        if (sa==0) // whole line lies on segment
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v0());
            line2.intersection = new ExistingVertexIntersectionPoint(fh.v1());
            O = new FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sc < 0;
            std::cerr<< "use line segment ab below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sc == 0)
            {
                BOOL_MESH_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                return; // triangle doesn't cross the plane (only touches it)
            }
            line1.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge1());
            line1.from  = new FPoint3(b);
            line1.to    = new FPoint3(c);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge2());
            line2.from  = new FPoint3(c);
            line2.to    = new FPoint3(a);
            isDirectionOfInnerFacePart = sc > 0;
        }

    }
    else if (sb == sc)
    {
        if (sb==0) // whole line lies on segment
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v1());
            line2.intersection = new ExistingVertexIntersectionPoint(fh.v2());
            O = new FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sa < 0;
            std::cerr<< "use line segment bc below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sa == 0)
            {
                BOOL_MESH_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                return; // triangle doesn't cross the plane (only touches it)
            }            line1.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge2());
            line1.from  = new FPoint3(c);
            line1.to    = new FPoint3(a);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge0());
            line2.from  = new FPoint3(a);
            line2.to    = new FPoint3(b);
            isDirectionOfInnerFacePart = sa > 0;
        }

    }
    else if (sc == sa)
    {
        if (sc==0) // whole line lies on segment
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v2());
            line2.intersection = new ExistingVertexIntersectionPoint(fh.v0());
            O = new FPoint(line2.intersection->p());
            isDirectionOfInnerFacePart = sb < 0;
            std::cerr<< "use line segment ac below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sb == 0)
            {
                BOOL_MESH_DEBUG_PRINTLN(" triangle doesn't cross the plane (only touches it)");
                return; // triangle doesn't cross the plane (only touches it)
            }            line1.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge0());
            line1.from  = new FPoint3(a);
            line1.to    = new FPoint3(b);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge1());
            line2.from  = new FPoint3(b);
            line2.to    = new FPoint3(c);
            isDirectionOfInnerFacePart = sb > 0;
        }

    } else // sa =/= sb =/= sc =/= sa  : all unequal, so one point must lie in the middle ON the plane
    {
        if (sa == 0)
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v0());
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v0().idx);
            O = new FPoint(line1.intersection->p());
            line2.from = new FPoint3(b);
            line2.to   = new FPoint3(c);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge1());
            isDirectionOfInnerFacePart = sb > 0;
        }
        else if (sb == 0)
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v1());
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v1().idx);
            O = new FPoint(line1.intersection->p());
            line2.from  = new FPoint3(c);
            line2.to    = new FPoint3(a);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge2());
            isDirectionOfInnerFacePart = sc > 0;
        }
        else if (sc == 0)
        {
            line1.intersection = new ExistingVertexIntersectionPoint(fh.v2());
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v2().idx);
            O = new FPoint(line1.intersection->p());
            line2.from  = new FPoint3(a);
            line2.to    = new FPoint3(b);
            line2.intersection = new NewIntersectionPoint(Point(0,0,0), fh.edge0());
            isDirectionOfInnerFacePart = sa > 0;
        } else
        {
            BOOL_MESH_DEBUG_PRINTLN(" uncaught case! end of boolMesh.cpp...");
        }
    }

    if (line1.intersection == nullptr)
        BOOL_MESH_DEBUG_PRINTLN(" no line1.intersection !!!!!! ");
    else if (line2.intersection == nullptr)
        BOOL_MESH_DEBUG_PRINTLN(" no line2.intersection !!!!!! ");
    else
    {
        BOOL_MESH_DEBUG_PRINTLN(" we got intersections :) ");
        isCorrect = true;
    }

}
