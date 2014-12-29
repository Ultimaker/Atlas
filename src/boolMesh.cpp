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

    mesh.vertices.emplace_back(Point(0  ,0  ,0), -1);
    mesh.vertices.emplace_back(Point(10000,0  ,0), -1);
    mesh.vertices.emplace_back(Point(10000,10000,0), -1);

    mesh.vertices.emplace_back(Point(8000,   0,10000), -1);
    mesh.vertices.emplace_back(Point(0,   8000,10000), -1);
    mesh.vertices.emplace_back(Point(8000,   0,-10000), -1);

    mesh.createFace(0,1,2);
    mesh.createFace(3,4,5);

    HE_FaceHandle fh1(mesh, 0);
    HE_FaceHandle fh2(mesh, 1);

    BOOL_MESH_DEBUG_DO(mesh.debugOutputWholeMesh();)

    BooleanFVMeshOps::intersect(fh1, fh2);
}

void BooleanFVMeshOps::intersect(HE_FaceHandle& fh1, HE_FaceHandle& fh2)
{
    BOOL_MESH_DEBUG_PRINTLN("intersecting");
    //! see Tomas Moller - A Fast Triangle-Triangle Intersection Test

    Point a1 = fh1.p0();
    Point b1 = fh1.p1();
    Point c1 = fh1.p2();
    Point a2 = fh2.p0();
    Point b2 = fh2.p1();
    Point c2 = fh2.p2();

    BOOL_MESH_DEBUG_PRINTLN("init finished");

    Point ab1 = b1-a1;
    Point ac1 = c1-a1;

    Point ab2 = b2-a2;
    Point ac2 = c2-a2;

    Point n1 = ab1.cross(ac1);
    Point n2 = ab2.cross(ac2);

    auto resizeNormal = [](Point& n) { while (n.vSize2() > std::numeric_limits<spaceType>::max()/4 )    n /=2; };
    //auto resizeNormal = [](Point& n) { while (n.vSize2() > 3545354 )    n /=2; };
    //auto resizeNormal = [](Point& n) { return; };

    resizeNormal(n1);
    resizeNormal(n2);

    if (n1==n2) return; // parallel triangles! (also in the coplanar case we don't do anything)


    spaceTypeD d1 = n1.dot(a1) * -1;
    BOOL_MESH_DEBUG_SHOW(d1);
    spaceTypeD d2 = n2.dot(a2) * -1;
    BOOL_MESH_DEBUG_SHOW(d2);

    auto dp1 = [&](Point& a) { return n1.dot(a) + d1; }; // distance to plane 2
    auto dp2 = [&](Point& a) { return n2.dot(a) + d2; }; // distance to plane 1

    auto sign = [](spaceTypeD a) { return char((0<a) - (a<0)); };

    char sa1 = sign(dp2(a1));
    char sb1 = sign(dp2(b1));
    char sc1 = sign(dp2(c1));

    char sa2 = sign(dp1(a2));
    char sb2 = sign(dp1(b2));
    char sc2 = sign(dp1(c2));

    if (sa1 == sb1 && sb1 == sc1)
    {
        BOOL_MESH_DEBUG_PRINTLN(" no intersection, or coplanar! " << sa1);
        BOOL_MESH_DEBUG_PRINTLN(dp2(a1) << "," << dp2(b1) << ","<< dp2(c1));
        return; // no intersection (sign >0 or <0), or coplanar (sign=0)!
    }
    if (sa2 == sb2 && sb2 == sc2)
    {
        BOOL_MESH_DEBUG_PRINTLN(" no intersection! ");
        return; // no intersection
    }

    //Point* O = nullptr; // any point on the line of the intersection between the two planes. >> as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    Point O;

/* // check which two lines of triangle 2 cross the plane of triangle 2, and check if the plane is crossed in a vertex
    Point3* line11from; // the two intersecting lines from triangle 1
    Point3* line11to;
    Point3* line12from;
    Point3* line12to;

    HE_Vertex* intersection11 = nullptr; // reference to existing vertex or to new placeholder vertex not yet inserted into the mesh
    HE_Vertex* intersection12 = nullptr; // reference to existing vertex or to new placeholder vertex not yet inserted into the mesh

    // check which two lines of triangle 1 cross the plane of triangle 2, and check if the plane is crossed in a vertex
    if (sa1 == sb1)
    {
        if (sa1==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ab1 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sc1 == 0) return; // triangle doesn't cross the plane (only touches it)
            line11from  = &a1;
            line11to    = &c1;
            line12from  = &c1
            line12to    = &b1;
        }

    }
    else if (sa1 == sc1)
    {
        if (sa1==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ac1 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sb1 == 0) return; // triangle doesn't cross the plane (only touches it)
            line11from  = &a1;
            line11to    = &b1;
            line12from  = &c1
            line12to    = &b1;
        }

    }
    else if (sb1 == sc1)
    {
        if (sb1==0) // whole line lies on segment
        {
            std::cerr<< "use line segment bc1 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sa1 == 0) return; // triangle doesn't cross the plane (only touches it)
            line11from  = &a1;
            line11to    = &b1;
            line12from  = &a1;
            line12to    = &c1;
        }

    }
    else // sa1 =/= sb1 =/= sc1 =/= sa1  : all unequal, so one point must lie in the middle ON the plane
    {
        if (sa1 == 0)
        {
            intersection11 = &fh1.v0().vertex;
            line12from = &c1;
            line12to   = &b1;
        }
        else if (sb1 == 0)
        {
            intersection11 = &fh1.v1().vertex;
            line12from  = &a1;
            line12to    = &c1;
        }
        else if (sc1 == 0)
        {
            intersection11 = &fh1.v2().vertex;
            line12from  = &a1;
            line12to    = &b1;
        }
    }


    Point3* line21from; // the two intersecting lines from triangle 2
    Point3* line21to;
    Point3* line22from;
    Point3* line22to;

    HE_Vertex* intersection21 = nullptr; // reference to existing vertex or to new placeholder vertex not yet inserted into the mesh
    HE_Vertex* intersection22 = nullptr; // reference to existing vertex or to new placeholder vertex not yet inserted into the mesh

    // check which two lines of triangle 2 cross the plane of triangle 2, and check if the plane is crossed in a vertex
    if (sa2 == sb2)
    {
        if (sa2==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ab2 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sc2 == 0) return; // triangle doesn't cross the plane (only touches it)
            line21from  = &a2;
            line21to    = &c2;
            line22from  = &c2
            line22to    = &b2;
        }

    }
    else if (sa2 == sc2)
    {
        if (sa2==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ac2 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sb2 == 0) return; // triangle doesn't cross the plane (only touches it)
            line21from  = &a2;
            line21to    = &b2;
            line22from  = &c2
            line22to    = &b2;
        }

    }
    else if (sb2 == sc2)
    {
        if (sb2==0) // whole line lies on segment
        {
            std::cerr<< "use line segment bc2 below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sa2 == 0) return; // triangle doesn't cross the plane (only touches it)
            line21from  = &a2;
            line21to    = &b2;
            line22from  = &a2;
            line22to    = &c2;
        }

    }
    else // sa2 =/= sb2 =/= sc2 =/= sa2  : all une1ual, so one point must lie in the middle ON the plane
    {
        if (sa2 == 0)
        {
            intersection21 = &fh2.v0().vertex;
            line22from = &c2;
            line22to   = &b2;
        }
        else if (sb2 == 0)
        {
            intersection21 = &fh2.v2().vertex;
            line22from  = &a2;
            line22to    = &c2;
        }
        else if (sc2 == 0)
        {
            intersection21 = &fh2.v2().vertex;
            line22from  = &a2;
            line22to    = &b2;
        }
    }

*/


    IntersectionEnv edges1 = getIntersectingEdges(a1,b1,c1,sa1,sb1,sc1, fh1);
    IntersectionEnv edges2 = getIntersectingEdges(a2,b2,c2,sa2,sb2,sc2, fh2);

    BOOL_MESH_DEBUG_PRINTLN("getIntersectingEdges finished");



    Point n3 = n1.cross(n2); // normal of a plane through the origin and perpendicular to plane 1 and 2. >> as 'D' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

    resizeNormal(n3);

BOOL_MESH_DEBUG_PRINTLN("n3 = "<<n3);


    BOOL_MESH_DEBUG_PRINTLN(" O1 = " << edges1.O);
    BOOL_MESH_DEBUG_PRINTLN(" O2 = " << edges2.O);

    if (edges1.O != nullptr)        O = *edges1.O;
    else if (edges2.O != nullptr)   O = *edges2.O;
    else
    {
        PointD d2n1 = d2 * PointD(n1) / n1.vSize();
        PointD d1n2 = d1 * PointD(n2) / n2.vSize();
        BOOL_MESH_DEBUG_SHOW(d2);
        BOOL_MESH_DEBUG_SHOW(n1);
        BOOL_MESH_DEBUG_SHOW(d2n1);
        BOOL_MESH_DEBUG_SHOW(d1);
        BOOL_MESH_DEBUG_SHOW(n2);
        BOOL_MESH_DEBUG_SHOW(d1n2);
        BOOL_MESH_DEBUG_SHOW(d2n1 - d1n2);
        BOOL_MESH_DEBUG_SHOW((d2n1 - d1n2 ).cross(n3));
        BOOL_MESH_DEBUG_SHOW((d2n1 - d1n2 ).cross(n3) / n3.vSize2());
        Point p0 = (  (d2n1 - d1n2  ).cross(n3) / n3.vSize2()  ).downCast(); // as 'p0' in http://geomalgorithms.com/a05-_intersect-1.html : Intersection of 2 Planes . (C)
        O = p0; // as 'O' in Tomas Moller - A Fast Triangle-Triangle Intersection Test
    }

BOOL_MESH_DEBUG_PRINTLN("O = " << O);
BOOL_MESH_DEBUG_PRINTLN(" ");


    auto pL = [&](Point& a)
        {
//            BOOL_MESH_DEBUG_PRINTLN("pL(a)");
//            BOOL_MESH_DEBUG_SHOW(a);
//            BOOL_MESH_DEBUG_SHOW(n3);
//            BOOL_MESH_DEBUG_SHOW(O);
//            BOOL_MESH_DEBUG_SHOW(a-O);
//            BOOL_MESH_DEBUG_SHOW(n3.vSize());
//            BOOL_MESH_DEBUG_PRINTLN("\\end pL(.)\n");
            return n3.dot(a - O)  / n3.vSize() ;
        };

    //spaceTypeD x = pL(a1) + (pL(b1) - pL(a1)) * dp2(a1) / (dp2(a1) - dp2(b1)); // as 't1' in Tomas Moller - A Fast Triangle-Triangle Intersection Test

    // typedef double xType; // => class member typedef..

    auto i1 = [&](Point& a, Point& b)
        {
            spaceTypeD pLa = pL(a);
            spaceTypeD pLb = pL(b);
//            BOOL_MESH_DEBUG_PRINTLN("\n i1 for "<<a << ","<<b);
//            BOOL_MESH_DEBUG_SHOW(pLa);
//            BOOL_MESH_DEBUG_SHOW(pLb);
//            BOOL_MESH_DEBUG_SHOW(dp2(a));
//            BOOL_MESH_DEBUG_SHOW(dp2(b));
//            BOOL_MESH_DEBUG_SHOW(pLa + (pLb - pLa) * dp2(a) / xType(dp2(a) - dp2(b)) );
            return pLa + (pLb - pLa) * dp2(a) / xType(dp2(a) - dp2(b));
        }; // intersect line from plane 1 with plane 2
    auto i2 = [&](Point& a, Point& b)
        {
            BOOL_MESH_DEBUG_PRINTLN("\n i2 for "<<a << ","<<b);
            spaceTypeD pLa = pL(a);
            spaceTypeD pLb = pL(b);
//            BOOL_MESH_DEBUG_SHOW(pLa);
//            BOOL_MESH_DEBUG_SHOW(pLb);
//            BOOL_MESH_DEBUG_SHOW(dp1(a));
//            BOOL_MESH_DEBUG_SHOW(dp1(b));
//            BOOL_MESH_DEBUG_SHOW(pLa + (pLb - pLa) * dp1(a) / xType(dp1(a) - dp1(b)) );
            return pLa + (pLb - pLa) * dp1(a) / xType(dp1(a) - dp1(b));
        }; // intersect line from plane 2 with plane 1


    xType x11;
    xType x12;
    xType x21;
    xType x22;

    if (edges1.line1.intersection == nullptr) {
        x11 = i1(*edges1.line1.from, *edges1.line1.to);
        BOOL_MESH_DEBUG_PRINTLN(O << " + " << x11 << " * "<<n3<<" = "<< (O + x11 * n3));
        edges1.line1.intersection = new HE_Vertex(O + x11 * n3 /n3.vSize(), -1);
        BOOL_MESH_DEBUG_PRINTLN(edges1.line1.intersection->p);
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x11 from given vertex ");
        x11 = divide(edges1.line1.intersection->p - O , n3);
    }
    if (edges1.line2.intersection == nullptr) {
        x12 = i1(*edges1.line2.from, *edges1.line2.to);
        BOOL_MESH_DEBUG_PRINTLN(O << " + " << x12 << " * "<<n3<<" = "<< (O + x12 * n3));
        edges1.line2.intersection = new HE_Vertex(O + x12 * n3 /n3.vSize(), -1);
        BOOL_MESH_DEBUG_PRINTLN(edges1.line2.intersection->p);
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x12 from given vertex ");
        x12 = divide(edges1.line2.intersection->p - O , n3);
    }
    if (edges2.line1.intersection == nullptr) {
        x21 = i2(*edges2.line1.from, *edges2.line1.to);
        edges2.line1.intersection = new HE_Vertex(O + x21 * n3 /n3.vSize(), -1);
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x21 from given vertex ");
        x21 = divide(edges2.line1.intersection->p - O , n3);
    }
    if (edges2.line2.intersection == nullptr) {
        x22 = i2(*edges2.line2.from, *edges2.line2.to);
        edges2.line2.intersection = new HE_Vertex(O + x22 * n3 /n3.vSize(), -1);
    } else {
        BOOL_MESH_DEBUG_PRINTLN("using intersection x22 from given vertex ");
        x22 = divide(edges2.line2.intersection->p - O , n3);
    }


//BOOL_MESH_DEBUG_PRINTLN(" ");
//BOOL_MESH_DEBUG_PRINTLN("x11 = " << x11);
//BOOL_MESH_DEBUG_PRINTLN("p11 = " << edges1.line1.intersection->p);
//BOOL_MESH_DEBUG_PRINTLN("x12 = " << x12);
//BOOL_MESH_DEBUG_PRINTLN("p12 = " << edges1.line2.intersection->p);
//BOOL_MESH_DEBUG_PRINTLN("x21 = " << x21);
//BOOL_MESH_DEBUG_PRINTLN("p21 = " << edges2.line1.intersection->p);
//BOOL_MESH_DEBUG_PRINTLN("x22 = " << x22);
//BOOL_MESH_DEBUG_PRINTLN("p22 = " << edges2.line2.intersection->p);


    if (x11 > x12)
    {
        BOOL_MESH_DEBUG_PRINTLN("first intersections swapped");
        std::swap(x11,x12);
        std::swap(edges1.line1, edges1.line2);
    }
    if (x21 > x22)
    {
        BOOL_MESH_DEBUG_PRINTLN("second intersections swapped");
        std::swap(x21,x22);
        std::swap(edges2.line1, edges2.line2);
    }

    if (x12 < x21 || x22 < x11)
    {
        BOOL_MESH_DEBUG_PRINTLN("no overlap!");
        return; // no overlap!
    }

    if (x11 > x21)
    {
        std::cerr << "from first = "<< edges1.line1.intersection->p <<std::endl;
    } else
    {
        std::cerr << "from second = "<< edges2.line1.intersection->p <<std::endl;
    }

    if (x12 < x22)
    {
        std::cerr << "to first = "<< edges1.line2.intersection->p <<std::endl;
    } else
    {
        std::cerr << "to second = "<< edges2.line2.intersection->p <<std::endl;
    }

        BOOL_MESH_DEBUG_PRINTLN("finished!");
}



xType BooleanFVMeshOps::divide(Point a, Point& b) //!< assumes the two vectors are in the same direction
{
    xType ret;
    if (b.x >= b.y && b.x >= b.z)
        ret = xType(a.x) / b.x;
    else if (b.y >= b.z)
        ret =  xType(a.y) / b.y;
    else
        ret = xType(a.z) / b.z;

    BOOL_MESH_DEBUG_PRINTLN("\n dividing (" << a<<","<<b <<") = "<< ret);
    return ret;
}












BooleanFVMeshOps::IntersectionEnv BooleanFVMeshOps::getIntersectingEdges(Point3& a, Point3& b, Point3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{
    BOOL_MESH_DEBUG_PRINTLN("getIntersectingEdges");

    IntersectionEnv ret;
    ret.computeIntersectingEdges(a,b,c,sa,sb,sc, fh);
    return ret;
}
void BooleanFVMeshOps::IntersectionEnv::computeIntersectingEdges(Point3& a, Point3& b, Point3& c, char sa, char sb, char sc, HE_FaceHandle fh)
{

    // check which two lines of triangle  cross the plane of triangle , and check if the plane is crossed in a vertex
    if (sa == sb)
    {
        if (sa==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ab below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sc == 0) return; // triangle doesn't cross the plane (only touches it)
            line1.from  = &a;
            line1.to    = &c;
            line2.from  = &c;
            line2.to    = &b;
        }

    }
    else if (sa == sc)
    {
        if (sa==0) // whole line lies on segment
        {
            std::cerr<< "use line segment ac below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sb == 0) return; // triangle doesn't cross the plane (only touches it)
            line1.from  = &a;
            line1.to    = &b;
            line2.from  = &c;
            line2.to    = &b;
        }

    }
    else if (sb == sc)
    {
        if (sb==0) // whole line lies on segment
        {
            std::cerr<< "use line segment bc below, instead of the computed line segment" << std::endl;
        }
        else
        {
            if (sa == 0) return; // triangle doesn't cross the plane (only touches it)
            line1.from  = &a;
            line1.to    = &b;
            line2.from  = &a;
            line2.to    = &c;
        }

    }
    else // sa =/= sb =/= sc =/= sa  : all une1ual, so one point must lie in the middle ON the plane
    {
        if (sa == 0)
        {
            line1.intersection = &fh.v0().vertex();
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v0().idx);
            O = &line1.intersection->p;
            line2.from = &c;
            line2.to   = &b;
        }
        else if (sb == 0)
        {
            line1.intersection = &fh.v1().vertex();
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v1().idx);
            O = &line1.intersection->p;
            line2.from  = &a;
            line2.to    = &c;
        }
        else if (sc == 0)
        {
            line1.intersection = &fh.v2().vertex();
            BOOL_MESH_DEBUG_PRINTLN("using intersection from given vertex " << fh.v2().idx);
            O = &line1.intersection->p;
            line2.from  = &a;
            line2.to    = &b;
        }
    }

}
