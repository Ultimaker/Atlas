#ifndef TRIANGULATION3D_H
#define TRIANGULATION3D_H

#include <tuple>
#include <poly2tri.h>

#include "utils/PlaneEquation.h"


#include "MACROS.h" // debug
// enable/disable debug output
#define TRIANGULATION3D_DEBUG 1

#if TRIANGULATION3D_DEBUG == 1
#   define TRIANGULATION3D_DEBUG_DO(x) DEBUG_DO(x)
#   define TRIANGULATION3D_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define TRIANGULATION3D_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#else
#   define TRIANGULATION3D_DEBUG_DO(x)
#   define TRIANGULATION3D_DEBUG_SHOW(x)
#   define TRIANGULATION3D_DEBUG_PRINTLN(x)
#endif



struct P2T_CoordGetter
{
    static double x(p2t::Point a) { return a.x; };
    static double y(p2t::Point a) { return a.y; };
};

template<typename Pt3D>
class Triangulation3D
{
public:

    static void triangulate(Pt3D vector_dim_1, Pt3D vector_dim_2,   std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles);
    static void triangulate(Pt3D a, Pt3D bx, Pt3D by,               std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles);

    static void test();

private:
    void triangulate(std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles);

    PlaneEquation<Pt3D, p2t::Point, P2T_CoordGetter> basis;

    Triangulation3D(Pt3D vector_dim_1, Pt3D vector_dim_2) : basis(vector_dim_1, vector_dim_2) { }
    Triangulation3D(Pt3D a, Pt3D bx, Pt3D by) : basis(a, bx, by) { }

};



template<typename Pt3D>
void Triangulation3D<Pt3D>::triangulate(Pt3D vector_dim_1, Pt3D vector_dim_2, std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles)
{
    return Triangulation3D(vector_dim_1, vector_dim_2).triangulate(outline, holes, triangles);
}

template<typename Pt3D>
void Triangulation3D<Pt3D>::triangulate(Pt3D a, Pt3D bx, Pt3D by, std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles)
{
    return Triangulation3D(a, bx, by).triangulate(outline, holes, triangles);
}



template<typename Pt3D>
void Triangulation3D<Pt3D>::triangulate(std::vector<Pt3D>& outline, std::vector<std::vector<Pt3D>>& holes, std::vector<std::tuple<Pt3D, Pt3D, Pt3D>>& triangles)
{

    TRIANGULATION3D_DEBUG_SHOW(basis.x_axis);
    TRIANGULATION3D_DEBUG_SHOW(basis.y_axis);



    std::vector<p2t::Point*> polyline;
    for (Pt3D p : outline)
        polyline.push_back(new p2t::Point(basis.project(p)));

    p2t::CDT cdt(polyline);

    std::vector<std::vector<p2t::Point*>> holes_p2t;
    for (std::vector<Pt3D> hole : holes)
    {
        holes_p2t.emplace_back();
        std::vector<p2t::Point*>& hole_p2t = holes_p2t.back();
        for (Pt3D p : hole)
            hole_p2t.push_back(new p2t::Point(basis.project(p)));

        cdt.AddHole(hole_p2t);
    }


    cdt.Triangulate();


    //list<Triangle*> triangles = cdt->GetMap(); // cdt->GetTriangles();
    std::vector<p2t::Triangle*> triangles_p2t = cdt.GetTriangles();

    for (p2t::Triangle* tri : triangles_p2t)
        triangles.emplace_back(basis.project(*tri->GetPoint(0)), basis.project(*tri->GetPoint(1)), basis.project(*tri->GetPoint(2)));


    polyline.clear();
    for (std::vector<p2t::Point*>& hole_p2t : holes_p2t)
        holes_p2t.clear();
}



template<typename Pt3D>
void Triangulation3D<Pt3D>::test()
{
    std::vector<Pt3D> outline;
    std::vector<std::vector<Pt3D>> holes;
    std::vector<std::tuple<Pt3D, Pt3D, Pt3D>> triangles;

    outline.emplace_back(0,0 ,0);
    outline.emplace_back(0,10, 10);
    outline.emplace_back(10,10, 20);
    outline.emplace_back(10,0, 10);

    holes.emplace_back();
    std::vector<Pt3D>& hole = holes.back();

    hole.emplace_back(2,2, 4);
    hole.emplace_back(2,8, 10);
    hole.emplace_back(8,8, 16);
    hole.emplace_back(8,6, 14);
    hole.emplace_back(6,6, 12);
    hole.emplace_back(6,7, 13);
    hole.emplace_back(4,7, 11);
    hole.emplace_back(4,3, 7);
    hole.emplace_back(6,3, 9);
    hole.emplace_back(6,4, 10);
    hole.emplace_back(8,4, 12);
    hole.emplace_back(8,2, 10);


    triangulate(outline[1],outline[0], outline[2], outline, holes, triangles);

    for (std::tuple<Pt3D, Pt3D, Pt3D> tri : triangles)
    {
        Pt3D p1, p2, p3;
        std::tie(p1, p2, p3) = tri;
        std::cerr << p1 << std::endl;
        std::cerr << p2 << std::endl;
        std::cerr << p2 << std::endl;
        std::cerr << p3 << std::endl;
        std::cerr << p3 << std::endl;
        std::cerr << p1 << std::endl;
        std::cerr << std::endl;
    }
}


#endif // TRIANGULATION3D_H
