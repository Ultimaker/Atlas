#ifndef POLYHEDRA_H
#define POLYHEDRA_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

typedef int prim_data;
typedef CGAL::Simple_cartesian<prim_data> Kernel;
typedef CGAL::Polyhedron_traits_with_normals_3<Kernel> Traits;
typedef CGAL::Polyhedron_3<Traits> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef HalfedgeDS::Vertex Vertex;
typedef HalfedgeDS::Vertex_iterator Vertex_iterator;
typedef Vertex::Point Point;
//typedef Point Point3;

class PointUtils
{
public:
    static bool testLength(Point& p, int32_t len);
    static int64_t vSize2(Point& p);
};

class PolyhedraUtils
{
public:
    static Point min(Polyhedron& p);
    static Point max(Polyhedron& p);
};

struct TempFace
{
    int vert_ids[3];
    TempFace(int x, int y, int z)
    {
        vert_ids[0] = x;
        vert_ids[1] = y;
        vert_ids[2] = z;
    };
};

class Builder;


class PolyhedronLoader
{
    friend class Builder;
    public:
        void addFace(Point& v0, Point& v1, Point& v2); //!< automatically inserts vertex if points don't correspond to any existing vertex

        void addVertex(Point&

        Polyhedron construct();
        void construct(Polyhedron& p);
        std::vector<Point> vertices;
        std::vector<TempFace> faces;
    protected:
        std::map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
        int findIndexOfVertex(Point& v);

    private:

};


class Builder : public CGAL::Modifier_base<HalfedgeDS>
{
public:
    Builder(PolyhedronLoader& loader) : loader(loader) {};
    void operator()( HalfedgeDS& hds);
protected:
    PolyhedronLoader loader;
};

#endif // POLYHEDRA_H
