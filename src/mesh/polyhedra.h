#ifndef POLYHEDRA_H
#define POLYHEDRA_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include "../Kernel.h"

typedef int32_t prim_data;
typedef CGAL::Simple_cartesian<prim_data> Kernel;
typedef CGAL::Polyhedron_traits_with_normals_3<Kernel> Traits;
typedef CGAL::Polyhedron_3<Traits> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef HalfedgeDS::Vertex PhVertex;
typedef HalfedgeDS::Vertex_iterator Vertex_iterator;
typedef PhVertex::Point PhPoint;
//typedef Point Point3;

class PhPointUtils
{
public:
    static bool testLength(PhPoint& p, int32_t len);
    static int64_t vSize2(PhPoint& p);
};

class PolyhedraUtils
{
public:
    static PhPoint min(Polyhedron& p);
    static PhPoint max(Polyhedron& p);
};

struct TempFace
{
    spaceType vert_ids[3];
    TempFace(spaceType x, spaceType y, spaceType z)
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
        void addFace(PhPoint& v0, PhPoint& v1, PhPoint& v2); //!< automatically inserts vertex if points don't correspond to any existing vertex

        void addVertex(Point& p);
        void addVertex(PhPoint& p);

        void addFace(int v0_idx, int v1_idx, int v2_idx);

        Polyhedron construct();
        void construct(Polyhedron& p);
        std::vector<PhPoint> vertices;
        std::vector<TempFace> faces;
    protected:
        std::map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
        int findIndexOfVertex(PhPoint& v);

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




class MeshToPolyhedronConverter
{
public:
    template<class MeshT, class VH, class FH>
    static void convert(MeshT& mesh, Polyhedron& result)
    {
        PolyhedronLoader pl;

        for (int v = 0; v < mesh.vertices.size(); v++)
        {
            Point p = (VH(mesh, v).p());
            pl.addVertex(p);
        }

        for (int f = 0; f < mesh.faces.size(); f++)
        {
            FH fh(mesh, f);
            pl.addFace(fh.v0().idx, fh.v1().idx, fh.v2().idx);
        }

        pl.construct(result);
    }



};

#endif // POLYHEDRA_H
