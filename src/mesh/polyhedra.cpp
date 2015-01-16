#include "polyhedra.h"


#define MM2INT(n) (int64_t((n) * 1000))



static inline uint32_t pointHash(PhPoint& p)
{
    return ((p.x() + MELD_DISTANCE/2) / MELD_DISTANCE) ^ (MM2INT((p.y() + MELD_DISTANCE/2).to_double() / MELD_DISTANCE) << 10) ^ (MM2INT(((p.z() + MELD_DISTANCE/2)).to_double() / MELD_DISTANCE) << 20);
}


bool PhPointUtils::testLength(PhPoint& p, int32_t len)
{
    if (p.x() > len || p.x() < -len)
        return false;
    if (p.y() > len || p.y() < -len)
        return false;
    if (p.z() > len || p.z() < -len)
        return false;
    return vSize2(p) <= len*len;
}
int64_t PhPointUtils::vSize2(PhPoint& p)
{
    return int64_t(p.x())*int64_t(p.x())+int64_t(p.y())*int64_t(p.y())+int64_t(p.z())*int64_t(p.z());
}



PhPoint PolyhedraUtils::min(Polyhedron& p)
{
    if (p.size_of_vertices() < 1)
        return PhPoint(0, 0, 0);

    PhPoint first = p.vertices_begin()->point();
    prim_data x = first.x();
    prim_data y = first.y();
    prim_data z = first.z();
    //for(unsigned int i=0; i<vertices.size(); i++)
    for(Vertex_iterator i = p.vertices_begin() ; i != p.vertices_end() ; i++)
    {
        x = std::min(x, i->point().x());
        y = std::min(y, i->point().y());
        z = std::min(z, i->point().z());
    }
    return PhPoint(x,y,z);
}

PhPoint PolyhedraUtils::max(Polyhedron& p)
{
    if (p.size_of_vertices() < 1)
        return PhPoint(0, 0, 0);

    PhPoint first = p.vertices_begin()->point();
    prim_data x = first.x();
    prim_data y = first.y();
    prim_data z = first.z();
    //for(unsigned int i=0; i<vertices.size(); i++)
    for(Vertex_iterator i = p.vertices_begin() ; i != p.vertices_end() ; i++)
    {
        x = std::max(x, i->point().x());
        y = std::max(y, i->point().y());
        z = std::max(z, i->point().z());
    }
    return PhPoint(x,y,z);
}



void PolyhedronLoader::addVertex(Point& p)
{
    PhPoint php(p.x, p.y, p.z);
    addVertex(php);
}
void PolyhedronLoader::addVertex(PhPoint& p)
{
    vertices.emplace_back(p);
}

void PolyhedronLoader::addFace(int v0_idx, int v1_idx, int v2_idx)
{
    faces.push_back(TempFace(v0_idx, v1_idx, v2_idx));
}




void PolyhedronLoader::addFace(PhPoint& v0, PhPoint& v1, PhPoint& v2)
{
    int i0 = findIndexOfVertex(v0);
    int i1 = findIndexOfVertex(v1);
    int i2 = findIndexOfVertex(v2);
    faces.push_back(TempFace(i0,i1,i2));
}


int PolyhedronLoader::findIndexOfVertex(PhPoint& v)
{
    uint32_t hash = pointHash(v);

    for(unsigned int idx = 0; idx < vertex_hash_map[hash].size(); idx++)
    {
        if ((vertices[vertex_hash_map[hash][idx]] - v).squared_length() < MELD_DISTANCE)
        {
            return vertex_hash_map[hash][idx];
        }
    }
    vertex_hash_map[hash].push_back(vertices.size());
    vertices.emplace_back(v);
    return vertices.size() - 1;
}


void Builder::operator()( HalfedgeDS& hds)
{
// Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> B( hds, true);
    B.begin_surface( loader.vertices.size(), loader.faces.size(), loader.faces.size()*3);
//    typedef typename HalfedgeDS::Vertex Vertex;
//    typedef typename Vertex::Point PhPoint;

    // vertices
    for (PhPoint& vert : loader.vertices)
    {
        //B.add_vertex( PhPoint( vert.x, vert.y, vert.z));
        B.add_vertex(vert);
    }


    // faces
    for (TempFace& f : loader.faces)
    {
        B.begin_facet();
        B.add_vertex_to_facet( f.vert_ids[0]);
        B.add_vertex_to_facet( f.vert_ids[1]);
        B.add_vertex_to_facet( f.vert_ids[2]);
        B.end_facet();
    }

    B.end_surface();
};



void PolyhedronLoader::construct(Polyhedron& ph)
{
    Builder builder(*this);
    ph.delegate(builder);
}

Polyhedron PolyhedronLoader::construct()
{
    Polyhedron ph;
    construct(ph);
    return ph;
}


//template<class MeshT, class VH, class FH>
//void MeshToPolyhedronConverter::convert(MeshT& mesh, Polyhedron& result)
//{
//    PolyhedronLoader pl;
//
//    for (int v = 0; v < mesh.vertices.size(); v++)
//    {
//        pl.addVertex(VH(mesh, v).p());
//    }
//
//    for (int f = 0; f < mesh.faces.size(); f++)
//    {
//        FH fh(mesh, f);
//        pl.addFace(fh.v0().idx, fh.v1().idx, fh.v2().idx);
//    }
//
//    pl.construct(result);
//}

//
//int main_polyhedra_loader()
//{
//    Polyhedron P;
//    Build_triangle<HalfedgeDS> triangle;
//    P.delegate( triangle);
//    CGAL_assertion( P.is_triangle( P.halfedges_begin()));
//    return 0;
//}

