#include "polyhedra.h"


#define MM2INT(n) (int64_t((n) * 1000))



static inline uint32_t pointHash(Point& p)
{
    return ((p.x() + MELD_DISTANCE/2) / MELD_DISTANCE) ^ (((p.y() + MELD_DISTANCE/2) / MELD_DISTANCE) << 10) ^ (((p.z() + MELD_DISTANCE/2) / MELD_DISTANCE) << 20);
}


bool PointUtils::testLength(Point& p, int32_t len)
{
    if (p.x() > len || p.x() < -len)
        return false;
    if (p.y() > len || p.y() < -len)
        return false;
    if (p.z() > len || p.z() < -len)
        return false;
    return vSize2(p) <= len*len;
}
int64_t PointUtils::vSize2(Point& p)
{
    return int64_t(p.x())*int64_t(p.x())+int64_t(p.y())*int64_t(p.y())+int64_t(p.z())*int64_t(p.z());
}



Point PolyhedraUtils::min(Polyhedron& p)
{
    if (p.size_of_vertices() < 1)
        return Point(0, 0, 0);

    Point first = p.vertices_begin()->point();
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
    return Point(x,y,z);
}

Point PolyhedraUtils::max(Polyhedron& p)
{
    if (p.size_of_vertices() < 1)
        return Point(0, 0, 0);

    Point first = p.vertices_begin()->point();
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
    return Point(x,y,z);
}





void PolyhedronLoader::addFace(Point& v0, Point& v1, Point& v2)
{
    int i0 = findIndexOfVertex(v0);
    int i1 = findIndexOfVertex(v1);
    int i2 = findIndexOfVertex(v2);
    faces.push_back(TempFace(i0,i1,i2));
}


int PolyhedronLoader::findIndexOfVertex(Point& v)
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
//    typedef typename Vertex::Point Point;

    // vertices
    for (Point& vert : loader.vertices)
    {
        //B.add_vertex( Point( vert.x, vert.y, vert.z));
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

//
//int main_polyhedra_loader()
//{
//    Polyhedron P;
//    Build_triangle<HalfedgeDS> triangle;
//    P.delegate( triangle);
//    CGAL_assertion( P.is_triangle( P.halfedges_begin()));
//    return 0;
//}

