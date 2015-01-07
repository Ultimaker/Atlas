#ifndef MESHVERTEX_H
#define MESHVERTEX_H

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp> // is_base_of<BaseClass, DerivedClass>::value

#include "../settings.h"
#include <iostream> // ostream

#include "../Kernel.h"

#include "../BoundingBox.h"

/*!
Vertex type to be used in a Mesh.
*/
class MeshVertex
{
public:
    Point p; //!< location of the vertex
    MeshVertex(Point p) : p(p) {} //!< doesn't set connected_faces
};

template<typename Vertex, typename VertexHandle, typename Face, typename FaceHandle>
class Mesh;

template<typename V, typename F, typename FH, typename M>
struct MeshVertexHandle
{
    typedef MeshVertexHandle<V,F,FH,M> VH;
//    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertex, V>::value));
//    BOOST_STATIC_ASSERT((boost::is_base_of<Mesh<V,VH,F,FH>, M>::value));
    M& m;
    int idx;
    MeshVertexHandle(M& m_, int idx_) : m(m_), idx(idx_) {};

    V& vertex() { return m.vertices[idx]; };
    Point& p() { return m.vertices[idx].p; };

    bool operator==(const MeshVertexHandle& b) { return idx==b.idx; }; // TODO: more sophisticated check
    bool operator!=(const MeshVertexHandle &other) const {
        return !(*this == other);
    }
};

#endif//MESHVERTEX_H

