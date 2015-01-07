#ifndef MESHFACE_H
#define MESHFACE_H

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp> // is_base_of<BaseClass, DerivedClass>::value

#include "../settings.h"
#include <iostream> // ostream

#include "../Kernel.h"

#include "../BoundingBox.h"

#include "MeshVertex.h"

template<typename Vertex>
class MeshFace
{
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertex, Vertex>::value));

};

template<typename V, typename VH, typename F, typename M>
struct MeshFaceHandle
{
    typedef MeshFaceHandle<V,VH,F,M> FH;
//    BOOST_STATIC_ASSERT((boost::is_base_of<MeshFace<V>, F>::value));
//    BOOST_STATIC_ASSERT((boost::is_base_of<Mesh<V,VH,F,FH>, M>::value));
    M& m;
    int idx;
    MeshFaceHandle(M& m_, int idx_) : m(m_), idx(idx_) {};

    F& face() { return m.faces[idx]; };

    virtual VH v0() =0;
    virtual VH v1() =0;
    virtual VH v2() =0;
    Point p0() { return v0().vertex().p; };
    Point p1() { return v1().vertex().p; };
    Point p2() { return v2().vertex().p; };

    bool operator==(const MeshFaceHandle& b) { return idx==b.idx; }; // TODO: more sophisticated check
    bool operator!=(const MeshFaceHandle &other) const {
        return !(*this == other);
    };
};


#endif//MESHFACE_H

