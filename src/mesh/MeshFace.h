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
    M* m;
    int idx;
    MeshFaceHandle(M& m_, int idx_) : m(&m_), idx(idx_) {};

    F& face() { return m->faces[idx]; };

    virtual VH v0() const =0;
    virtual VH v1() const =0;
    virtual VH v2() const =0;

    VH v(int i) const {
        switch(i)
        {
        case 0: return v0();
        case 1: return v1();
        case 2: return v2();
        }
        return v0();
    }

    Point& p0() const { return v0().vertex().p; };
    Point& p1() const { return v1().vertex().p; };
    Point& p2() const { return v2().vertex().p; };

    FPoint normal() { return FPoint( (p1-p0).cross(p2-p0) ).normalized(); };

    bool hasVertex(VH vh) const { return v0()==vh || v1()==vh || v2()==vh; };

    Point& p(int i)  const
    {
        switch(i)
        {
        case 0: return p0();
        case 1: return p1();
        case 2: return p2();
        }
        return p0();
    }

    virtual bool operator==(const MeshFaceHandle& b) const { return idx==b.idx && m==b.m; }; // TODO: more sophisticated check
    virtual bool operator!=(const MeshFaceHandle &other) const {
        return !(*this == other);
    };
};


#endif//MESHFACE_H

