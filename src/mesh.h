#ifndef MESH_H
#define MESH_H

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp> // is_base_of<BaseClass, DerivedClass>::value

#include "settings.h"
#include <iostream> // ostream

#include "Kernel.h"

#include "BoundingBox.h"

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
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertex, V>::value));
    BOOST_STATIC_ASSERT((boost::is_base_of<Mesh<V,VH,F,FH>, M>::value));
    V* v;
    M* m;
};

template<typename Vertex>
class MeshFace
{
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertex, Vertex>::value));
public:
    virtual Vertex v0() =0;
    virtual Vertex v1() =0;
    virtual Vertex v2() =0;
    Point p0() { return v0().p; };
    Point p1() { return v1().p; };
    Point p2() { return v2().p; };
};

template<typename V, typename VH, typename F, typename M>
struct MeshFaceHandle
{
    typedef MeshFaceHandle<V,VH,F,M> FH;
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshFace<V>, F>::value));
    BOOST_STATIC_ASSERT((boost::is_base_of<Mesh<V,VH,F,FH>, M>::value));
    F* f;
    M* m;
};


/*!
A Mesh represents the most basic requirements of a 3D model.

It contains (a list of) all vertices and faces.

A vertex has a location, p.
A face can access its three vertices (indirectly).


*/

template<typename Vertex, typename VertexHandle, typename Face, typename FaceHandle>
class Mesh : public SettingsBase // inherits settings
{
    typedef Vertex V;
    typedef VertexHandle VH;
    typedef Face F;
    typedef FaceHandle FH;
    typedef Mesh<V,VH,F,FH> M;
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertex, Vertex>::value));
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshVertexHandle<V,F,FH,M>, VertexHandle>::value));
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshFace<Vertex>, Face>::value));
    BOOST_STATIC_ASSERT((boost::is_base_of<MeshFaceHandle<V,VH,F,M>, FaceHandle>::value));
public:
    std::vector<Vertex> vertices;//!< list of all vertices in the mesh
    std::vector<Face> faces; //!< list of all faces in the mesh

    Mesh(SettingsBase* parent); //!< initializes the settings

    virtual void addFace(Point& p0, Point& p1, Point& p2) =0; //!< add a face to the mesh without settings it's traits.
    virtual void clear() =0; //!< clears all data
    virtual void finish() =0; //!< complete the model : compute all traits.

    BoundingBox bbox;

    virtual BoundingBox computeFaceBbox(FaceHandle& fh) =0;

    virtual VertexHandle findVert(Point& p) =0;

};


#endif//MESH_H

