#ifndef MESH_H
#define MESH_H

#include <type_traits> // is_base_of<BaseClass, DerivedClass>::value
//#include <boost/static_assert.hpp>

#include "../settings.h"
#include <iostream> // ostream

#include "../Kernel.h"

#include "../BoundingBox.h"

#include "MeshFace.h"
#include "MeshVertex.h"


enum class ModelProblemType { WHOLES, OVERLAPPING_EDGES, DEGENERATE_FACES, UNKNOWN };
struct ModelProblem
{
    ModelProblemType type;
    // VertexHandle* problematic_vertex;
    // ... etc.
    std::string msg;
    ModelProblem(const std::string& msg_) : type(ModelProblemType::UNKNOWN), msg(msg_) {};
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
    static_assert(std::is_base_of<MeshVertex, Vertex>::value, "Cannot instantiate a Mesh with those types!");
//    static_assert(std::is_base_of<MeshVertexHandle<V,F,FH,M>, VertexHandle>::value, "msg");
    static_assert(std::is_base_of<MeshFace<Vertex>, Face>::value, "Cannot instantiate a Mesh with those types!");
//    static_assert(std::is_base_of<MeshFaceHandle<V,VH,F,M>, FaceHandle>::value, "msg");
public:
    std::vector<Vertex> vertices;//!< list of all vertices in the mesh
    std::vector<Face> faces; //!< list of all faces in the mesh

    Mesh(SettingsBase* parent) //!< initializes the settings
    : SettingsBase(parent)
    { };

    virtual void addFace(Point& p0, Point& p1, Point& p2) =0; //!< add a face to the mesh without settings it's traits.
    virtual void clear() =0; //!< clears all data
    virtual void finish() =0; //!< complete the model : compute all traits.

    BoundingBox bbox;

    virtual BoundingBox computeFaceBbox(FaceHandle& fh)
    {
        Point& p0 = fh.p0();
        Point& p1 = fh.p1();
        Point& p2 = fh.p2();
        return BoundingBox(p0, p1) + p2;
    };

    //virtual VertexHandle findVert(Point& p) =0;

};


#endif//MESH_H

