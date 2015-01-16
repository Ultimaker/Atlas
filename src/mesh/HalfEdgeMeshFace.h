#ifndef HALFEDGEMESHFACE_H
#define HALFEDGEMESHFACE_H

#include "../Kernel.h"

#include <hash_fun.h> // hash function object

#include "MeshFace.h"
#include "HalfEdgeMeshVertex.h"

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/


//     /|\ |   _
//      |  |   [ front of face ]
//      | \|/  "
// the face is always on the left hand side of the half-edge

class HE_Face : public MeshFace<HE_Vertex>
{
    public:
        int edge_idx[3];


        std::string toString() {return "edges: "+std::to_string(edge_idx[0])+", "+std::to_string(edge_idx[1])+", "+std::to_string(edge_idx[2]); };
        HE_Face() {};

        HE_Face(int e0, int e1, int e2) { edge_idx[0] = e0; edge_idx[1] = e1; edge_idx[2] = e2; };



//        inline double cosAngle()
//        {
//            Point3 normal = getNormal();
//            return double(normal.z) / double(normal.vSize()); // fabs
//        };
};



struct HE_FaceHandle : public MeshFaceHandle<HE_Vertex, HE_VertexHandle, HE_Face, HE_Mesh>
{
    HE_FaceHandle(HE_Mesh& m, int idx) : MeshFaceHandle(m, idx) {};

    HE_VertexHandle v0();
    HE_VertexHandle v1();
    HE_VertexHandle v2();

    HE_EdgeHandle edge0();
    HE_EdgeHandle edge1();
    HE_EdgeHandle edge2();

    HE_EdgeHandle getEdgeFrom(HE_VertexHandle& v);

};

namespace std {
template <>
class hash<HE_FaceHandle> {
public:
    size_t operator()(const HE_FaceHandle & fh) const
    {
    return fh.idx;
    }
};
}

#endif // HALFEDGEMESHFACE_H
