#ifndef HEPMESHFACE_H
#define HEPMESHFACE_H

#include "../Kernel.h"

#include "MeshFace.h"
#include "HEPMeshVertex.h"

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/


//     /|\ |   _
//      |  |   [ front of face ]
//      | \|/  "
// the face is always on the left hand side of the half-edge
class HEP_Edge;

class HEP_Face : public MeshFace<HEP_Vertex>
{
    public:
        HEP_Edge* edge[3];


        HEP_Face() {};

        HEP_Face(HEP_Edge* e0, HEP_Edge* e1, HEP_Edge* e2) { edge[0] = e0; edge[1] = e1; edge[2] = e2; };

        std::string toString() {return "edges: "+std::to_string(long(edge[0]))+", "+std::to_string(long(edge[1]))+", "+std::to_string(long(edge[2])); };


//        inline double cosAngle()
//        {
//            Point3 normal = getNormal();
//            return double(normal.z) / double(normal.vSize()); // fabs
//        };
};



struct HEP_FaceHandle : public MeshFaceHandle<HEP_Vertex, HEP_VertexHandle, HEP_Face, HEP_Mesh>
{
    HEP_Face& f;
    HEP_Face& face() { return f; };
    HEP_FaceHandle(HEP_Mesh& m, HEP_Face* face) : MeshFaceHandle(m, -1), f(*face) {};

    HEP_VertexHandle v0() const ;
    HEP_VertexHandle v1() const ;
    HEP_VertexHandle v2() const ;

    HEP_EdgeHandle edge0() const ;
    HEP_EdgeHandle edge1() const ;
    HEP_EdgeHandle edge2() const ;

    bool operator==(const HEP_FaceHandle& b) const { return &f==&b.f; };
    bool operator!=(const HEP_FaceHandle &other) const {
        return !(*this == other);
    };

    HEP_EdgeHandle getEdgeFrom(HEP_VertexHandle& v) const ;
};

#endif // HEPMESHFACE_H
