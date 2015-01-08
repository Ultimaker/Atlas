#ifndef HEPMESHVERTEX_H
#define HEPMESHVERTEX_H

#include "../Kernel.h"

#include "MeshVertex.h"

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/

class HEP_Edge;

class HEP_Vertex : public MeshVertex
{
    public:
        //std::vector<HEP_Edge> edges;
        HEP_Edge* someEdge;
        HEP_Vertex(Point3 p, HEP_Edge* someEdge)
        : MeshVertex(p)
        , someEdge(someEdge)
        {};


        std::string toString() { return "p"+std::to_string(p.x)+","+std::to_string(p.y)+","+std::to_string(p.z) + " - someEdge: "+std::to_string(long(someEdge)); };
};


class HEP_Mesh;
class HEP_Face;
class HEP_FaceHandle;
class HEP_EdgeHandle;

class FVMeshVertexHandle;

class HEP_VertexHandle : public MeshVertexHandle<HEP_Vertex, HEP_Face, HEP_FaceHandle, HEP_Mesh>
{
public:
    HEP_Vertex& v;
    HEP_VertexHandle(HEP_Mesh& m, HEP_Vertex* vertex) : MeshVertexHandle(m, -1), v(*vertex) {};

    HEP_Vertex& vertex() { return v; };

    HEP_EdgeHandle someEdge();

    Point p();

    bool operator==(const HEP_VertexHandle& b) const { return &v==&b.v; };
    bool operator!=(const HEP_VertexHandle &other) const {
        return !(*this == other);
    }

    bool isManifold(FVMeshVertexHandle& correspondingFVMeshVertex);

    void getConnectedEdgeGroups(FVMeshVertexHandle& correspondingFVMeshVertex, std::vector<std::vector<HEP_EdgeHandle>> & result);

    static void testGetConnectedEdgeGroups();

    /*!
    Split a nonmanifold vertex and move new vertices along some edge of the corresponding manifold part.
    */
    void splitWhenNonManifold(FVMeshVertexHandle& correspondingFVMeshVertex);

};


#endif // HEPMESHVERTEX_H
