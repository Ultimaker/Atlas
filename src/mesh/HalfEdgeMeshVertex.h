#ifndef HALFEDGEMESHVERTEX_H
#define HALFEDGEMESHVERTEX_H

#include "../Kernel.h"

#include "MeshVertex.h"

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/

class HE_Vertex : public MeshVertex
{
    public:
        //std::vector<HE_Edge> edges;
        int someEdge_idx;
        HE_Vertex(Point3 p, int someEdge_idx)
        : MeshVertex(p)
        , someEdge_idx(someEdge_idx)
        {};


        std::string toString() { return "p"+std::to_string(p.x)+","+std::to_string(p.y)+","+std::to_string(p.z) + " - someEdge: "+std::to_string(someEdge_idx); };
};


class HE_Mesh;
class HE_Face;
class HE_FaceHandle;
class HE_EdgeHandle;
class FVMeshVertexHandle;

struct HE_VertexHandle : public MeshVertexHandle<HE_Vertex, HE_Face, HE_FaceHandle, HE_Mesh>
{
    HE_VertexHandle(HE_Mesh& m, int idx) : MeshVertexHandle(m, idx) {};

    HE_Vertex& vertex();

    HE_EdgeHandle someEdge();

    Point p();

    bool isManifold(FVMeshVertexHandle& correspondingFVMeshVertex);

    void getConnectedEdgeGroups(FVMeshVertexHandle& correspondingFVMeshVertex, std::vector<std::vector<HE_EdgeHandle>> & result);

    static void testGetConnectedEdgeGroups();
    /*!
    Split a nonmanifold vertex and move new vertices along some edge of the corresponding manifold part.
    */
    void splitWhenNonManifold(FVMeshVertexHandle& correspondingFVMeshVertex);

};


#endif // HALFEDGEMESHVERTEX_H
