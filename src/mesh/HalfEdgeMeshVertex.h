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

struct HE_VertexHandle : public MeshVertexHandle<HE_Vertex, HE_Face, HE_FaceHandle, HE_Mesh>
{
    HE_VertexHandle(HE_Mesh& m, int idx) : MeshVertexHandle(m, idx) {};

    HE_Vertex& vertex();

    HE_EdgeHandle someEdge();

    Point p();

};


#endif // HALFEDGEMESHVERTEX_H
