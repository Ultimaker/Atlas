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


//        std::string toString() { return "p"+std::to_string(p.x)+","+std::to_string(p.y)+","+std::to_string(p.z) + " - someEdge: "+std::to_string(someEdge_idx); };
};


class HEP_Mesh;
class HEP_Face;
class HEP_FaceHandle;
class HEP_EdgeHandle;

struct HEP_VertexHandle : public MeshVertexHandle<HEP_Vertex, HEP_Face, HEP_FaceHandle, HEP_Mesh>
{
    HEP_Vertex& v;
    HEP_VertexHandle(HEP_Mesh& m, HEP_Vertex* vertex) : MeshVertexHandle(m, -1), v(*vertex) {};

    HEP_Vertex& vertex() { return v; };

    HEP_EdgeHandle someEdge();

    Point p();
};


#endif // HEPMESHVERTEX_H
