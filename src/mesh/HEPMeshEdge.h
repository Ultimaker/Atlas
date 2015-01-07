#ifndef HEPMESHEDGE_H
#define HEPMESHEDGE_H

#include "../Kernel.h"

class HEP_Mesh;

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/
class HEP_Vertex;
class HEP_Face;

class HEP_Edge {
    public:
        HEP_Vertex* from_vert;

        HEP_Edge* next_edge;

        HEP_Edge* converse_edge;

        HEP_Face* face;

        HEP_Edge(HEP_Vertex* from_vert, HEP_Face* face)
        : from_vert(from_vert)
        //, to_vert(to_vert)
        , face(face)
        {};


};

class HEP_VertexHandle;
class HEP_FaceHandle;

struct HEP_EdgeHandle
{
    HEP_Mesh& m;
    int idx;
    HEP_Edge& e;
    HEP_Edge& edge() { return e; };

    HEP_EdgeHandle(HEP_Mesh& m_, HEP_Edge* edge) : m(m_), idx(-1), e(*edge) {};

    HEP_VertexHandle from_vert();
    HEP_VertexHandle to_vert();

    HEP_EdgeHandle next();
    HEP_EdgeHandle converse();

    Point p0();
    Point p1();

    HEP_FaceHandle face();

    bool operator==(const HEP_EdgeHandle& b) const { return idx==b.idx; }; // TODO: more sophisticated check
    bool operator!=(const HEP_EdgeHandle &other) const {
        return !(*this == other);
    };

    void set(HEP_EdgeHandle& b);
};


#endif // HEPMESHEDGE_H
