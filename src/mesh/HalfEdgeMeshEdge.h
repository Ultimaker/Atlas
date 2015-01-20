#ifndef HALFEDGEMESHEDGE_H
#define HALFEDGEMESHEDGE_H

#include "../Kernel.h"

class HE_Mesh;

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/


class HE_Edge {
    public:
        int from_vert_idx;

        int next_edge_idx;

        int converse_edge_idx;

        int face_idx;

        HE_Edge(int from_vert_idx, int face_idx)
        : from_vert_idx(from_vert_idx)
        //, to_vert_idx(to_vert_idx)
        , face_idx(face_idx)
        {};

        std::string toString() {
            return
             "from vertex "+ std::to_string(from_vert_idx) +
            //" to "+ std::to_string(to_vert_idx)
            // +" from edge " + std::to_string(prev_edge_idx)
            +" to edge "+std::to_string(next_edge_idx)+" of face"+std::to_string(face_idx)+"; converse of "+std::to_string(converse_edge_idx) ;
            };

};

class HE_VertexHandle;
class HE_FaceHandle;

struct HE_EdgeHandle
{
    HE_Mesh* m;
    int idx;
    HE_Edge& edge();

    HE_EdgeHandle(HE_Mesh& m_, int idx_) : m(&m_), idx(idx_) {};

    HE_VertexHandle from_vert();
    HE_VertexHandle to_vert();

    HE_EdgeHandle next();
    HE_EdgeHandle converse();

    Point p0();
    Point p1();

    HE_FaceHandle face();

    bool operator==(const HE_EdgeHandle& b) const { return idx==b.idx && m==b.m; }; // TODO: more sophisticated check
    bool operator!=(const HE_EdgeHandle &other) const {
        return !(*this == other);
    };

    void set(HE_EdgeHandle& b);
    HE_EdgeHandle& operator =(const HE_EdgeHandle& other);
};


#endif // HALFEDGEMESHEDGE_H
