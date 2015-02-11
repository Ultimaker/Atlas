#include "HalfEdgeMeshEdge.h"
#include "../utils/logoutput.h"

#include "HalfEdgeMesh.h"
#include "HalfEdgeMeshVertex.h"
#include "HalfEdgeMeshFace.h"


#include "../MACROS.h" // debug
// enable/disable debug output
#define HE_MESH_DEBUG 0

#define HE_MESH_DEBUG_SHOW(x) DEBUG_SHOW(x)
#define HE_MESH_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) DEBUG_DO(x)
#else
#  define HE_MESH_DEBUG_DO(x)
#endif



#include <iostream>

HE_EdgeHandle& HE_EdgeHandle:: operator =(const HE_EdgeHandle& other)
{ m = other.m; idx= other.idx; };

HE_Edge& HE_EdgeHandle::edge() { return m->edges[idx]; };

HE_EdgeHandle HE_EdgeHandle::next()
{
    return HE_EdgeHandle(*m, m->edges[idx].next_edge_idx) ;
};

HE_EdgeHandle HE_EdgeHandle::converse()
{
    return HE_EdgeHandle(*m,  m->edges[idx].converse_edge_idx);
};

HE_VertexHandle HE_EdgeHandle::from_vert()
{
    return HE_VertexHandle(*m, m->edges[idx].from_vert_idx);
}
HE_VertexHandle HE_EdgeHandle::to_vert()
{
    return next().from_vert();
};
HE_VertexHandle HE_EdgeHandle::v0() { return from_vert(); }
HE_VertexHandle HE_EdgeHandle::v1() { return to_vert(); }


HE_FaceHandle HE_EdgeHandle::face()
{
    return HE_FaceHandle(*m, m->edges[idx].face_idx);
}

Point& HE_EdgeHandle::p0() { return from_vert().vertex().p; }
Point& HE_EdgeHandle::p1() { return to_vert().vertex().p; }

void HE_EdgeHandle::set(HE_EdgeHandle& b) { idx = b.idx; m=b.m; }
