#include "HEPMeshEdge.h"
#include "../utils/logoutput.h"

#include "HEPMesh.h"
#include "HEPMeshVertex.h"
#include "HEPMeshFace.h"

// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif



#include <iostream>

//HEP_Edge& HEP_EdgeHandle::edge() { return edge; };

//
//HEP_EdgeHandle& HEP_EdgeHandle:: operator =(const HEP_EdgeHandle& other)
//{
//      ! impossible due to the fact that the edge is a reference!
//    m = other.m;
//    idx= other.idx;
//    &e = &other.e;
//};

HEP_EdgeHandle HEP_EdgeHandle::next()
{
    return HEP_EdgeHandle(m, e.next_edge) ;
};

HEP_EdgeHandle HEP_EdgeHandle::converse()
{
    return HEP_EdgeHandle(m,  e.converse_edge);
};

HEP_VertexHandle HEP_EdgeHandle::from_vert()
{
    return HEP_VertexHandle(m, e.from_vert);
}
HEP_VertexHandle HEP_EdgeHandle::to_vert()
{
    return next().from_vert();
};

HEP_FaceHandle HEP_EdgeHandle::face()
{
    return HEP_FaceHandle(m, e.face);
}

Point HEP_EdgeHandle::p0() { return from_vert().vertex().p; }
Point HEP_EdgeHandle::p1() { return to_vert().vertex().p; }

void HEP_EdgeHandle::set(HEP_EdgeHandle& b) { idx = b.idx; m=b.m; e = b.e; }
