#include "HEPMeshFace.h"

#include "HEPMesh.h"
#include "HEPMeshEdge.h"
#include "HEPMeshVertex.h"


#include "../MACROS.h" // debug
// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) DEBUG_DO(x)
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif


HEP_EdgeHandle HEP_FaceHandle::edge0()
{
    return HEP_EdgeHandle(*m, f.edge[0]);
}

HEP_EdgeHandle HEP_FaceHandle::edge1()
{
    return HEP_EdgeHandle(*m, f.edge[1]);
}

HEP_EdgeHandle HEP_FaceHandle::edge2()
{
    return HEP_EdgeHandle(*m, f.edge[2]);
}

HEP_VertexHandle HEP_FaceHandle::v0()
{
    return edge0().from_vert();
}

HEP_VertexHandle HEP_FaceHandle::v1()
{
    return edge1().from_vert();
}

HEP_VertexHandle HEP_FaceHandle::v2()
{
    return edge2().from_vert();
}
HEP_EdgeHandle HEP_FaceHandle::getEdgeFrom(HEP_VertexHandle& v)
{
    if (edge0().from_vert() == v) return edge0();
    if (edge1().from_vert() == v) return edge1();
    if (edge2().from_vert() == v) return edge2();
    HEP_MESH_DEBUG_PRINTLN("getEdgeFrom returning nullptr!!!");
    return HEP_EdgeHandle(*m, nullptr); // face is not connected to the vertex!!
}
