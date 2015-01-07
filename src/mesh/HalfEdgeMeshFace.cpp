#include "HalfEdgeMeshFace.h"

#include "HalfEdgeMesh.h"
#include "HalfEdgeMeshEdge.h"
#include "HalfEdgeMeshVertex.h"

// enable/disable debug output
#define HE_MESH_DEBUG 0

#define HE_MESH_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#define HE_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HE_MESH_DEBUG_DO(x)
#endif


HE_EdgeHandle HE_FaceHandle::edge0()
{
    return HE_EdgeHandle(m, m.faces[idx].edge_idx[0]);
}

HE_EdgeHandle HE_FaceHandle::edge1()
{
    return HE_EdgeHandle(m, m.faces[idx].edge_idx[1]);
}

HE_EdgeHandle HE_FaceHandle::edge2()
{
    return HE_EdgeHandle(m, m.faces[idx].edge_idx[2]);
}

HE_VertexHandle HE_FaceHandle::v0()
{
    return edge0().from_vert();
}

HE_VertexHandle HE_FaceHandle::v1()
{
    return edge1().from_vert();
}

HE_VertexHandle HE_FaceHandle::v2()
{
    return edge2().from_vert();
}
