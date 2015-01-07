#include "HEPMeshFace.h"

#include "HEPMesh.h"
#include "HEPMeshEdge.h"
#include "HEPMeshVertex.h"

// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif


HEP_EdgeHandle HEP_FaceHandle::edge0()
{
    return HEP_EdgeHandle(m, f.edge[0]);
}

HEP_EdgeHandle HEP_FaceHandle::edge1()
{
    return HEP_EdgeHandle(m, f.edge[1]);
}

HEP_EdgeHandle HEP_FaceHandle::edge2()
{
    return HEP_EdgeHandle(m, f.edge[2]);
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
