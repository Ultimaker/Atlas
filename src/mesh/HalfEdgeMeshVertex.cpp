#include "HalfEdgeMeshVertex.h"

#include "HalfEdgeMesh.h"
#include "HalfEdgeMeshEdge.h"

// enable/disable debug output
#define HE_MESH_DEBUG 0

#define HE_MESH_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#define HE_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HE_MESH_DEBUG_DO(x)
#endif


HE_EdgeHandle HE_VertexHandle::someEdge()
{
    return HE_EdgeHandle(m, m.vertices[idx].someEdge_idx);
}

Point HE_VertexHandle::p() { return vertex().p; };

HE_Vertex& HE_VertexHandle::vertex()
{
    return m.vertices[idx];
};

