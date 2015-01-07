#include "HEPMeshVertex.h"

#include "HEPMesh.h"
#include "HEPMeshEdge.h"

// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif


HEP_EdgeHandle HEP_VertexHandle::someEdge()
{
    return HEP_EdgeHandle(m, v.someEdge);
}

Point HEP_VertexHandle::p() { return vertex().p; };
