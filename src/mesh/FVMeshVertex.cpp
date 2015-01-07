#include "FVMesh.h"

FVMeshVertex& FVMeshVertexHandle::vertex()
{
    return m.vertices[idx];
};
