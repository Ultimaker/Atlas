#include "FVMeshFace.h"

#include "FVMesh.h"
#include "FVMeshVertex.h"


FVMeshVertexHandle FVMeshFaceHandle::v0()
{
    return FVMeshVertexHandle(m, m.faces[idx].vertex_index[0]);
}

FVMeshVertexHandle FVMeshFaceHandle::v1()
{
    return FVMeshVertexHandle(m, m.faces[idx].vertex_index[1]);
}

FVMeshVertexHandle FVMeshFaceHandle::v2()
{
    return FVMeshVertexHandle(m, m.faces[idx].vertex_index[2]);
}
