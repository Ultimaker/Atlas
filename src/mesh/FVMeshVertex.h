#ifndef FVMESHVERTEX_H
#define FVMESHVERTEX_H


#include "Mesh.h"
#include "MeshFace.h"
#include "MeshVertex.h"



/*!
Vertex type to be used in a FVMesh.

Keeps track of which faces connect to it.
*/
class FVMeshVertex : public MeshVertex
{
public:
    std::vector<uint32_t> connected_faces; //!< list of the indices of connected faces

    FVMeshVertex(Point p) : MeshVertex(p) {} //!< doesn't set connected_faces
};


class FVMesh;
class FVMeshFace;
class FVMeshFaceHandle;

struct FVMeshVertexHandle : public MeshVertexHandle<FVMeshVertex, FVMeshFace, FVMeshFaceHandle, FVMesh>
{
    FVMeshVertexHandle(FVMesh& m, int idx) : MeshVertexHandle(m, idx) {};

    FVMeshVertex& vertex();
};

#endif//FVMESHVERTEX_H

