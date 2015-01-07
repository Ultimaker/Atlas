#ifndef FVMESHFACE_H
#define FVMESHFACE_H


#include "Mesh.h"
#include "MeshFace.h"
#include "MeshVertex.h"

#include "FVMeshVertex.h"

/*! A FVMeshFace is a 3 dimensional model triangle with 3 points. These points are already converted to integers

A face has 3 connected faces, corresponding to its 3 edges.

Note that a correct model may have more than 2 faces connected via a single edge!
In such a case the face_index stored in connected_face_index is the one connected via the outside; see ASCII art below:

: horizontal slice through vertical edge connected to four faces :

\verbatim
[inside] x|
         x| <--+--- faces which contain each other in their connected_face_index fiels
   xxxxxxx|   \|/
   -------+-------
      ^   |xxxxxxx
      +-->|x
      |   |x [inside]
      |
    faces which contain each other in their connected_face_index fiels
\endverbatim
*/
class FVMeshFace : public MeshFace<FVMeshVertex>
{
public:
    int vertex_index[3] = {-1}; //!< counter-clockwise ordering
    int connected_face_index[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
};



struct FVMeshFaceHandle : public MeshFaceHandle<FVMeshVertex, FVMeshVertexHandle, FVMeshFace, FVMesh>
{
    FVMeshFaceHandle(FVMesh& m, int idx) : MeshFaceHandle(m, idx) {};

    FVMeshVertexHandle v0();
    FVMeshVertexHandle v1();
    FVMeshVertexHandle v2();
};



#endif//FVMESHFACE_H

