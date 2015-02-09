#ifndef FVMESH_H
#define FVMESH_H

#include "../settings.h"
#include <iostream> // ostream

#include "../BoundingBox.h"

#include "Mesh.h"
#include "MeshFace.h"
#include "MeshVertex.h"

#include "FVMeshFace.h"
#include "FVMeshVertex.h"

/*!
An FVMesh is a basic representation of a 3D model. It contains all the faces as FVMeshFaces.

A face is represented by 3 verts, and a vert is represented by a point in 3D and it also knows all connected faces.
Moreover, a face knows to which faces it is connected.
This last property is computed when finish() is called.

An FVMesh is an implementation of the <a href="http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes">Face-vertex mesh datastructure</a>
, with the additional trait of a face which stores the 3 connected faces.

See FVMeshFace for the specifics of how/when faces are connected.
*/
class FVMesh : public Mesh<FVMeshVertex, FVMeshVertexHandle, FVMeshFace, FVMeshFaceHandle> // inherits from mesh interface
{
    //! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
    std::map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
public:
    typedef FVMeshVertex Vertex;
    typedef FVMeshFace Face;
    FVMesh(SettingsBase* parent); //!< initializes the settings

    void addFace(Point3& v0, Point3& v1, Point3& v2); //!< add a face to the mesh without settings it's connected_faces.
    void clear(); //!< clears all data
    void finish(); //!< complete the model : set the connected_face_index fields of the faces.

    void debugOuputBasicStats(std::ostream& out);


private:
    int findIndexOfVertex(Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.
    /*!
    Get the index of the face connected to the face with index \p notFaceIdx, via vertices \p idx0 and \p idx1.
    In case multiple faces connect with the same edge, return the next counter-clockwise face when viewing from \p idx1 to \p idx0.
    */
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx);
};


#endif//FVMESH_H

