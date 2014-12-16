#ifndef FVMESH_H
#define FVMESH_H

#include "settings.h"
#include <iostream> // ostream

#include "BoundingBox.h"

/*!
Vertex type to be used in a FVMesh.

Keeps track of which faces connect to it.
*/
class FVMeshVertex
{
public:
    Point3 p; //!< location of the vertex
    std::vector<uint32_t> connected_faces; //!< list of the indices of connected faces

    FVMeshVertex(Point3 p) : p(p) {} //!< doesn't set connected_faces
};

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
class FVMeshFace
{
public:
    int vertex_index[3] = {-1}; //!< counter-clockwise ordering
    int connected_face_index[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
};


/*!
An FVMesh is a basic representation of a 3D model. It contains all the faces as FVMeshFaces.

A face is represented by 3 verts, and a vert is represented by a point in 3D and it also knows all connected faces.
Moreover, a face knows to which faces it is connected.
This last property is computed when finish() is called.

An FVMesh is an implementation of the <a href="http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes">Face-vertex mesh datastructure</a>
, with the additional trait of a face which stores the 3 connected faces.

See FVMeshFace for the specifics of how/when faces are connected.
*/
class FVMesh : public SettingsBase // inherits settings
{
    //! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
    std::map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
public:
    std::vector<FVMeshVertex> vertices;//!< list of all vertices in the mesh
    std::vector<FVMeshFace> faces; //!< list of all faces in the mesh

    FVMesh(SettingsBase* parent); //!< initializes the settings

    void addFace(Point3& v0, Point3& v1, Point3& v2); //!< add a face to the mesh without settings it's connected_faces.
    void clear(); //!< clears all data
    void finish(); //!< complete the model : set the connected_face_index fields of the faces.

    Point3 min(); //!< min (in x,y and z) vertex of the bounding box
    Point3 max(); //!< max (in x,y and z) vertex of the bounding box

    void debugOuputBasicStats(std::ostream& out);

    BoundingBox computeFaceBbox(int f);

private:
    int findIndexOfVertex(Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.
    /*!
    Get the index of the face connected to the face with index \p notFaceIdx, via vertices \p idx0 and \p idx1.
    In case multiple faces connect with the same edge, return the next counter-clockwise face when viewing from \p idx1 to \p idx0.
    */
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx);
};


#endif//FVMESH_H

