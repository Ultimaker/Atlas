#ifndef HEPMESH_H
#define HEPMESH_H

#include "../Kernel.h"

#include "FVMesh.h"
#include "../BoundingBox.h"

#include "HalfEdgeMesh.h"

#include "HEPMesh.h"
#include "HEPMeshFace.h"
#include "HEPMeshEdge.h"
#include "HEPMeshVertex.h"

/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/


//     /|\ |   _
//      |  |   [ front of face ]
//      | \|/  "
// the face is always on the left hand side of the half-edge



/*!
Half-edge mesh - a more elaborate data structure to store a mesh.

Each edge is represented by (in general) two directed half-edges.
The next and prev edge are the other edges within the face.
The converse is the half-edge directed in the opposite direction, belonging to the face on the other side of the edge.

Note that a correct model may have more than 2 faces connected to a single edge!
The converse of a half-edge of such an edge belonging to face F is given by the half-edge belonging to face F2, connected to F via the outside; see ASCII art below:

: horizontal slice through vertical edge connected to four faces :

\verbatim
[outside]  |x
           |x <--+--- faces with half-edges being each others converse
           |x   \|/
           |xxxxxxx
   --------+-------
   xxxxxxxx|
      ^   x|
      +-->x|
      |   x|  [outside]
      |
    faces with half-edges being each others converse
\endverbatim

As such we should keep in mind that when starting from some half-edge connected to a vertex, we cannot be guaranteed to be able to traverse all connected edges just by using the operations getNext() and getConverse()!
Walking along the surface of a model means walking along the outside of the model (as opposed to the inside).

*/
class HEP_Mesh : public Mesh<HEP_Vertex, HEP_VertexHandle, HEP_Face, HEP_FaceHandle> // inherits from mesh interface
{
    public:
        typedef HEP_Vertex Vertex;
        typedef HEP_Edge Edge;
        typedef HEP_Face Face;

        //std::vector<HEP_Vertex> vertices;
        //std::vector<HEP_Face> faces;
        std::vector<HEP_Edge> edges;

        HEP_Mesh(FVMesh& mesh);
        HEP_Mesh(HE_Mesh& mesh);
        HEP_Mesh() : Mesh(nullptr) {};
        virtual ~HEP_Mesh();


        //Point3 getNormal(HEP_Face& face) const;
        Point3 getNormal(HEP_Face* f);// const;

        void connectEdgesPrevNext(HEP_Edge* prev, HEP_Edge* next);
        void connectEdgesConverse(HEP_Edge* e1, HEP_Edge* e2);

        HEP_Face* createFace(HEP_Vertex* v0, HEP_Vertex* v1, HEP_Vertex* v2);
//        int createFaceWithEdge(int e, int v);
        HEP_Vertex* createVertex(Point p);
        HEP_Edge* createConverse(HEP_Edge* e);

        BoundingBox computeBbox();

        void debugOuputBasicStats(std::ostream& out);

        void addFace(Point& p0, Point& p1, Point& p2) ; //!< add a face to the mesh without settings it's traits.
        void clear() ; //!< clears all data
        void finish() ; //!< complete the model : compute all traits.



        void debugOutputWholeMesh();
    protected:
    private:
        //int findIndexOfVertex(Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.

};


#endif // HEPMESH_H
