#ifndef HALFEDGEMESH_H
#define HALFEDGEMESH_H

#include "../Kernel.h"

#include "FVMesh.h"
#include "../BoundingBox.h"

#include "HalfEdgeMesh.h"
#include "HalfEdgeMeshFace.h"
#include "HalfEdgeMeshEdge.h"
#include "HalfEdgeMeshVertex.h"


class PrintObject;

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
The converse of a half-edge of such an edge belonging to face F is given by the half-edge belonging to face F2, connected to F via the inside; see ASCII art below:

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
class HE_Mesh : public Mesh<HE_Vertex, HE_VertexHandle, HE_Face, HE_FaceHandle> // inherits from mesh interface
{
    public:
        typedef HE_Vertex Vertex;
        typedef HE_Edge Edge;
        typedef HE_Face Face;
        //std::vector<HE_Vertex> vertices;
        //std::vector<HE_Face> faces;
        std::vector<HE_Edge> edges;

        HE_Mesh(FVMesh& mesh);
        HE_Mesh() : Mesh(nullptr) {};
        virtual ~HE_Mesh();


        HE_Edge* getNext(HE_Edge& edge)   ;
        //HE_Edge* getPrev(HE_Edge& edge)   ;
        HE_Edge* getConverse(HE_Edge& edge)   ;
        HE_Vertex* getTo(HE_Edge& edge)   ;
        HE_Vertex* getFrom(HE_Edge& edge)   ;

        HE_Edge* getSomeEdge(HE_Face& face)   ;
        HE_Edge* getSomeEdge(HE_Vertex& vertex)   ;

        HE_Face* getFace(HE_Edge& edge)   ;

        //Point3 getNormal(HE_Face& face) const;
        Point3 getNormal(int f_idx);// const;

        void connectEdgesPrevNext(int prev, int next);
        void connectEdgesConverse(int e1, int e2);

        int createFace(int v0_idx, int v1_idx, int v2_idx);
//        int createFaceWithEdge(int e_idx, int v_idx);
        int createVertex(Point p);
        int createConverse(int e_idx);

        BoundingBox computeBbox();


        void addFace(Point& p0, Point& p1, Point& p2) ; //!< add a face to the mesh without settings it's traits.
        void clear() ; //!< clears all data
        void finish() ; //!< complete the model : compute all traits.



        void checkModel(std::vector<ModelProblem>& result);


        void debugOuputBasicStats(std::ostream& out);
        void debugOutputWholeMesh();

        void makeManifold(FVMesh& correspondingFVMesh);

        static void testMakeManifold(PrintObject* model);

        HE_FaceHandle getFaceWithPoints(HE_VertexHandle v1, HE_VertexHandle v2, HE_FaceHandle notFace);

    protected:
    private:
        //int findIndexOfVertex(Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.

};


#endif // HALFEDGEMESH_H
