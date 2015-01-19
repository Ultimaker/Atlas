#include "boolMesh.h"

#include <memory> // sharred_ptr

void BooleanFVMeshOps::subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result)
{
//! is more efficient when keep is smaller than subtracted.

/*
1) For every pair of triangles that intersect (of which the intersection hasn't already been computed):
    - Compute the intersection as line segment
    - Do a depth-first exhaustive search along the fracture line (mind that the fracture line may split when multiple faces connect to the same edge)
        Have two current-triangle-watchers for each mesh
            Convert new line segments into vertices and (connected) edges into a 'poly-edge'
        Map face of both meshes to a set of thus obtained poly-edges
            Add poly-edge to set of poly-edges of the face of each mesh

2) For each intersected face of both meshes:
    - Triangulate the part of the triangle we want to keep
    - Map original face to all resulting triangles
    - Add triangles to new mesh

3) Do 2 breadth-first exhaustive searches starting from the fracture, copying all triangles to the new mesh.
*/

    // : \/ construct an AABB tree from the keep-mesh
    typedef AABB_Tree<HE_FaceHandle>::Node Node;

    std::vector<Node*> leaves;
    for (int f = 0; f < keep.faces.size(); f++)
    {
        HE_FaceHandle fh(keep, f);
        leaves.push_back(new Node(keep.computeFaceBbox(f), &fh));
    }

    AABB_Tree<HE_FaceHandle> keep_aabb(leaves);

    for (int f = 0 ; f < subtracted.faces.size() ; f++)
    {
        BoundingBox tribbox = subtracted.computeFaceBbox(f);
        std::vector<HE_FaceHandle*> intersectingFaces;
        keep_aabb.getIntersections(tribbox, intersectingFaces);

        HE_FaceHandle face(subtracted, f);

    }

}





















void BooleanFVMeshOps::completeFractureLine(TriangleIntersection& first)
{

}





void BooleanFVMeshOps::getFacetIntersectionlineSegment(HE_FaceHandle& triangle, TriangleIntersection& first, FractureLineSegment& result)
{

    IntersectionPoint* first_intersectionPoint = first.from->clone();


    std::shared_ptr<TriangleIntersection> intersectionSegment = std::shared_ptr<TriangleIntersection> (&first);
    IntersectionPoint* intersectionPoint = first.to->clone();

    while (intersectionPoint->p() != first_intersectionPoint->p())
    {

        if      (intersectionPoint->p() == intersectionSegment->from->p())  intersectionPoint = intersectionSegment->to->clone();
        else if (intersectionPoint->p() == intersectionSegment->to->p())    intersectionPoint = intersectionSegment->from->clone();
        else BOOL_MESH_DEBUG_PRINTLN(" two consecutive intersection points are disconnected???!?! ");

        switch (intersectionPoint->getType())
        {
        case NEW: // intersection with edge
        {
            HE_FaceHandle nextFace = intersectionPoint->edge.converse().face();
            intersectionSegment = TriangleIntersectionComputation::intersect(triangle, nextFace, intersectionPoint->p());
        }
        break;
        case EXISTING: // intersection lies exactly on vertex

        break;
        }

    }



}





