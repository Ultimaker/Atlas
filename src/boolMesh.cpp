#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <list> // == double-linked list, used as queue

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





















void BooleanFVMeshOps::completeFractureLine(std::shared_ptr<TriangleIntersection> first)
{

}





void BooleanFVMeshOps::getFacetIntersectionlineSegment(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLineSegment& result)
{
    typedef Graph<IntersectionPoint, TriangleIntersection>::Node Node;
    typedef Graph<IntersectionPoint, TriangleIntersection>::Arrow Arrow;

    IntersectionPoint first_intersectionPoint = *first->from;

    Node* first_node = result.fracture.addNode(first_intersectionPoint);

    Node* last_node = first_node;

    //std::list<Arrow> todo; // holds the prev intersectionSegment and the new intersectionPoint which should be one end of the new lineSegment

    std::shared_ptr<TriangleIntersection> intersectionSegment (first);
    IntersectionPoint intersectionPoint = *first->to;
    HE_FaceHandle lastFace = triangle2;

    while (intersectionPoint.p() != first_intersectionPoint.p())
    {
        Node* newNode = result.fracture.connectToNewNode(*last_node, intersectionPoint, *intersectionSegment);

        switch (intersectionPoint.getType())
        {
        case NEW: // intersection with edge
        {
            lastFace = intersectionPoint.edge.converse().face();
            intersectionSegment = TriangleIntersectionComputation::intersect(triangle1, lastFace, intersectionPoint.p());
        }
        break;
        case EXISTING: // intersection lies exactly on vertex
            HE_EdgeHandle first_outEdge = intersectionPoint.vh.someEdge();
            HE_EdgeHandle outEdge = first_outEdge;
            HE_FaceHandle prevFace = lastFace;
            do {
                if (outEdge.face() != prevFace)
                {
                    lastFace = outEdge.face();
                    intersectionSegment = TriangleIntersectionComputation::intersect(triangle1, lastFace, intersectionPoint.p());
                }

                outEdge = outEdge.converse().next();
            } while ( outEdge != first_outEdge);
        break;
        }

        assert(intersectionSegment->to);
        assert(intersectionSegment->from);

        {// update the intersectionPoint
            if      (intersectionPoint.p() == intersectionSegment->from->p())  intersectionPoint = *intersectionSegment->to;
            else if (intersectionPoint.p() == intersectionSegment->to->p())    intersectionPoint = *intersectionSegment->from;
            else BOOL_MESH_DEBUG_PRINTLN(" two consecutive intersection points are disconnected???!?! ");
        }
    }



}





