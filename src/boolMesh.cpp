#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

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





void BooleanFVMeshOps::getFacetIntersectionlineSegment(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLinePart& result)
{
    typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;
    typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;

    std::unordered_map<HE_VertexHandle, Node*> vertex2node; // to know when we have closed a cycle
    // vertices are the only places where the fracture can split

    std::unordered_set<HE_FaceHandle> checked_faces;

    IntersectionPoint first_intersectionPoint = *first->from;

    Node* first_node = result.fracture.addNode(first_intersectionPoint);

    Node* second_node = result.fracture.addNode(*first->to);
    Arrow* first_arrow = result.fracture.connect(*first_node, *second_node, IntersectionSegment(*first, triangle1));

    result.start = first_arrow;

    std::list<Arrow*> todo; // holds the prev intersectionSegment and the intersectionPoint which should be the first end of the new lineSegment
    todo.push_front(first_arrow);

    while (!todo.empty())
    {
        Arrow* current = todo.back();
        todo.pop_back();

        Node& connectingNode = *current->to;
        IntersectionPoint& connectingPoint = connectingNode.data;


        switch (connectingPoint.getType())
        {
        case NEW: // intersection with edge
        {
            HE_FaceHandle newFace = connectingPoint.edge.converse().face();
            if (checked_faces.find(newFace) == checked_faces.end())
            {
                checked_faces.insert(newFace);
                std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(triangle1, newFace, connectingPoint.p());
                addIntersectionToGraphAndTodo(connectingNode, *triangleIntersection, triangle1, newFace, vertex2node, result, todo);
            }
        }
        break;
        case VERTEX: // intersection lies exactly on vertex
            HE_EdgeHandle first_outEdge = connectingPoint.vh.someEdge();
            HE_FaceHandle prevFace = current->data.otherFace;
            HE_EdgeHandle outEdge = first_outEdge;
            do {
                if (outEdge.face() != prevFace)
                {
                    HE_FaceHandle newFace = outEdge.face();
                    if (checked_faces.find(newFace) == checked_faces.end())
                    {
                        checked_faces.insert(newFace);
                        std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(triangle1, newFace, connectingPoint.p());
                        if (triangleIntersection->intersectionType == LINE_SEGMENT)
                        {
                            addIntersectionToGraphAndTodo(connectingNode, *triangleIntersection, triangle1, newFace, vertex2node, result, todo);
                        }
                    }
                }

                outEdge = outEdge.converse().next();
            } while ( outEdge != first_outEdge);
        break;
        }

    }

/*
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
        case VERTEX: // intersection lies exactly on vertex
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

*/

}




void BooleanFVMeshOps::addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*> todo)
{
    IntersectionPoint& connectingPoint = connectingNode.data;

    Node* new_node;

    if      (connectingPoint.p() == triangleIntersection.to->p())  std::swap(triangleIntersection.from, triangleIntersection.to);
    else assert (connectingPoint.p() == triangleIntersection.from->p());


    bool new_point_is_already_done = false;
    if      (triangleIntersection.to->getType() == VERTEX)
    {
        std::unordered_map<HE_VertexHandle, Node*>::const_iterator node = vertex2node.find(triangleIntersection.to->vh);

        if (node == vertex2node.end()) // vertex is not yet present in graph
        {
            new_node = result.fracture.addNode(*triangleIntersection.to);
            vertex2node.insert( { {triangleIntersection.to->vh, new_node} } );

        } else
        {
            new_node = node->second;
            new_point_is_already_done = true;
            // dont push to todo! it is already there!
        }
    } else
    {
        new_node = result.fracture.addNode(*triangleIntersection.to);
    }


    Arrow* new_arrow = result.fracture.connect(connectingNode, *new_node, IntersectionSegment(triangleIntersection, newFace));
    if (!new_point_is_already_done)
    {
        // check whether we reached the end of the triangle
        switch (new_node->data.type)
        {
        case NEW:
            if (new_node->data.edge.face() == originalFace)
                result.endPoints.push_back(new_arrow);
            else
                todo.push_front(new_arrow);
        break;
        case VERTEX:
            if (originalFace.hasVertex(new_node->data.vh))
                result.endPoints.push_back(new_arrow);
            else
                todo.push_front(new_arrow);
        break;
        }
    }
}

