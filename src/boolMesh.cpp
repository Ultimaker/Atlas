#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

#include "modelFile/modelFile.h" // PrintObject

void BooleanMeshOps::subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result)
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





















void BooleanMeshOps::completeFractureLine(std::shared_ptr<TriangleIntersection> first)
{

}





void BooleanMeshOps::getFacetIntersectionlineSegment(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLinePart& result)
{
    BOOL_MESH_DEBUG_PRINTLN(" starting getFacetIntersectionlineSegment");
    BOOL_MESH_DEBUG_PRINTLN("\n\n\n\n\n\n\n\n");

    DEBUG_HERE;

    typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;
    typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;
    DEBUG_HERE;

    std::unordered_map<HE_VertexHandle, Node*> vertex2node; // to know when we have closed a cycle
    // vertices are the only places where the fracture can split

    std::unordered_set<HE_FaceHandle> checked_faces;

    BOOL_MESH_DEBUG_PRINTLN(toString(first->intersectionType));

    Node* first_node = result.fracture.addNode(*first->from);
    DEBUG_HERE;
    Node* second_node = result.fracture.addNode(*first->to);
    DEBUG_HERE;
    Arrow* first_arrow = result.fracture.connect(*first_node, *second_node, IntersectionSegment(*first, triangle1));
    DEBUG_HERE;

    result.start = first_arrow;

    DEBUG_HERE;
    result.debugOutput();
    DEBUG_HERE;

    checked_faces.insert(triangle2);



    std::list<Arrow*> todo; // holds the prev intersectionSegment and the intersectionPoint which should be the first end of the new lineSegment
    todo.push_front(first_arrow);

    BOOL_MESH_DEBUG_PRINTLN("\n ================== ");
    BOOL_MESH_DEBUG_PRINTLN(" starting main loop ");
    BOOL_MESH_DEBUG_PRINTLN(" ================== ");

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
            BOOL_MESH_DEBUG_PRINTLN(" NEW case for face " << newFace.idx);
            if (checked_faces.find(newFace) == checked_faces.end())
            {
                checked_faces.insert(newFace);
                std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(triangle1, newFace, connectingPoint.p());
                addIntersectionToGraphAndTodo(connectingNode, *triangleIntersection, triangle1, newFace, vertex2node, result, todo);
            } else BOOL_MESH_DEBUG_PRINTLN("face " << newFace.idx <<" checked already!");
        }
        break;
        case VERTEX: // intersection lies exactly on vertex
            BOOL_MESH_DEBUG_PRINTLN(" VERTEX case ");
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
        BOOL_MESH_DEBUG_PRINTLN("------");
        result.debugOutput();
        BOOL_MESH_DEBUG_PRINTLN("------");
    }
}




void BooleanMeshOps::addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*>& todo)
{
    IntersectionPoint& connectingPoint = connectingNode.data;

    Node* new_node;

    if      (connectingPoint.p() == triangleIntersection.to->p())   // TODO: more libral check
    {
        BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" == "<< triangleIntersection.to->p());
        BOOL_MESH_DEBUG_PRINTLN("swapping...");
        std::swap(triangleIntersection.from, triangleIntersection.to);
    }
    else
    {
        BOOL_MESH_DEBUG_PRINTLN("asserting :");
        BOOL_MESH_DEBUG_SHOW((connectingPoint.p() == triangleIntersection.from->p()));
        assert (connectingPoint.p() == triangleIntersection.from->p()); // TODO: more libral check
    }


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
            BOOL_MESH_DEBUG_PRINTLN("vertex is already in graph!");
        }
    } else
    {
        if (triangleIntersection.to->getLocation() == result.start->from->data.getLocation()) // TODO: more libral check
        {
            new_node = result.start->from;
        } else
        {
            BOOL_MESH_DEBUG_PRINTLN("normal case: adding node...");
            new_node = result.fracture.addNode(*triangleIntersection.to);
        }
    }

    // make the new arrow
    Arrow* new_arrow = result.fracture.connect(connectingNode, *new_node, IntersectionSegment(triangleIntersection, newFace));


    if (!new_point_is_already_done)
    {
        // check whether we reached the end of the triangle
        switch (new_node->data.type)
        {
        case NEW:
            if (new_node->data.edge.face() == originalFace)
            {
                BOOL_MESH_DEBUG_PRINTLN("exiting triangle");
                result.endPoints.push_back(new_arrow);
            }
            else
            {
                BOOL_MESH_DEBUG_PRINTLN("normal case, adding new arrow to todo");
                todo.push_front(new_arrow);
            }
        break;
        case VERTEX:
            if (originalFace.hasVertex(new_node->data.vh))
            {
                BOOL_MESH_DEBUG_PRINTLN("exiting triangle");
                result.endPoints.push_back(new_arrow);
            }
            else
                todo.push_front(new_arrow);
        break;
        }
    }

}


void BooleanMeshOps::test_getFacetIntersectionlineSegment(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    FVMesh& mesh = model->meshes[0];


    HE_Mesh heMesh(mesh);
    heMesh.makeManifold(mesh);

    HE_Mesh other;
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(2*mesh.bbox.size().x,0,0), 0);
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,2*mesh.bbox.size().y,0), 1);
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-2*mesh.bbox.size().y,0), 2);
    other.edges.emplace_back(0,1);
    other.edges.emplace_back(1,2);
    other.edges.emplace_back(2,0);
    other.connectEdgesPrevNext(0,1);
    other.connectEdgesPrevNext(1,2);
    other.connectEdgesPrevNext(2,0);
    other.faces.emplace_back(0,1,2);
    other.edges[0].face_idx = 0;
    other.edges[1].face_idx = 0;
    other.edges[2].face_idx = 0;
    HE_FaceHandle otherFace(other, 0);

    int f;
    std::shared_ptr<TriangleIntersection> triangleIntersection;
    for (f = 0; f < heMesh.faces.size(); f++)
    {
        HE_FaceHandle face(heMesh, f);
        triangleIntersection = TriangleIntersectionComputation::intersect(face, otherFace);
        BOOL_MESH_DEBUG_PRINTLN(toString(triangleIntersection->intersectionType));
        if (triangleIntersection->intersectionType == LINE_SEGMENT) break;
    }
    if (f == heMesh.faces.size())
    {
        BOOL_MESH_DEBUG_PRINTLN("mesh doesn't intersect the face defined to go through its middle!");
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.min);
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.max);
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.mid());


        for (int i = 0; i < mesh.vertices.size(); i++)
            BOOL_MESH_DEBUG_SHOW(mesh.vertices[i].p);

        BOOL_MESH_DEBUG_SHOW(" \n==============\n");

        heMesh.debugOutputWholeMesh();

        int vi = heMesh.vertices.size();
        int ei = heMesh.edges.size();
        int fi = heMesh.faces.size();
        heMesh.vertices.emplace_back(mesh.bbox.mid() + Point3(2*mesh.bbox.size().x,0,0), ei+0);
        heMesh.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,2*mesh.bbox.size().y,0), ei+1);
        heMesh.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-2*mesh.bbox.size().y,0), ei+2);
        heMesh.edges.emplace_back(vi+0,vi+1);
        heMesh.edges.emplace_back(vi+1,vi+2);
        heMesh.edges.emplace_back(vi+2,vi+0);
        heMesh.connectEdgesPrevNext(ei+0,ei+1);
        heMesh.connectEdgesPrevNext(ei+1,ei+2);
        heMesh.connectEdgesPrevNext(ei+2,ei+0);
        heMesh.faces.emplace_back(ei+0,ei+1,ei+2);
        heMesh.edges[ei+0].face_idx = fi;
        heMesh.edges[ei+1].face_idx = fi;
        heMesh.edges[ei+2].face_idx = fi;
        saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "crappa.stl");
        BOOL_MESH_DEBUG_PRINTLN(other.vertices[0].p);
        BOOL_MESH_DEBUG_PRINTLN(other.vertices[1].p);
        BOOL_MESH_DEBUG_PRINTLN(other.vertices[2].p);
        exit(0);
    }

    heMesh.debugOutputWholeMesh();


    HE_FaceHandle intersectingFace(heMesh, f);


    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << "=== start getFacetIntersectionlineSegment ===\n" << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    FractureLinePart result;
    getFacetIntersectionlineSegment(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    result.debugOutput();
}
