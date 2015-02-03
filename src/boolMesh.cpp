#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

#include "modelFile/modelFile.h" // PrintObject

#include <stdio.h>

#include "settings.h" // MAX_EDGES_PER_VERTEX

#include "MACROS.h" // debug

#include <fstream> // write to file (debug)

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





















void BooleanMeshOps::completeFractureLine(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, std::unordered_map<HE_FaceHandle, FractureLinePart>& face2fracture)
{

    std::list<FractureLinePart> todo;

    todo.emplace_front();

    FractureLinePart& first_frac = todo.back();
    getFacetFractureLinePart(triangle1, triangle2, first, first_frac);


    face2fracture.insert(std::pair<HE_FaceHandle, FractureLinePart>(triangle1, first_frac));

    for (Arrow* arrow : first_frac.endPoints)
    {
        switch(arrow->to->data.getType())
        {
        case IntersectionPointType::NEW:
            BOOL_MESH_DEBUG_PRINTLN("NEW");
        break;
        case IntersectionPointType::VERTEX:
            BOOL_MESH_DEBUG_PRINTLN("VERTEX");

        break;
        }
    }
}

spaceType inaccuracy = 20; //!< the margin of error between the endpoint of the first triangle and the startpoint of the second



void BooleanMeshOps::getFacetFractureLinePart(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, std::shared_ptr<TriangleIntersection> first, FractureLinePart& result)
{
    BOOL_MESH_DEBUG_PRINTLN(" starting getFacetFractureLinePart");
    BOOL_MESH_DEBUG_PRINTLN("\n\n\n\n\n\n\n\n");


    typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;
    typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;

    std::unordered_map<HE_VertexHandle, Node*> vertex2node; // to know when we have closed a cycle
    // vertices are the only places where the fracture can split

    std::unordered_set<HE_FaceHandle> checked_faces;

    BOOL_MESH_DEBUG_PRINTLN((first->intersectionType));

    Node* first_node = result.fracture.addNode(*first->from);
    Node* second_node = result.fracture.addNode(*first->to);
    Arrow* first_arrow = result.fracture.connect(*first_node, *second_node, IntersectionSegment(*first, triangle1));

    result.start = first_arrow;

    checked_faces.insert(triangle2);



    std::list<Arrow*> todo; // holds the prev intersectionSegment and the intersectionPoint which should be the first end of the new lineSegment
    todo.push_front(first_arrow);

    BOOL_MESH_DEBUG_PRINTLN("\n ================== ");
    BOOL_MESH_DEBUG_PRINTLN(" starting main loop ");
    BOOL_MESH_DEBUG_PRINTLN(" ================== ");

    long counter = 0;
    while (!todo.empty())
    {
        counter++;
        if (counter == MAX_EDGES_PER_VERTEX)
        {
            std::cerr << "triangle seems to intersect with over " << counter << " triangles in the other mesh! infinite loop?" << std::endl;
            break;
        }
        Arrow* current = todo.back();
        todo.pop_back();

        Node& connectingNode = *current->to;
        IntersectionPoint& connectingPoint = connectingNode.data;


        switch (connectingPoint.getType())
        {
        case IntersectionPointType::NEW: // intersection with edge
        {
            HE_FaceHandle newFace = connectingPoint.edge.converse().face();
            BOOL_MESH_DEBUG_PRINTLN(" NEW case for face " << newFace.idx);
            if (checked_faces.find(newFace) == checked_faces.end())
            {
                checked_faces.insert(newFace);
                std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(triangle1, newFace);//, connectingPoint.p());
                if (! triangleIntersection->from || ! triangleIntersection->to)
                { // problem!
                    //BOOL_MESH_DEBUG_DO(
                        {
                            BOOL_MESH_DEBUG_PRINTLN("ERROR! : no intersection!!! type = " << (triangleIntersection->intersectionType));
                            BOOL_MESH_DEBUG_PRINTLN("saving mesh with problem to problem.stl");
                            Point3 normal = (newFace.p1() - newFace.p0()).cross(newFace.p2() - newFace.p0());
                            while (normal.testLength(10000))
                                normal*=2;
                            while (!normal.testLength(20000))
                                normal/=2;
                            HE_Mesh& heMesh = *newFace.m;
                            int vi = heMesh.vertices.size();
                            int ei = heMesh.edges.size();
                            int fi = heMesh.faces.size();
                            heMesh.vertices.emplace_back(connectingPoint.getLocation(), ei+0);
                            heMesh.vertices.emplace_back(connectingPoint.getLocation() + normal + Point3(1000,1000,1000), ei+1);
                            heMesh.vertices.emplace_back(connectingPoint.getLocation() + normal + Point3(-1000,-1000,-1000), ei+2);
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

                            HE_FaceHandle oldFace = connectingPoint.edge.face();
                            heMesh.faces[oldFace.idx].edge_idx[0] = 0;
                            heMesh.faces[oldFace.idx].edge_idx[1] = 0;
                            heMesh.faces[oldFace.idx].edge_idx[2] = 0;

                            saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "bs/problem.stl");
                            BOOL_MESH_DEBUG_PRINTLN("saving done..");
                            exit(0);
                        }
                    //);
                }
                addIntersectionToGraphAndTodo(connectingNode, *triangleIntersection, triangle1, newFace, vertex2node, result, todo);
            } else
            {
//                BOOL_MESH_DEBUG_PRINTLN("face " << newFace.idx <<" checked already!");
//                BOOL_MESH_DEBUG_PRINTLN("face : " << current->data.lineSegment.to->edge.converse().face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev face (from): " << current->data.lineSegment.from->edge.face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev face (to): " << current->data.lineSegment.to->edge.face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev prev face : " << current->data.lineSegment.from->edge.converse().face().idx);
            }
        }
        break;
        case IntersectionPointType::VERTEX: // intersection lies exactly on vertex
            BOOL_MESH_DEBUG_PRINTLN(" VERTEX case ");
            HE_EdgeHandle first_outEdge = connectingPoint.vertex.someEdge();
            HE_FaceHandle prevFace = current->data.otherFace;
            HE_EdgeHandle outEdge = first_outEdge;

            int edge_counter = 0;
            do {
                edge_counter++;
                if (edge_counter > MAX_EDGES_PER_VERTEX)
                {
                    std::cerr << "Vertex seems to be connected to over 1000 edges, breaking edges-around-vertex-iteration!" << std::endl;
                    break;
                }

                if (outEdge.face() != prevFace)
                {
                    HE_FaceHandle newFace = outEdge.face();
                    if (checked_faces.find(newFace) == checked_faces.end())
                    {
                        checked_faces.insert(newFace);
                        std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(triangle1, newFace);// , connectingPoint.p());
                        if (triangleIntersection->intersectionType == IntersectionType::LINE_SEGMENT)
                        {
                            assert(triangleIntersection->from);
                            assert(triangleIntersection->to);
                            if ( triangleIntersection->from->type == IntersectionPointType::VERTEX
                                && triangleIntersection->to->type == IntersectionPointType::VERTEX
                                && triangleIntersection->from->vertex != triangleIntersection->to->vertex)
                            {
                                checked_faces.insert(newFace.m->getFaceWithPoints(triangleIntersection->from->vertex, triangleIntersection->to->vertex, newFace));
                            }
                            addIntersectionToGraphAndTodo(connectingNode, *triangleIntersection, triangle1, newFace, vertex2node, result, todo);
                        }
                    }
                }

                outEdge = outEdge.converse().next();
            } while ( outEdge != first_outEdge);
        break;
        }
    }

    DEBUG_DO(
        std::ofstream csv;
        csv.open("bs/BS.csv", std::fstream::out | std::fstream::app);
//        for (Node* node : result.fracture.nodes)
//        {
//            csv << node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z << std::endl;
//        }
        for (Arrow* a : result.fracture.arrows)
        {
            Point p0 = a->data.lineSegment.from->p();
            csv << p0.x <<", " << p0.y << ", " << p0.z << std::endl;
            Point p1 = a->data.lineSegment.to->p();
            csv << p1.x <<", " << p1.y << ", " << p1.z << std::endl;
        }
        csv.close();
    );

}




void BooleanMeshOps::addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace, HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*>& todo)
{
    IntersectionPoint& connectingPoint = connectingNode.data;


    Node* new_node;

    if ( triangleIntersection.to->compareSourceConverse(connectingPoint) )
    {
        BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" =~= "<< triangleIntersection.to->p());
        BOOL_MESH_DEBUG_PRINTLN("swapping...");
        std::swap(triangleIntersection.from, triangleIntersection.to);
        std::swap(triangleIntersection.isDirectionOfInnerPartOfTriangle1, triangleIntersection.isDirectionOfInnerPartOfTriangle2);
    }
    if (! triangleIntersection.from->compareSourceConverse(connectingPoint)  )
    {
        BOOL_MESH_DEBUG_PRINTLN("warning! consequent intersection segments do not connect at source mesh!");
        if ( (triangleIntersection.to->p() - connectingPoint.p()).vSize2() < (triangleIntersection.from->p() - connectingPoint.p()).vSize2() )
        {
            BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" =~= "<< triangleIntersection.to->p());
            BOOL_MESH_DEBUG_PRINTLN("swapping...");
            std::swap(triangleIntersection.from, triangleIntersection.to);
            std::swap(triangleIntersection.isDirectionOfInnerPartOfTriangle1, triangleIntersection.isDirectionOfInnerPartOfTriangle2);
        }
    }
    if ( ! (connectingPoint.p() - triangleIntersection.from->p()).testLength(inaccuracy)  )
    {
        BOOL_MESH_DEBUG_PRINTLN("warning! large difference between endpoint of first triangle intersection and start point of second! :");
        BOOL_MESH_DEBUG_SHOW((connectingPoint.p() - triangleIntersection.from->p()).vSize());
        //BOOL_MESH_DEBUG_DO(
        {
            Point3 normal = (newFace.p1() - newFace.p0()).cross(newFace.p2() - newFace.p0());
            while (normal.testLength(10000))
            normal*=2;
            while (!normal.testLength(20000))
            normal/=2;
            HE_Mesh& heMesh = *newFace.m;
            int vi = heMesh.vertices.size();
            int ei = heMesh.edges.size();
            int fi = heMesh.faces.size();
            heMesh.vertices.emplace_back(triangleIntersection.from->p(), ei+0);
            heMesh.vertices.emplace_back(triangleIntersection.from->p() + normal, ei+1);
            heMesh.vertices.emplace_back(triangleIntersection.to->p() + Point3(-1000,-1000,-1000), ei+2);
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


             vi = heMesh.vertices.size();
             ei = heMesh.edges.size();
             fi = heMesh.faces.size();
            heMesh.vertices.emplace_back(connectingPoint.p(), ei+0);
            heMesh.vertices.emplace_back(connectingPoint.p() + normal + Point3(1000,1000,1000), ei+1);
            heMesh.vertices.emplace_back(connectingPoint.p() + normal + Point3(-1000,-1000,-1000), ei+2);
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


             vi = heMesh.vertices.size();
             ei = heMesh.edges.size();
             fi = heMesh.faces.size();
            heMesh.vertices.emplace_back(originalFace.p0(), ei+0);
            heMesh.vertices.emplace_back(originalFace.p1(), ei+1);
            heMesh.vertices.emplace_back(originalFace.p2(), ei+2);
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

             vi = heMesh.vertices.size();
             ei = heMesh.edges.size();
             fi = heMesh.faces.size();
            heMesh.vertices.emplace_back(newFace.p0(), ei+0);
            heMesh.vertices.emplace_back(newFace.p1(), ei+1);
            heMesh.vertices.emplace_back(newFace.p2(), ei+2);
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

            saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "large_difference.stl");
        }
        //);
    }


    bool new_point_is_already_done = false;
    if      (triangleIntersection.to->getType() == IntersectionPointType::VERTEX)
    {
        std::unordered_map<HE_VertexHandle, Node*>::const_iterator node = vertex2node.find(triangleIntersection.to->vertex);

        if (node == vertex2node.end()) // vertex is not yet present in graph
        {
            new_node = result.fracture.addNode(*triangleIntersection.to);
            vertex2node.insert( { {triangleIntersection.to->vertex, new_node} } );

        } else
        {
            new_node = node->second;
            new_point_is_already_done = true;
            // dont push to todo! it is already there!
            BOOL_MESH_DEBUG_PRINTLN("vertex is already in graph!");
        }
    } else
    {
        if ( triangleIntersection.to->compareSourceConverse( result.start->from->data ) )
        {
            BOOL_MESH_DEBUG_PRINTLN("connecting to starting node...");
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
        case IntersectionPointType::NEW:
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
        case IntersectionPointType::VERTEX:
            if (originalFace.hasVertex(new_node->data.vertex))
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
















void BooleanMeshOps::test_getFacetFractureLinePart(PrintObject* model)
{
    std::remove("BS.csv");
    std::cerr << "=============================================\n" << std::endl;

    FVMesh& mesh = model->meshes[0];

    DEBUG_HERE;

    HE_Mesh heMesh(mesh);
    DEBUG_HERE;

    heMesh.makeManifold(mesh);

    DEBUG_HERE;

    std::vector<ModelProblem> problems;
    heMesh.checkModel(problems);
    if (problems.size() > 0)
    {
        std::cout << "Model problems detected! :" << std::endl;
        for (ModelProblem problem : problems)
        {
            std::cout << problem.msg << std::endl;
        }
        std::exit(0);
    } else std::cout << " no problems! :D" << std::endl;




    HE_Mesh other;
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(2*mesh.bbox.size().x,0,1000), 0);
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,2*mesh.bbox.size().y,-1000), 1);
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-2*mesh.bbox.size().y,-1000), 2);
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(2*mesh.bbox.size().x,5000,0), 0);
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-5000,2*mesh.bbox.size().z), 1);
    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-5000,-2*mesh.bbox.size().z), 2);
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
        //BOOL_MESH_DEBUG_PRINTLN((triangleIntersection->intersectionType));
        if (triangleIntersection->intersectionType == IntersectionType::LINE_SEGMENT) break;
    }
    {
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.min);
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.max);
        BOOL_MESH_DEBUG_SHOW(mesh.bbox.mid());


        for (int i = 0; i < mesh.vertices.size(); i++)
            BOOL_MESH_DEBUG_SHOW(mesh.vertices[i].p);

        BOOL_MESH_DEBUG_SHOW(" \n==============\n");

        //heMesh.debugOutputWholeMesh();

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
        saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "bs/crappa.stl");
        BOOL_MESH_DEBUG_SHOW(other.vertices[0].p);
        BOOL_MESH_DEBUG_SHOW(other.vertices[1].p);
        BOOL_MESH_DEBUG_SHOW(other.vertices[2].p);
        if (f == heMesh.faces.size())
        {
            BOOL_MESH_DEBUG_PRINTLN("mesh doesn't intersect the face defined to go through its middle!");
            exit(0);
        }
    }

    //heMesh.debugOutputWholeMesh();


    HE_FaceHandle intersectingFace(heMesh, f);


    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << "=== start getFacetFractureLinePart ===\n" << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    std::unordered_map<HE_FaceHandle, FractureLinePart> face2fracture;
    completeFractureLine(otherFace, intersectingFace, triangleIntersection, face2fracture);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");

//    std::ofstream csv("BS.csv");
//    for (Node* node : result.fracture.nodes)
//    {
//        csv << node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z << std::endl;
//    }
//    csv.close();
}

void BooleanMeshOps::test_completeFractureLine(PrintObject* model)
{
    std::remove("BS.csv");
    std::cerr << "=============================================\n" << std::endl;

    FVMesh& mesh = model->meshes[0];

    DEBUG_HERE;

    HE_Mesh heMesh(mesh);
    DEBUG_HERE;

    heMesh.makeManifold(mesh);

    DEBUG_HERE;

    std::vector<ModelProblem> problems;
    heMesh.checkModel(problems);
    if (problems.size() > 0)
    {
        std::cout << "Model problems detected! :" << std::endl;
        for (ModelProblem problem : problems)
        {
            std::cout << problem.msg << std::endl;
        }
        std::exit(0);
    } else std::cout << " no problems! :D" << std::endl;




    HE_Mesh other(heMesh);
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(2*mesh.bbox.size().x,0,1000), 0);
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,2*mesh.bbox.size().y,-1000), 1);
//    other.vertices.emplace_back(mesh.bbox.mid() + Point3(-2*mesh.bbox.size().x,-2*mesh.bbox.size().y,-1000), 2);
    for (HE_Vertex& v : other.vertices)
        v.p += mesh.bbox.size()*3/10;

    other.bbox = other.computeBbox();


    std::ofstream out("bs/test_completeFractureLine.stl");

    out << "solid name" << std::endl;

    {
        auto getPoint = [&heMesh](int f, int v) { return HE_FaceHandle(heMesh, f).p(v); } ;

        Point3 vert;
        for (int f = 0; f < heMesh.faces.size() ; f++)
        {

            out << "facet normal 0 0 0" << std::endl;
            out << "    outer loop" << std::endl;
            vert = getPoint(f,0);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            vert = getPoint(f,1);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            vert = getPoint(f,2);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            out << "    endloop" << std::endl;
            out << "endfacet" << std::endl;

        }
    }

    out << "\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
    {
        auto getPoint = [&other](int f, int v) { return HE_FaceHandle(other, f).p(v); } ;

        Point3 vert;

        for (int f = 0; f < other.faces.size() ; f++)
        {

            out << "facet normal 0 0 0" << std::endl;
            out << "    outer loop" << std::endl;
            vert = getPoint(f,0);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            vert = getPoint(f,1);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            vert = getPoint(f,2);
            out << "        vertex " <<vert.x <<" "<<vert.y<<" "<<vert.z  << std::endl;
            out << "    endloop" << std::endl;
            out << "endfacet" << std::endl;

        }
    }
    out << "endsolid name" << std::endl;
    out.close();



    HE_FaceHandle otherFace(other, 0);

    int f;
    std::shared_ptr<TriangleIntersection> triangleIntersection;
    for (int f0 = 0; f0 < other.faces.size(); f0++)
    {
        otherFace.idx = f0;
        for (f = 0; f < heMesh.faces.size(); f++)
        {
            HE_FaceHandle face(heMesh, f);
            triangleIntersection = TriangleIntersectionComputation::intersect(face, otherFace);
            //BOOL_MESH_DEBUG_PRINTLN((triangleIntersection->intersectionType));
            if (triangleIntersection->intersectionType == IntersectionType::LINE_SEGMENT) break;
        }
        if (triangleIntersection->intersectionType == IntersectionType::LINE_SEGMENT) break;
    }
    if (f == heMesh.faces.size())
    {
        BOOL_MESH_DEBUG_PRINTLN("no intersection found with first triangle of other mesh!");
        exit(0);
    }

    //heMesh.debugOutputWholeMesh();


    HE_FaceHandle intersectingFace(heMesh, f);


    std::cerr << triangleIntersection->intersectionType << std::endl;

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << "=== start completeFractureLine ===\n" << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    std::unordered_map<HE_FaceHandle, FractureLinePart> face2fracture;
    completeFractureLine(otherFace, intersectingFace, triangleIntersection, face2fracture);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");


}
