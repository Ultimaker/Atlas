#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

#include "modelFile/modelFile.h" // PrintObject

#include <stdio.h>

#include "settings.h" // MAX_EDGES_PER_VERTEX

#include "MACROS.h" // debug

#include <fstream> // write to file (debug)

#include <boost/iterator/transform_iterator.hpp>

long errorCounter = 0;
long nonErrorCounter = 0;

void getFaceEdgeSegments(bool aboveIntersection, std::vector<FractureLinePart>& fracturesOnFace, std::vector<std::pair<IntersectionPoint, IntersectionPoint>>& segments) //!< get the segments belonging to the face
{
    if (fracturesOnFace.size() == 0) return;


    HE_FaceHandle face = fracturesOnFace[0].face;
    #if BOOL_MESH_DEBUG == 1
    for (FractureLinePart p : fracturesOnFace)
    {
        if (p.face != face)
        {
            BOOL_MESH_DEBUG_PRINTLN("ERROR! getFaceEdgeSegments called for different faces!");
        }

        for (Arrow* a : p.fracture.arrows)
        {
            Arrow* prev = a->from->last_in;
            if (prev != nullptr)
            {
                auto direction = [](Arrow* prev) { return (prev->data.otherFace_is_second_triangle)?
                                prev->data.lineSegment.isDirectionOfInnerPartOfTriangle1
                                : prev->data.lineSegment.isDirectionOfInnerPartOfTriangle2; };

//                if (direction(a) != direction(prev))
                if (a->data.lineSegment.isDirectionOfInnerPartOfTriangle2 != prev->data.lineSegment.isDirectionOfInnerPartOfTriangle2)
                {
                    BOOL_MESH_DEBUG_PRINTLN("ERROR! subsequent intersections not directed the same way!");
                    BOOL_MESH_DEBUG_SHOW(prev->data.lineSegment.isDirectionOfInnerPartOfTriangle2);
                    BOOL_MESH_DEBUG_SHOW(a->data.lineSegment.isDirectionOfInnerPartOfTriangle2);
                    errorCounter++;
                } else nonErrorCounter++;
            }
        }
    }
    #endif
}

void debug_csv(std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> & face2fractures, std::string filename = "WHOLE.csv")
{
    DEBUG_DO(
        std::ofstream csv;
        csv.open("CRAP.csv", std::fstream::out | std::fstream::app);
        csv << "0,0,0" << std::endl;
        csv.close();
    );

    std::ofstream csv;
    csv.open(filename, std::fstream::out | std::fstream::app);

    const Point offset(1, 1, 1);

    Point offset_now(0,0,0);
    Point p(0,0,0);
    for (auto mapping: face2fractures)
    {
        for (FractureLinePart frac : mapping.second)
        {
            for (auto arrow : frac.fracture.arrows)
            {
                p = arrow->to->data.p() + offset_now;
                csv << p.x <<", " << p.y << ", " << p.z << std::endl;
                p = arrow->from->data.p() + offset_now;
                csv << p.x <<", " << p.y << ", " << p.z << std::endl;
            }

            bool aboveIntersection = true; // TODO
            std::vector<std::pair<IntersectionPoint, IntersectionPoint>> segments;
            getFaceEdgeSegments(aboveIntersection, mapping.second, segments);

            offset_now += offset;
        }
    }
    csv << p.x <<", " << p.y << ", " << p.z << std::endl;
    csv << "5000,0,30000" << std::endl;
    csv.close();
    BOOL_MESH_DEBUG_SHOW(errorCounter);
    BOOL_MESH_DEBUG_SHOW(nonErrorCounter);
};


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



    std::vector<HE_FaceHandle> keep_faces;
    for (int f = 0; f < keep.faces.size(); f++)
    {
        keep_faces.emplace_back(keep, f);
    }

BOOL_MESH_DEBUG_PRINTLN("constructing AABB-tree...");
    AABB_Tree<HE_FaceHandle> keep_aabb(keep_faces.begin(), keep_faces.end());
BOOL_MESH_DEBUG_PRINTLN("finished constructing AABB-tree");



    typedef std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle> > Face2Faces;
    Face2Faces face2intersectingFaces;

    long totalTriTriIntersectionComputations = 0;
    long totalTriTriIntersections = 0;

                std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fractures;

    for (int f = 0 ; f < subtracted.faces.size() ; f++)
    {
        HE_FaceHandle face(subtracted, f);

        BoundingBox tribbox = face.bbox();
        std::vector<HE_FaceHandle> intersectingBboxFaces;
        keep_aabb.getIntersections(tribbox, intersectingBboxFaces);
        std::vector<std::pair<BoundingBox, HE_FaceHandle>> intersectingBboxFaces2;
        keep_aabb.getIntersections(tribbox, intersectingBboxFaces2);

        totalTriTriIntersectionComputations += intersectingBboxFaces.size();

        for (const HE_FaceHandle intersectingBboxFace : intersectingBboxFaces)
        {

            HE_FaceHandle tri1 = HE_FaceHandle(intersectingBboxFace);
            HE_FaceHandle& tri2 = face;

            std::shared_ptr<TriangleIntersection> triangleIntersection = TriangleIntersectionComputation::intersect(tri1, tri2);

            if (!triangleIntersection->from || !triangleIntersection->to)
                continue; // they don't intersect!
            else
            {
                totalTriTriIntersections++;
                Face2Faces::const_iterator result;
                if ( (result = face2intersectingFaces.find(tri1)) != face2intersectingFaces.end()
                && result->second.find(tri2) != result->second.end())
                {
BOOL_MESH_DEBUG_PRINTLN("face intersection already performed!");
                    continue;
                }
                if ( (result = face2intersectingFaces.find(tri2)) != face2intersectingFaces.end()
                && result->second.find(tri1) != result->second.end())
                {
BOOL_MESH_DEBUG_PRINTLN("face intersection already contained in some fracture line part!");
                    continue;
                }


//BOOL_MESH_DEBUG_PRINTLN("subtract > walking along face");
//                std::unordered_map<HE_FaceHandle, FractureLinePart> face2fracture_here;
//                typedef std::unordered_map<HE_FaceHandle, FractureLinePart>::iterator val_it;
//                completeFractureLine(tri1, face, *triangleIntersection, face2fracture_here);






                completeFractureLine(tri1, face, *triangleIntersection, face2fractures);

//                for (auto mapping : face2fracture_here)
//                    if (!(std::pair<val_it, bool> result = face2fracture.emplace(mapping.first, mapping.second)).second)
//                        result.






                for (std::pair<HE_FaceHandle, std::vector<FractureLinePart>> face_intersectingFaces : face2fractures)
                {
                    std::unordered_set<HE_FaceHandle> intersectingFaces;
                    for (FractureLinePart& frac : face_intersectingFaces.second)
                        for (Arrow* arrow : frac.fracture.arrows)
                            intersectingFaces.insert(arrow->data.otherFace);

                    std::pair<Face2Faces::iterator, bool> result = face2intersectingFaces.emplace(face_intersectingFaces.first, intersectingFaces);
                    if (!result.second) // if key was already present
                        result.first->second.insert(intersectingFaces.begin(), intersectingFaces.end()); // insert all faces in set
                }
//
//                for (auto face_faces : face2intersectingFaces)
//                {
//                    std::cerr << face_faces.first << std::endl;
//                    for (auto face : face_faces.second)
//                        std::cerr << face << ", " ;
//                    std::cerr << std::endl;
//                }
//                exit(0);
            }

        }
   }

// verification
std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fractures_keep;
std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fractures_subtracted;

for (std::pair<HE_FaceHandle, std::vector<FractureLinePart>> face_intersectingFaces : face2fractures)
{
    if (face_intersectingFaces.first.m == &keep)
    {
        face2fractures_keep.insert(face_intersectingFaces);
    } else if (face_intersectingFaces.first.m == &subtracted)
    {
        face2fractures_subtracted.insert(face_intersectingFaces);
    } else
    {
        DEBUG_PRINTLN("face belongs to no mesh!");
        exit(0);

    }
}


debug_csv(face2fractures_keep, "WHOLE_keep.csv");
debug_csv(face2fractures_subtracted, "WHOLE_subtracted.csv");


                debug_csv(face2fractures, "WHOLE.csv");

BOOL_MESH_DEBUG_SHOW(keep.faces.size());
BOOL_MESH_DEBUG_SHOW(totalTriTriIntersectionComputations);
BOOL_MESH_DEBUG_SHOW(totalTriTriIntersections);
BOOL_MESH_DEBUG_PRINTLN("average # intersection computations per triangle: "<< float(totalTriTriIntersectionComputations) / subtracted.faces.size());
BOOL_MESH_DEBUG_PRINTLN("\t( brute force would be: "<<keep.faces.size() * subtracted.faces.size() << ")");
BOOL_MESH_DEBUG_PRINTLN("average # intersections per triangle: "<< float(totalTriTriIntersections) / subtracted.faces.size());

}





















/*!
Make a complete a fracture line by walking along an intersection between two meshes.
Start from an intersection segment [first] between faces [triangle1] and [triangle2].

*/
void BooleanMeshOps::completeFractureLine(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, TriangleIntersection& first, std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>>& face2fractures)
{
    BOOL_MESH_DEBUG_PRINTLN("completeFractureLine");
    BOOL_MESH_DEBUG_SHOW(triangle1.m << ","<<triangle1.idx);
    BOOL_MESH_DEBUG_SHOW(triangle2.m << ","<<triangle2.idx);

    //std::vector<FractureLinePart> fractureLines;
    std::list<FractureLinePart*> todo;

    std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle> > walkedFacesFromEndpoints; // the faces for which a fracture line has been created and which faces of the other mesh are the starting and end_points
    // TODO /\ get as parameter ?


    bool throwAwayFirstFracLinePart = true; // TODO: choose and throw away rest of code!
    // true cause it can start somewhere in the middle of the fractureline
    if (throwAwayFirstFracLinePart)
    {
        FractureLinePart* first_frac;
        first_frac = new FractureLinePart(triangle1); // TODO: delete!
        getFacetFractureLinePart(triangle1, triangle2, first, *first_frac);


//        std::unordered_set<HE_FaceHandle> endPointFaces;
//        first_frac->endPointsFacesWithoutFirst(endPointFaces);
//        walkedFacesFromEndpoints.insert(std::pair<HE_FaceHandle, std::unordered_set<HE_FaceHandle> >(triangle1, endPointFaces) );

        // dont add to face2fracture

        todo.push_front(first_frac);
        if (first_frac->endPoints.size()==0)
        {
            std::vector<FractureLinePart> fracs;
            fracs.emplace_back(*first_frac);
            face2fractures.emplace(triangle1, fracs);
            BOOL_MESH_DEBUG_PRINTLN("First fracture line part covers whole fracture!");
            return;
        }
//    DEBUG_DO(
//        std::ofstream csv;
//        csv.open("CRAP.csv", std::fstream::out | std::fstream::app);
//        csv << "50000,50000,60000" << std::endl;
//        csv.close();
//    );
    } else
    {

//        fractureLines.emplace_back(triangle1);
//        FractureLinePart& first_frac = fractureLines.back();

        typedef std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>>::iterator val_it;
        std::vector<FractureLinePart> fracs;
        fracs.emplace_back(triangle1);
        std::pair<val_it, bool> new_mapping = face2fractures.emplace(triangle1, fracs);
        if (!new_mapping.second)
            new_mapping.first->second.emplace_back(triangle1);

        FractureLinePart& first_frac = new_mapping.first->second.back();

        getFacetFractureLinePart(triangle1, triangle2, first, first_frac);

        std::unordered_set<HE_FaceHandle> endPointFaces;
        first_frac.endPointsFaces(endPointFaces);
        walkedFacesFromEndpoints.insert(std::pair<HE_FaceHandle, std::unordered_set<HE_FaceHandle> >(triangle1, endPointFaces) );


        todo.push_front(&first_frac);
    }


    while (!todo.empty())
    {
        FractureLinePart& current_frac = *todo.back();
        todo.pop_back();

        HE_FaceHandle start_face = current_frac.face;    // from where to start walking over the fracture in the next face
        for (Arrow* arrow : current_frac.endPoints)
        {
            HE_FaceHandle next_face = arrow->data.otherFace; // face to walk

            BOOL_MESH_DEBUG_SHOW(next_face.m << ","<<next_face.idx);
            BOOL_MESH_DEBUG_SHOW(start_face.idx);

            bool addNewFaceToMap = false;
            std::unordered_set<HE_FaceHandle>* endPoints = nullptr;
            { // check whether the fracture line hasn't already been created
                std::unordered_map<HE_FaceHandle, std::unordered_set<HE_FaceHandle> >
                    ::iterator endPointsFound = walkedFacesFromEndpoints.find(next_face);
                if (endPointsFound == walkedFacesFromEndpoints.end())
                {
                    BOOL_MESH_DEBUG_PRINTLN("new face to walk: " << next_face.idx);
                    addNewFaceToMap = true;
                }
                else
                {
                    BOOL_MESH_DEBUG_PRINTLN("already walked face: " << next_face.idx);
                    endPoints = &endPointsFound->second;
                    std::unordered_set<HE_FaceHandle>::const_iterator faceFound = endPoints->find(start_face);
                    if (faceFound != endPoints->end()) // if face is found
                    {
                        BOOL_MESH_DEBUG_PRINTLN("already walked face fracture! skipping");
                        continue; // skip the fracture line! it has already been checked!
                    } else
                        BOOL_MESH_DEBUG_PRINTLN(", but with other endpoint/startingpoint...");
                }
            }

            typedef std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>>::iterator val_it;
            std::vector<FractureLinePart> fracs;
            fracs.emplace_back(next_face);
            std::pair<val_it, bool> new_mapping = face2fractures.emplace(next_face, fracs);
            if (!new_mapping.second)
                new_mapping.first->second.emplace_back(next_face);




            FractureLinePart& next_frac = new_mapping.first->second.back();
            getFacetFractureLinePart(next_face, start_face, arrow->data.lineSegment, next_frac);




//            fractureLines.emplace_back(next_face);
//            FractureLinePart& next_frac = fractureLines.back();
//
//            getFacetFractureLinePart(next_face, start_face, arrow->data.lineSegment, next_frac);
//
//            face2fractures.insert(std::pair<HE_FaceHandle, FractureLinePart>(next_face, next_frac));

            todo.push_front(&next_frac);



            std::unordered_set<HE_FaceHandle> endPointFaces;
            next_frac.endPointsFaces(endPointFaces);
            if (addNewFaceToMap)
            {
                walkedFacesFromEndpoints.insert(std::pair<HE_FaceHandle, std::unordered_set<HE_FaceHandle> >(next_face, endPointFaces) );
            } else
            {
                endPoints->insert(endPointFaces.begin(), endPointFaces.end());
            }

        }
    }
}

spaceType inaccuracy = 20; //!< the margin of error between the endpoint of the first triangle and the startpoint of the second


/*!
Walk over [triangle1] along the intersections it has with faces connected to [triangle2],
starting at the [first] intersection and going in the direction first.from -> first.to until we would exit [triangle1].

Store the intersection segments in a graph in the [result].

Also the [result] stores the starting intersection segment, the endpoints where the fracture exits [triangle1] and the faces involved in each intersection segment.

In case the [first] intersection is not an entering intersection of [triangle1], i.e. when [first].from is not on an edge/vertex of triangle1,
the fracture stored in result is not the whole fracture covering [triangle1], but is may cover only half the fracture line part.

In case the intersection doesn't exit [triangle1] the graph of intersection segments in [result] will be a cyclic graph, with no endpoints.
*/
void BooleanMeshOps::getFacetFractureLinePart(HE_FaceHandle& triangle1, HE_FaceHandle& triangle2, TriangleIntersection& first, FractureLinePart& result)
{
//    DEBUG_DO(
//        std::ofstream csv;
//        csv.open("CRAP.csv", std::fstream::out | std::fstream::app);
//        csv << "100000,100000,100000" << std::endl;
//        csv.close();
//    );

    BOOL_MESH_DEBUG_PRINTLN(" starting getFacetFractureLinePart");
    BOOL_MESH_DEBUG_PRINTLN("\n\n\n\n\n\n\n\n");


    typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;
    typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;

    std::unordered_map<HE_VertexHandle, Node*> vertex2node; // to know when we have closed a cycle
    // vertices are the only places where the fracture can split

    std::unordered_set<HE_FaceHandle> checked_faces;

    BOOL_MESH_DEBUG_PRINTLN((first.intersectionType));

    if (first.to->belongsToSource(triangle1))
    {
        std::swap(first.to, first.from);
        first.isDirectionOfInnerPartOfTriangle1 = ! first.isDirectionOfInnerPartOfTriangle1;
        first.isDirectionOfInnerPartOfTriangle2 = ! first.isDirectionOfInnerPartOfTriangle2;
    }

    Node* first_node = result.fracture.addNode(*first.from);
    Node* second_node = result.fracture.addNode(*first.to);
    Arrow* first_arrow = result.fracture.connect(*first_node, *second_node, IntersectionSegment(first, triangle2, false)); // triangleIntersection was computed on (triangle2, triangle1) !

    result.start = first_arrow;


    if (first.to->belongsToSource(triangle1))
    {
        result.endPoints.push_back(first_arrow);
        BOOL_MESH_DEBUG_PRINTLN("FractureLinePart (small):");
        result.debugOutput();
        return;
    }



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

        if (connectingPoint.getSourceMesh() == triangle1.m)
        {
            BOOL_MESH_DEBUG_PRINTLN(" WARNING! intersecting mesh with itself!");
        }

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

                            saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "problem.stl");
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
        csv.open("CRAP.csv", std::fstream::out | std::fstream::app);
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

//    BOOL_MESH_DEBUG_PRINTLN("FractureLinePart:");
//    result.debugOutput();
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
        triangleIntersection.isDirectionOfInnerPartOfTriangle1 = ! triangleIntersection.isDirectionOfInnerPartOfTriangle1;
        triangleIntersection.isDirectionOfInnerPartOfTriangle2 = ! triangleIntersection.isDirectionOfInnerPartOfTriangle2;
    }
    if (! triangleIntersection.from->compareSourceConverse(connectingPoint)  )
    {
        BOOL_MESH_DEBUG_PRINTLN("WARNING! subsequent intersection segments do not connect at source mesh!");
        if ( (triangleIntersection.to->p() - connectingPoint.p()).vSize2() < (triangleIntersection.from->p() - connectingPoint.p()).vSize2() )
        {
            BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" =~= "<< triangleIntersection.to->p());
            BOOL_MESH_DEBUG_PRINTLN("swapping...");
            std::swap(triangleIntersection.from, triangleIntersection.to);
            triangleIntersection.isDirectionOfInnerPartOfTriangle1 = ! triangleIntersection.isDirectionOfInnerPartOfTriangle1;
            triangleIntersection.isDirectionOfInnerPartOfTriangle2 = ! triangleIntersection.isDirectionOfInnerPartOfTriangle2;
        }
    }
    if ( ! (connectingPoint.p() - triangleIntersection.from->p()).testLength(inaccuracy)  )
    {
        BOOL_MESH_DEBUG_PRINTLN("WARNING! large difference between endpoint of first triangle intersection and start point of second! :");
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

    { //update old point:
        PointD middle = PointD(connectingPoint.p()) + PointD(triangleIntersection.from->p());
        connectingPoint.p() = (middle / 2).downCast();
        // TODO: more advanced middling scheme when multiple l;ines connect to the same point
    }

    bool new_point_is_already_done = false;
    if      (triangleIntersection.to->getType() == IntersectionPointType::VERTEX)
    {
        std::unordered_map<HE_VertexHandle, Node*>::const_iterator
            node = vertex2node.find(triangleIntersection.to->vertex);

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

            PointD middle = PointD(new_node->data.p()) + PointD(triangleIntersection.to->p());
            new_node->data.p() = (middle / 2).downCast();
        }
    } else
    {
        if ( triangleIntersection.to->compareSourceConverse( result.start->from->data ) )
        {
            BOOL_MESH_DEBUG_PRINTLN("connecting to starting node...");
            new_node = result.start->from;

            PointD middle = PointD(new_node->data.p()) + PointD(triangleIntersection.to->p());
            new_node->data.p() = (middle / 2).downCast();
        } else
        {
            BOOL_MESH_DEBUG_PRINTLN("normal case: adding node...");
            new_node = result.fracture.addNode(*triangleIntersection.to);
        }
    }

    // make the new arrow
    BOOL_MESH_DEBUG_PRINTLN("making new arrow with face: " << newFace.m <<" "<<newFace.idx);
    Arrow* new_arrow = result.fracture.connect(connectingNode, *new_node, IntersectionSegment(triangleIntersection, newFace, true));


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
    std::remove("CRAP.csv");
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
        saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "crappa.stl");
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

    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fractures;
    //std::unordered_set<HE_FaceHandle>  walkedFacesFromEndpoints;

    completeFractureLine(otherFace, intersectingFace, *triangleIntersection, face2fractures);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");

//    std::ofstream csv("CRAP.csv");
//    for (Node* node : result.fracture.nodes)
//    {
//        csv << node->data.p().x <<", " << node->data.p().y << ", " << node->data.p().z << std::endl;
//    }
//    csv.close();
}

void BooleanMeshOps::test_completeFractureLine(PrintObject* model)
{
    std::remove("CRAP.csv");
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


    std::ofstream out("test_completeFractureLine.stl");

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

    std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> face2fractures;
    completeFractureLine(otherFace, intersectingFace, *triangleIntersection, face2fractures);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");


}


void BooleanMeshOps::test_subtract(PrintObject* model)
{
    std::remove("CRAP.csv");
    std::remove("WHOLE.csv");
    std::remove("WHOLE_keep.csv");
    std::remove("WHOLE_subtracted.csv");

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


    std::ofstream out("test_subtract.stl");

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


    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << "=== start subtract ===\n" << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    HE_Mesh result;
    subtract(heMesh, other, result);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");


}
