#include "boolMesh.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

#include "modelFile/modelFile.h" // PrintObject

#include <stdio.h>

#include <cmath> // acos

#include "settings.h" // MAX_EDGES_PER_VERTEX

#include "MACROS.h" // debug

#include <fstream> // write to file (debug)

#include <boost/iterator/transform_iterator.hpp>


bool debug_show_triangle_edges_connected_to_fracture_line = true;

/*!
get the segments belonging to the edges of the face which belong to the part above/below the fracture lines

algorithm idea:
1) order all endPoints on angle from the middle point of the face
2) group consecutive pairs
    (uneven, uneven+1) or (even, even+1) depending on the direction of the intersection segments at the endpoints
-) add indirection via vertices of face which are not already present in the endpoints

idea2:
-order endPoints per edge on distance to first vertex of edge
    keep vertices separate
    check which vertices are occupied
-divide edges into parts
-leave out edges which have no endPoints on it and for which the previous or next edge doesn't begin on the vertex
*/
void getFaceEdgeSegments(bool aboveIntersection, std::vector<FractureLinePart>& fracturesOnFace, std::vector<std::pair<IntersectionPoint, IntersectionPoint>>& segments)
{
    if (fracturesOnFace.size() == 0) return;

    // TODO: check whether fracture is simple (1 or 2 segments?) and do it the simple way in that case

    HE_FaceHandle face = fracturesOnFace[0].face;

    #if BOOL_MESH_DEBUG == 1
        // check fractures on bugs!
        // : check whether all fractures lie on same face
        // : check whether all fracture lines are aligned correctly
        // : check whether a fracture line lies on an edge (warning)
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
                    auto direction = [](Arrow* prev) { return prev->data.isDirectionOfInnerPartOfMainTriangle(); };

                    if (direction(a) != direction(prev))
                    {
                        BOOL_MESH_DEBUG_PRINTLN("ERROR! subsequent intersections not directed the same way!");
                        BOOL_MESH_DEBUG_SHOW(direction(a));
                        BOOL_MESH_DEBUG_SHOW(direction(prev));
                    }
                }
                if (a->from->data.compareSource(a->to->data))
                    BOOL_MESH_DEBUG_PRINTLN("WARNING! intersection segment lies on the edge of a face!");
            }
        }
    #endif


    struct DirectedPoint
    {
        IntersectionPoint p;
        bool upToDown;
        DirectedPoint(IntersectionPoint p, bool upToDown) : p(p), upToDown(upToDown) { };
    };
    auto isEdgeStart = [&aboveIntersection] (DirectedPoint dp) { return dp.upToDown != aboveIntersection; };  // TODO: other way around??


    DirectedPoint* v0 = nullptr;
    DirectedPoint* v1 = nullptr;
    DirectedPoint* v2 = nullptr;
    std::vector<DirectedPoint> endPoints_e0;
    std::vector<DirectedPoint> endPoints_e1;
    std::vector<DirectedPoint> endPoints_e2;


    auto processArrow = [&] (Arrow* a, bool endPoint)
    {
        DirectedPoint dp( (endPoint)? a->to->data : a->from->data  ,  a->data.isDirectionOfInnerPartOfMainTriangle() == endPoint );
        switch (dp.p.type)
        {
        case IntersectionPointType::NEW:
        {
            HE_EdgeHandle face_edge = (dp.p.edge.face() == face)? dp.p.edge : dp.p.edge.converse();
            if      (face_edge == face.edge0()) endPoints_e0.push_back(dp);
            else if (face_edge == face.edge1()) endPoints_e1.push_back(dp);
            else if (face_edge == face.edge2()) endPoints_e2.push_back(dp);
            else BOOL_MESH_DEBUG_PRINTLN("WARNING! didn't recognize edge to lie on face!!");
        } break;
        case IntersectionPointType::VERTEX:
            if      (dp.p.vertex == face.v0()) v0 = new DirectedPoint(dp);
            else if (dp.p.vertex == face.v1()) v1 = new DirectedPoint(dp);
            else if (dp.p.vertex == face.v2()) v2 = new DirectedPoint(dp);
            else BOOL_MESH_DEBUG_PRINTLN("WARNING! didn't recognize vertex to lie on face!!");
        break;
        }
    };

    for (FractureLinePart p : fracturesOnFace)
    {
        for (Arrow* a : p.endPoints)
        {
            processArrow(a, true);
        }
        processArrow(p.start, false);
    }


    auto compareDistTo = [](Point& to)
    {
        return [&to](DirectedPoint& a, DirectedPoint& b) { return (a.p.p()-to).vSize2() < (b.p.p()-to).vSize2(); };
    };

    std::sort(endPoints_e0.begin(), endPoints_e0.end(), compareDistTo(face.p0()));
    std::sort(endPoints_e1.begin(), endPoints_e1.end(), compareDistTo(face.p1()));
    std::sort(endPoints_e2.begin(), endPoints_e2.end(), compareDistTo(face.p2()));


    bool edgeNeedsYetToBeProcessed[3] = { }; // initialize as false

    //                 / face, &segments, &aboveIntersection
    auto processEdge = [&](std::vector<DirectedPoint>& endPoints, HE_EdgeHandle edge, bool* edgeNeedsYetToBeProcessed, DirectedPoint* v0, DirectedPoint* v1)
    {
        if (endPoints.size() > 0)
        {
            int pairing = -1;
            if ( ! isEdgeStart(endPoints[0]))
            {
                pairing = 0;
                segments.emplace_back(IntersectionPoint(edge.v0()), endPoints[0].p);
            }
            for (int i = 1; i < endPoints.size(); i+=2)
            {
                DirectedPoint a = endPoints[i+pairing];
                DirectedPoint b = endPoints[i+1+pairing];
                segments.emplace_back(a.p, b.p);
            }

            if ( isEdgeStart(endPoints.back()))
            {
                segments.emplace_back(endPoints.back().p, IntersectionPoint(edge.v1()));
            }
        } else
        {
            if (v0 != nullptr)
            {
                if (isEdgeStart(*v0))
                    segments.emplace_back(v0->p, IntersectionPoint(edge.v1()));
            } else if (v1 != nullptr)
            {   if (!isEdgeStart(*v1))
                    segments.emplace_back(IntersectionPoint(edge.v0()), v1->p);
            } else *edgeNeedsYetToBeProcessed = true;
        }
    };

    processEdge(endPoints_e0, face.edge0(), &edgeNeedsYetToBeProcessed[0], v0, v1);
    processEdge(endPoints_e1, face.edge1(), &edgeNeedsYetToBeProcessed[1], v1, v2);
    processEdge(endPoints_e2, face.edge2(), &edgeNeedsYetToBeProcessed[2], v2, v0);

    // some whole edges may still need to be included
    if (true)
    {
        auto processWholeEdge = [&](bool edgeNeedsYetToBeProcessed, bool prev_edgeNeedsYetToBeProcessed, bool next_edgeNeedsYetToBeProcessed
                                    , HE_EdgeHandle e, HE_EdgeHandle e_prev, HE_EdgeHandle e_next
                                    , std::vector<DirectedPoint>& endPoints, std::vector<DirectedPoint>& prev_endPoints, std::vector<DirectedPoint>& next_endPoints
                                    , DirectedPoint* v0, DirectedPoint* v1, DirectedPoint* vother)
        {
            if (edgeNeedsYetToBeProcessed && !prev_edgeNeedsYetToBeProcessed)
            {
                // note that v0 of this edge is not in the fracture, see code 20 lines up
                bool includeEdge = false;
                if (prev_endPoints.size() == 0 && vother != nullptr)
                    includeEdge = isEdgeStart(*vother);
                else includeEdge = isEdgeStart(prev_endPoints.back());

                if (includeEdge)
                {
                    segments.emplace_back(IntersectionPoint(e.v0()), IntersectionPoint(e.v1()));
                    if (next_edgeNeedsYetToBeProcessed)
                        segments.emplace_back(IntersectionPoint(e_next.v0()), IntersectionPoint(e_next.v1()));
                }
            }

        };
        processWholeEdge(edgeNeedsYetToBeProcessed[0], edgeNeedsYetToBeProcessed[2], edgeNeedsYetToBeProcessed[1], face.edge0(), face.edge2(), face.edge1(), endPoints_e0, endPoints_e2, endPoints_e1, v0, v1, v2);
        processWholeEdge(edgeNeedsYetToBeProcessed[1], edgeNeedsYetToBeProcessed[0], edgeNeedsYetToBeProcessed[2], face.edge1(), face.edge0(), face.edge2(), endPoints_e1, endPoints_e0, endPoints_e2, v1, v2, v0);
        processWholeEdge(edgeNeedsYetToBeProcessed[2], edgeNeedsYetToBeProcessed[1], edgeNeedsYetToBeProcessed[0], face.edge2(), face.edge1(), face.edge0(), endPoints_e2, endPoints_e1, endPoints_e0, v2, v0, v1);

    //    if ( !  (
    //               !edgeNeedsYetToBeProcessed[0] && !edgeNeedsYetToBeProcessed[1] && !edgeNeedsYetToBeProcessed[2]
    //            || edgeNeedsYetToBeProcessed[0] && edgeNeedsYetToBeProcessed[1] && edgeNeedsYetToBeProcessed[2]
    //            )   )
    //    { // some whole edges may still need to be included
    //
    //    }

    }

    if (v0 != nullptr) delete v0;
    if (v1 != nullptr) delete v1;
    if (v2 != nullptr) delete v2;
}




void BooleanMeshOps::debug_csv(std::unordered_map<HE_FaceHandle, std::vector<FractureLinePart>> & face2fractures, std::string filename)
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


            offset_now += offset;
        }

        bool aboveIntersection = useAboveFracture(mapping.first);

        if (debug_show_triangle_edges_connected_to_fracture_line)
        {
            std::vector<std::pair<IntersectionPoint, IntersectionPoint>> segments;
            getFaceEdgeSegments(aboveIntersection, mapping.second, segments);

            for (std::pair<IntersectionPoint, IntersectionPoint> segment : segments)
            {
                p = segment.first.p() + offset_now;
                csv << p.x <<", " << p.y << ", " << p.z << std::endl;
                p = segment.second.p() + offset_now;
                csv << p.x <<", " << p.y << ", " << p.z << std::endl;
            }
        }
    }
    csv << p.x <<", " << p.y << ", " << p.z << std::endl;
    csv << "5000,0,30000" << std::endl;
    csv.close();
};


















std::shared_ptr<TriangleIntersection> BooleanMeshOps::getIntersection(HE_FaceHandle tri1, HE_FaceHandle tri2)
{
    std::shared_ptr<TriangleIntersection> ret = TriangleIntersectionComputation::intersect(tri1, tri2);
    if (ret->intersectionType != IntersectionType::LINE_SEGMENT)
        return ret;
    /*
    possible cases when edgeOfTriangle1TouchingTriangle2 or vice versa:


    legend:
    - /     =faces
    o       =intersection line (top view!)
    x       =model inside

    A
    1---o---1
     xx/x\xx
      2   2

    B
     xxx xxx
    1---o---1
       /x\
      2   2

    C
    1----o----1
     xxx/ \xxx
      x/   \x
      2     2

    D                    |x
    1----o----2          |x                         |x
     xxx/x\xxx        ---o---    =   ---o---     +  o
       /x x\          xxx|xxx        xxx xxx        |x
      2     1            |x


    include edge?         A | B | C | D
    1 union 2           | x | x | x | v
    1 intersection 2    | x | x | x | v
    1 - 2               | x | x | x | v
    2 - 1               | x | x | x | v

    */
    if (ret->edgeOfTriangle1TouchingTriangle2 || ret->edgeOfTriangle2TouchingTriangle1)
    {
        std::shared_ptr<TriangleIntersection> other_intersection_12;
        HE_FaceHandle* converse_12;
        if (ret->edgeOfTriangle1TouchingTriangle2)
        {
            converse_12 = new HE_FaceHandle(ret->edgeOfTriangle1TouchingTriangle2->converse().face());
            other_intersection_12 = TriangleIntersectionComputation::intersect(*converse_12, tri2);
            if (other_intersection_12->intersectionType != IntersectionType::COPLANAR)
            { // align direction
                if (other_intersection_12->from->compareSource(*ret->to))
                    other_intersection_12->reverse();
                if ( ! (other_intersection_12->from->compareSource(*ret->from) && other_intersection_12->to->compareSource(*ret->to) ) )
                    BOOL_MESH_DEBUG_PRINTLN("WARNING! edge-triangle intersection doesn't coincide with converse_edge-triangle intersection!");
            }
        }
        std::shared_ptr<TriangleIntersection> other_intersection_21;
        HE_FaceHandle* converse_21;
        if (ret->edgeOfTriangle2TouchingTriangle1)
        {
            converse_21 = new HE_FaceHandle(ret->edgeOfTriangle2TouchingTriangle1->converse().face());
            other_intersection_21 = TriangleIntersectionComputation::intersect(tri1, *converse_21);
            if (other_intersection_21->intersectionType != IntersectionType::COPLANAR)
            { // align direction
                if (other_intersection_21->from->compareSource(*ret->to))
                    other_intersection_21->reverse();
                if ( ! (other_intersection_21->from->compareSource(*ret->from) && other_intersection_21->to->compareSource(*ret->to) ) )
                    BOOL_MESH_DEBUG_PRINTLN("WARNING! edge-triangle intersection doesn't coincide with converse_edge-triangle intersection!");
            }
        }


        enum class Op { UNION = 0, INTERSECTION = 1, DIFF_12 = 2, DIFF_21 = 3 , WTF = 4};
        auto getOp =  [tri1, tri2, this](BoolOpType op)
        {
            switch (op)
            {
            case BoolOpType::UNION: return Op::UNION;
            case BoolOpType::INTERSECTION: return Op::INTERSECTION;
            case BoolOpType::DIFFERENCE:
            if (tri1.m == &keep) return Op::DIFF_12;
            if (tri2.m == &keep) return Op::DIFF_21;
            break;
            }
            BOOL_MESH_DEBUG_PRINTLN("Error! unknown bool op type.");
            exit(0);
            return Op::WTF;
        };
        Op op_used = getOp(boolOpType);

        bool* edge_table [4];
        edge_table[0] = new bool[4] { true,  true,  false, false };
        edge_table[1] = new bool[4] { false, true,  true , false };
        edge_table[2] = new bool[4] { false, true,  true , false };
        edge_table[3] = new bool[4] { false, false, true , true  };

        bool* facet_table [4];
        facet_table[0] = new bool[4] { false, true,  false, true  };
        facet_table[1] = new bool[4] { false, true,  false, false };
        facet_table[2] = new bool[4] { false, false, true,  false };
        facet_table[3] = new bool[4] { false, false, true,  false };

        if (ret->edgeOfTriangle1TouchingTriangle2 && ret->edgeOfTriangle2TouchingTriangle1)
        {
            std::shared_ptr<TriangleIntersection> other_other_intersection =  TriangleIntersectionComputation::intersect(*converse_12, *converse_21);
            BOOL_MESH_DEBUG_PRINTLN("WHAT TO DO HERE? boolMesh.cpp:" << __LINE__);
        } else if (ret->edgeOfTriangle1TouchingTriangle2 && !ret->edgeOfTriangle2TouchingTriangle1)
        {
            if (other_intersection_12->intersectionType == IntersectionType::COPLANAR)
            {

                int column = -1;
                bool sameNormals = converse_12->normal().dot(tri2.normal()) > 0;

                bool intersectionDir_is_edgeDir = ( ret->edgeOfTriangle1TouchingTriangle2->p0() - ret->from->p() ).vSize2() < ( ret->edgeOfTriangle1TouchingTriangle2->p0() - ret->to->p() ).vSize2();
                bool underFace = ret->isDirectionOfInnerPartOfTriangle1 == intersectionDir_is_edgeDir; // whether non-coplanar face is outside tri2
                if (sameNormals)
                {
                    if (!underFace)
                        column = 1;
                    else
                        column = 3;
                } else
                {
                    if (!underFace)
                        column = 0;
                    else
                        column = 2;
                }
                bool use_facet = facet_table[static_cast<int>(op_used)][column];
                bool use_edge = edge_table[static_cast<int>(op_used)][column];
                if (!use_edge)
                {
                    ret->intersectionType = IntersectionType::NON_TOUCHING; // edge on face is equivalent to infinitesimally far outside/inside the mesh
                    BOOL_MESH_DEBUG_PRINTLN("discarding edge-triangle intersection");
                }
            } else if (ret->isDirectionOfInnerPartOfTriangle2 != other_intersection_12->isDirectionOfInnerPartOfTriangle2)
            {
                ret->intersectionType = IntersectionType::NON_TOUCHING; // edge on face is equivalent to infinitesimally far outside/inside the mesh
                BOOL_MESH_DEBUG_PRINTLN("discarding edge-triangle intersection");
            }
            return ret;
        } else if (!ret->edgeOfTriangle1TouchingTriangle2 && ret->edgeOfTriangle2TouchingTriangle1)
        {
            if (other_intersection_21->intersectionType == IntersectionType::COPLANAR)
            {

                int column = -1;
                bool sameNormals = converse_21->normal().dot(tri1.normal()) > 0;

                bool intersectionDir_is_edgeDir = ( ret->edgeOfTriangle2TouchingTriangle1->p0() - ret->from->p() ).vSize2() < ( ret->edgeOfTriangle2TouchingTriangle1->p0() - ret->to->p() ).vSize2();
                bool underFace = ret->isDirectionOfInnerPartOfTriangle2 == intersectionDir_is_edgeDir; // whether non-coplanar face is outside tri2
                if (sameNormals)
                {
                    if (!underFace)
                        column = 1;
                    else
                        column = 3;
                } else
                {
                    if (!underFace)
                        column = 0;
                    else
                        column = 2;
                }
                bool use_facet = facet_table[static_cast<int>(op_used)][column];
                bool use_edge = edge_table[static_cast<int>(op_used)][column];
                if (!use_edge)
                {
                    ret->intersectionType = IntersectionType::NON_TOUCHING; // edge on face is equivalent to infinitesimally far outside/inside the mesh
                    BOOL_MESH_DEBUG_PRINTLN("discarding edge-triangle intersection");
                }
            } else if (ret->isDirectionOfInnerPartOfTriangle1 != other_intersection_21->isDirectionOfInnerPartOfTriangle1)
            {
                ret->intersectionType = IntersectionType::NON_TOUCHING; // edge on face is equivalent to infinitesimally far outside/inside the mesh
                BOOL_MESH_DEBUG_PRINTLN("discarding edge-triangle intersection");
            }
            return ret;
        } else return ret;


        delete converse_12;
        delete converse_21;
    }
    return ret;
}


void BooleanMeshOps::subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result)
{

    BooleanMeshOps subtract(keep, subtracted, BoolOpType::DIFFERENCE);
    return subtract.perform(result);
}

void BooleanMeshOps::perform(HE_Mesh& result)
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

            std::shared_ptr<TriangleIntersection> triangleIntersection = getIntersection(tri1, tri2);

//            if (!triangleIntersection->from || !triangleIntersection->to)
            if (triangleIntersection->intersectionType != IntersectionType::LINE_SEGMENT)
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





                BOOL_MESH_DEBUG_PRINTLN("next fracture line.....................");
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

int innerSize = 0;
int innerSize_k = 0;
int innerSize_s = 0;

for (std::pair<HE_FaceHandle, std::vector<FractureLinePart>> face_intersectingFaces : face2fractures)
{
    innerSize += face_intersectingFaces.second[0].fracture.arrows.size();
    if (face_intersectingFaces.first.m == &keep)
    {
        face2fractures_keep.insert(face_intersectingFaces);
        innerSize_k += face_intersectingFaces.second[0].fracture.arrows.size();
    } else if (face_intersectingFaces.first.m == &subtracted)
    {
        face2fractures_subtracted.insert(face_intersectingFaces);
        innerSize_s += face_intersectingFaces.second[0].fracture.arrows.size();
    } else
    {
        DEBUG_PRINTLN("face belongs to no mesh!");
        exit(0);

    }
}

BOOL_MESH_DEBUG_SHOW(face2fractures.size());
BOOL_MESH_DEBUG_SHOW(face2fractures_keep.size());
BOOL_MESH_DEBUG_SHOW(face2fractures_subtracted.size());
BOOL_MESH_DEBUG_SHOW(innerSize);
BOOL_MESH_DEBUG_SHOW(innerSize_k);
BOOL_MESH_DEBUG_SHOW(innerSize_s);

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

    BOOL_MESH_DEBUG_PRINTLN("\n\n");
    BOOL_MESH_DEBUG_PRINTLN("starting completeFractureLine");
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
        BOOL_MESH_DEBUG_PRINTLN("next_face fracture line part...");
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
    BOOL_MESH_DEBUG_PRINTLN("end completeFractureLine");
    BOOL_MESH_DEBUG_PRINTLN("\n\n");

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

    BOOL_MESH_DEBUG_PRINTLN("\n\n");
    BOOL_MESH_DEBUG_PRINTLN(" starting getFacetFractureLinePart");


    typedef Graph<IntersectionPoint, IntersectionSegment>::Node Node;
    typedef Graph<IntersectionPoint, IntersectionSegment>::Arrow Arrow;

    std::unordered_map<HE_VertexHandle, Node*> vertex2node; // to know when we have closed a cycle
    // vertices are the only places where the fracture can split

    std::unordered_set<HE_FaceHandle> checked_faces;

    BOOL_MESH_DEBUG_PRINTLN((first.intersectionType));

    if (first.to->belongsToSource(triangle1))
    {
        first.reverse();
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
        BOOL_MESH_DEBUG_PRINTLN("next segment on face fracture line part...");
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

        std::vector<std::tuple<HE_FaceHandle, TriangleIntersection>> next_faces;
        getNextFacesOnFracture(triangle1, current, checked_faces, next_faces);
        for (std::tuple<HE_FaceHandle, TriangleIntersection> next_face : next_faces)
            addIntersectionToGraphAndTodo(connectingNode, std::get<1>(next_face), triangle1, std::get<0>(next_face), vertex2node, result, todo);
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



// NOTES ON HOW TO HANDLE EDGES WHICH COINCIDE WITH THE INTERSECTION
// ======================================================================================================

//if a normal intersectionsegment in mesh1
//    - ends in a vertex:
//        try all connected faces
//    - ends in an edge:
//        try connected face
//    - ends in other mesh:
//        stay on same face!
// same for mesh2


//for a segment which is part of an edge in mesh2: if the intersection in mesh1
//    - ends in a vertex:
//        try all connected faces                   SAME  :)
//    - ends in an edge:
//        try connected face                        SAME  :)
//    - ends in other mesh:
//        stay on same face!                        SAME  :)
//for a segment which is part of an edge in mesh1: if the intersection in mesh1
//    - ends in a vertex:
//        try all connected faces                   SAME  :)
//    - ends on the edge:
//        impossible!                               IGNORED  :P
//    - ends in other mesh:
//        try this face and the connected face!     DIFFERENT!!!!!!!! O_O
//
// combine these rules symmetrically for the other mesh!
//  -> try all combinations of possible triangles of mesh1 and mesh2


// how to handle the DIFFERENT case?
// a) when it occurs in triangle2 (the short-lived)
// b) when it occurs in triangle1 (the main triangle of which we are creating the fracture line part)
//
// a)
// case: segment coincident with edge of triangle2 ends in triangle1
// exit triangle1
// becomes b) case!
//
// b)
// case: segment coincident with edge of triangle1 ends in triangle2
// ! must generally also be the starting segment, since starting point lies on edge of triangle1 (thus would have exitted already)
//      (exception: when this case continues after the starting segment, and consecutive segments are also on the same edge of triangle1, ending in triangle2)
// if the next intersection segment is with triangle1: continue fracture line
// if the next intersection segment is with its converse: end fracture line

//
// FURTHER CONSIDERATIONS
//
// Don't consider the intersection to be an intersection when:
// c- both triangles sharing the edge lie on the same side of the other triangle and we do UNION or INTERSECTION
// d- one triangle of the triangles sharing the edge is coplanar with the other triangle and useCoplanarFaceIntersection(...) == false

//
// IMPLEMENTATION
//
// a) none
// b) where we mark an arrow as endpoint
// c) right after we compute an intersection
// d) [idem]




void BooleanMeshOps::getNextFacesOnFracture(HE_FaceHandle triangle1, Arrow* current, std::unordered_set<HE_FaceHandle>& checked_faces, std::vector<std::tuple<HE_FaceHandle, TriangleIntersection>>& result)
{
    IntersectionPoint& connectingPoint = current->to->data;

    switch (connectingPoint.getType())
    {
    case IntersectionPointType::NEW: // intersection with edge
    {
        HE_FaceHandle newFace = connectingPoint.edge.converse().face();
        BOOL_MESH_DEBUG_PRINTLN(" NEW case for face " << newFace.idx);
        if (checked_faces.find(newFace) == checked_faces.end())
        {
            checked_faces.insert(newFace);
            std::shared_ptr<TriangleIntersection> triangleIntersection = getIntersection(triangle1, newFace);//, connectingPoint.p());
            if (triangleIntersection->edgeOfTriangle2TouchingTriangle1)
            {
                HE_FaceHandle otherFace = triangleIntersection->edgeOfTriangle2TouchingTriangle1->converse().face();
                checked_faces.insert(otherFace);
            }

            if (! triangleIntersection->from || ! triangleIntersection->to)
                debug_export_problem(triangle1, triangleIntersection, newFace, connectingPoint);

            result.emplace_back(newFace, *triangleIntersection);

            //if (current->data.lineSegment)
        } else
        {
//                BOOL_MESH_DEBUG_PRINTLN("face " << newFace.idx <<" checked already!");
//                BOOL_MESH_DEBUG_PRINTLN("face : " << current->data.lineSegment.to->edge.converse().face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev face (from): " << current->data.lineSegment.from->edge.face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev face (to): " << current->data.lineSegment.to->edge.face().idx);
//                BOOL_MESH_DEBUG_PRINTLN("prev prev face : " << current->data.lineSegment.from->edge.converse().face().idx);
        }
    } break;
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
                    std::shared_ptr<TriangleIntersection> triangleIntersection = getIntersection(triangle1, newFace);// , connectingPoint.p());
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
                        result.emplace_back(newFace, *triangleIntersection);
                    }
                }
            }

            outEdge = outEdge.converse().next();
        } while ( outEdge != first_outEdge);
    break;
    }
}







void BooleanMeshOps::addIntersectionToGraphAndTodo(Node& connectingNode, TriangleIntersection& triangleIntersection, HE_FaceHandle originalFace,
            HE_FaceHandle newFace, std::unordered_map<HE_VertexHandle, Node*>& vertex2node, FractureLinePart& result, std::list<Arrow*>& todo)
{
    IntersectionPoint& connectingPoint = connectingNode.data;


    Node* new_node;

    { // make direction of arrows uniform over graph:
        if ( triangleIntersection.to->compareSourceConverse(connectingPoint) )
        {
            BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" =~= "<< triangleIntersection.to->p());
            BOOL_MESH_DEBUG_PRINTLN("reversing intersectionDirection...");
            triangleIntersection.reverse();
        }
        if (! triangleIntersection.from->compareSourceConverse(connectingPoint)  )
        {
            BOOL_MESH_DEBUG_PRINTLN("WARNING! subsequent intersection segments do not connect at source mesh!");
            if ( (triangleIntersection.to->p() - connectingPoint.p()).vSize2() < (triangleIntersection.from->p() - connectingPoint.p()).vSize2() )
            {
                BOOL_MESH_DEBUG_PRINTLN(connectingPoint.p() <<" =~= "<< triangleIntersection.to->p());
                BOOL_MESH_DEBUG_PRINTLN("reversing intersectionDirection...");
                triangleIntersection.reverse();
            }
        }
    }

    if ( ! (connectingPoint.p() - triangleIntersection.from->p()).testLength(inaccuracy)  )
    {
        debug_export_difference_mesh(originalFace, connectingPoint, triangleIntersection, newFace);
    }

    { //update old point:
        PointD middle = PointD(connectingPoint.p()) + PointD(triangleIntersection.from->p());
        connectingPoint.p() = (middle / 2).downCast();
        // TODO: more advanced middling scheme when multiple l;ines connect to the same point
    }

    bool new_point_is_already_done = false;

    { // find / construct the new_node
        switch (triangleIntersection.to->getType())
        {
        case IntersectionPointType::VERTEX:
        {
            std::unordered_map<HE_VertexHandle, Node*>::const_iterator
                node = vertex2node.find(triangleIntersection.to->vertex); // also checks whether we connect to the starting node?!?!

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
        } break;
        case IntersectionPointType::NEW:
        {
            if ( triangleIntersection.to->compareSourceConverse( result.start->from->data ) )
            {
                BOOL_MESH_DEBUG_PRINTLN("connecting to starting node...");
                new_node = result.start->from;

                PointD sum = PointD(new_node->data.p()) + PointD(triangleIntersection.to->p()); // compute middle
                new_node->data.p() = (sum / 2).downCast();
            } else
            {
                BOOL_MESH_DEBUG_PRINTLN("normal case: adding node...");
                new_node = result.fracture.addNode(*triangleIntersection.to);
            }
        } break;
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
        {
            bool edge_case = false;
            { // exception for edge-triangle intersection
                if (new_arrow->data.edgeOfMainTouchingOtherTriangle())
                {
                    std::shared_ptr<TriangleIntersection> next_triangleIntersection = TriangleIntersectionComputation::intersect(originalFace, new_node->data.edge.converse().face());
                    if (next_triangleIntersection->intersectionType == IntersectionType::LINE_SEGMENT)
                    {
                        edge_case = true;
                        BOOL_MESH_DEBUG_PRINTLN("edge-triangle edgecase: continuing fracture line part on edge of main triangle");
                    }
                }

            }
            if (new_node->data.edge.face() == originalFace && !edge_case)
            {
                BOOL_MESH_DEBUG_PRINTLN("exiting triangle");
                if (new_arrow->data.lineSegment.intersectionType == IntersectionType::COPLANAR)
                    BOOL_MESH_DEBUG_PRINTLN("ERROR! ERROR! new_arrow->data.intersectionType == IntersectionType::COPLANAR");
                result.endPoints.push_back(new_arrow);
            }
            else
            {
                BOOL_MESH_DEBUG_PRINTLN("normal case, adding new arrow to todo");
                todo.push_front(new_arrow);
            }
        } break;
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


void BooleanMeshOps::debug_export_difference_mesh(HE_FaceHandle originalFace, IntersectionPoint& connectingPoint, TriangleIntersection& triangleIntersection, HE_FaceHandle newFace)
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








/*!
exports a mesh with an arrow-like face pointing at connectingPoint (in the same direction is newFace) and omitting the previous face in connectingPoint
also adds the face for which we are currently computing the fracture line part
*/

void BooleanMeshOps::debug_export_problem(HE_FaceHandle triangle1, std::shared_ptr<TriangleIntersection> triangleIntersection, HE_FaceHandle newFace, IntersectionPoint& connectingPoint)
{ // problem!
    BOOL_MESH_DEBUG_PRINTLN("ERROR! : no intersection!!! type = " << (triangleIntersection->intersectionType));
    connectingPoint.debugOutput();
    BOOL_MESH_DEBUG_PRINTLN("saving mesh with problem to problem.stl");
    Point3 normal = newFace.norm();
    while (normal.testLength(10000))
        normal*=2;
    while (!normal.testLength(20000))
        normal/=2;
    HE_Mesh& heMesh = *newFace.m;
    {
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
    }
    {
        int vi = heMesh.vertices.size();
        int ei = heMesh.edges.size();
        int fi = heMesh.faces.size();
        heMesh.vertices.emplace_back(triangle1.p0(), ei+0);
        heMesh.vertices.emplace_back(triangle1.p1(), ei+1);
        heMesh.vertices.emplace_back(triangle1.p2(), ei+2);
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
    }
    {
        HE_FaceHandle oldFace = connectingPoint.edge.face();
        heMesh.faces[oldFace.idx].edge_idx[0] = 0;
        heMesh.faces[oldFace.idx].edge_idx[1] = 0;
        heMesh.faces[oldFace.idx].edge_idx[2] = 0;
    }

    saveMeshToFile<HE_Mesh, HE_VertexHandle, HE_FaceHandle>(heMesh, "problem.stl");
    BOOL_MESH_DEBUG_PRINTLN("saving done..");
    exit(0);
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

    BooleanMeshOps subtract(heMesh, other, BoolOpType::DIFFERENCE);

    int f;
    std::shared_ptr<TriangleIntersection> triangleIntersection;
    for (f = 0; f < heMesh.faces.size(); f++)
    {
        HE_FaceHandle face(heMesh, f);
        triangleIntersection = subtract.getIntersection(face, otherFace);
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

    subtract.completeFractureLine(otherFace, intersectingFace, *triangleIntersection, face2fractures);

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


#include "modelFile/modelFile.h"

void BooleanMeshOps::test_completeFractureLine()
{
    FMatrix3x3 transformation; // identity matrix
    bool success;

    FVMesh keep_fv(nullptr);
    success = loadModelSTL(&keep_fv, "/home/Dropbox/3D_models/Atlas_keep.stl", transformation);
    if (!success)
    {
        std::cerr << "loading model failed" << std::endl;
        exit(0);
    }
    HE_Mesh keep(keep_fv);


    FVMesh subtracted_fv(nullptr);
    success = loadModelSTL(&subtracted_fv, "/home/Dropbox/3D_models/Atlas_subtracted.stl", transformation);
    if (!success)
    {
        std::cerr << "loading model failed" << std::endl;
        exit(0);
    }
    HE_Mesh subtracted(subtracted_fv);

    test_completeFractureLine(keep, subtracted);

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

    test_completeFractureLine(heMesh, other);
}

void BooleanMeshOps::test_completeFractureLine(HE_Mesh& heMesh, HE_Mesh& other)
{
    BooleanMeshOps subtract(heMesh, other, BoolOpType::DIFFERENCE);

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
            triangleIntersection = subtract.getIntersection(face, otherFace);
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
    subtract.completeFractureLine(otherFace, intersectingFace, *triangleIntersection, face2fractures);

//    FractureLinePart result;
//    getFacetFractureLinePart(otherFace, intersectingFace, triangleIntersection, result);

    std::cerr << std::endl;
    std::cerr << "=============================================\n" << std::endl;
    std::cerr << std::endl;

    //BOOL_MESH_DEBUG_PRINTLN("------");
    //result.debugOutputNodePoints();
    //BOOL_MESH_DEBUG_PRINTLN("------");


}


void BooleanMeshOps::test_subtract()
{
    std::remove("CRAP.csv");
    std::remove("WHOLE.csv");
    std::remove("WHOLE_keep.csv");
    std::remove("WHOLE_subtracted.csv");

    std::cerr << "=============================================\n" << std::endl;
    FMatrix3x3 transformation; // identity matrix
    bool success;

    FVMesh keep_fv(nullptr);
    success = loadModelSTL(&keep_fv, "/home/tim/Dropbox/3D_models/Atlas_keep.stl", transformation);
    if (!success)
    {
        std::cerr << "loading keep model failed" << std::endl;
        exit(0);
    }
    HE_Mesh keep(keep_fv);


    FVMesh subtracted_fv(nullptr);
    success = loadModelSTL(&subtracted_fv, "/home/tim/Dropbox/3D_models/Atlas_subtracted.stl", transformation);
    if (!success)
    {
        std::cerr << "loading subtracted model failed" << std::endl;
        exit(0);
    }
    HE_Mesh subtracted(subtracted_fv);

    test_subtract(keep, subtracted);
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

    test_subtract(heMesh, other);
}
void BooleanMeshOps::test_subtract(HE_Mesh& heMesh, HE_Mesh& other)
{
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
