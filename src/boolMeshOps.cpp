#include "boolMeshOps.h"

#include <memory> // sharred_ptr

#include <unordered_set> //==hash_set

#include "modelFile/modelFile.h" // PrintObject

#include <stdio.h>

#include <cmath> // acos

#include "settings.h" // MAX_EDGES_PER_VERTEX

#include "MACROS.h" // debug

#include <fstream> // write to file (debug)

#include <boost/iterator/transform_iterator.hpp>


namespace boolOps {

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
/*
void getFaceEdgeSegments(bool aboveIntersection, std::vector<FractureLinePart>& fracturesOnFace, std::vector<std::pair<IntersectionPoint, IntersectionPoint>>& segments)
{
    if (fracturesOnFace.size() == 0) return;

    // TODO: check whether fracture is simple (1 or 2 segments?) and do it the simple way in that case

    HE_FaceHandle face = fracturesOnFace[0].face;

    #if BOOL_MESH_OPS_DEBUG == 1
        // check fractures on bugs!
        // : check whether all fractures lie on same face
        // : check whether all fracture lines are aligned correctly
        // : check whether a fracture line lies on an edge (warning)
        for (FractureLinePart p : fracturesOnFace)
        {
            if (p.face != face)
            {
                BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! getFaceEdgeSegments called for different faces!");
            }

            for (Arrow* a : p.fracture.arrows)
            {
                Arrow* prev = a->from->last_in;
                if (prev != nullptr)
                {
                    auto direction = [](Arrow* prev) { return true; }; // TODO   prev->data.isDirectionOfInnerPartOfMainTriangle(); };

                    if (direction(a) != direction(prev))
                    {
                        BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! subsequent intersections not directed the same way!");
                        BOOL_MESH_OPS_DEBUG_SHOW(direction(a));
                        BOOL_MESH_OPS_DEBUG_SHOW(direction(prev));
                    }
                }
                if (a->from->data.compareSource(a->to->data))
                    BOOL_MESH_OPS_DEBUG_PRINTLN("WARNING! intersection segment lies on the edge of a face!");
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
            else BOOL_MESH_OPS_DEBUG_PRINTLN("WARNING! didn't recognize edge to lie on face!!");
        } break;
        case IntersectionPointType::VERTEX:
            if      (dp.p.vertex == face.v0()) v0 = new DirectedPoint(dp);
            else if (dp.p.vertex == face.v1()) v1 = new DirectedPoint(dp);
            else if (dp.p.vertex == face.v2()) v2 = new DirectedPoint(dp);
            else BOOL_MESH_OPS_DEBUG_PRINTLN("WARNING! didn't recognize vertex to lie on face!!");
        break;
        }
    };

    for (FractureLinePart p : fracturesOnFace)
    {
        for (Arrow* a : p.endPoints)
        {
            processArrow(a, true);
        }
        //processArrow(p.start, false);
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
*/



void BooleanMeshOps::debug_csv(std::unordered_map<HE_FaceHandle, FractureLinePart> & face2fractures, std::string filename)
{
    DEBUG_DO(
        std::ofstream csv;
        csv.open("CRAP.csv", std::fstream::out | std::fstream::app);
        csv << "0,0,0" << std::endl;
        csv.close();
    );

    std::ofstream csv;
    csv.open(filename, std::fstream::out | std::fstream::app);

//    const Point offset(100, 100, 100);
    const Point offset(0, 0, 0);

    Point offset_now(0,0,0);
    Point p(0,0,0);
    for (auto mapping: face2fractures)
    {

        for (auto arrow : mapping.second.fracture.arrows)
        {
            p = arrow->to->data + offset_now;
            csv << p.x <<", " << p.y << ", " << p.z << std::endl;
            p = arrow->from->data + offset_now;
            csv << p.x <<", " << p.y << ", " << p.z << std::endl;
        }



        offset_now += offset;


    }
    csv << p.x <<", " << p.y << ", " << p.z << std::endl;
    csv << "5000,0,30000" << std::endl;
    csv.close();
};










int vertex_meld_distance = MELD_DISTANCE;

static inline uint32_t pointHash_simple(const Point3& p)
{
    return ((p.x + vertex_meld_distance/2) / vertex_meld_distance) ^ (((p.y + vertex_meld_distance/2) / vertex_meld_distance) << 10) ^ (((p.z + vertex_meld_distance/2) / vertex_meld_distance) << 20);
}

static inline uint32_t pointHash(const Point3& point)
{
    Point p = p/50;
    return pointHash_simple(p);
}

struct PointHasher{

    uint32_t operator()(const Point3& p) const { return pointHash(p); };

};



static inline uint32_t pointHash(const Point3& point, const Point3& relativeHash)
{
    Point p = p/50 + relativeHash;
    return pointHash_simple(p);
}

static inline Point3 getRealtiveForHash(const Point3& p, const Point3& relativeHash)
{
    return p + relativeHash*50;
}

struct IntersectionPointHasher {
    uint32_t operator()(const std::pair<TriangleIntersection,bool>& p) const
    {
        if (p.second)
            return pointHash(p.first.from->p_const());
        else
            return pointHash(p.first.to->p_const());
    };
    uint32_t operator()(const IntersectionPoint& p) const
    {
        return pointHash(p.p_const());
    };
};




std::shared_ptr<TriangleIntersection> BooleanMeshOps::getIntersection(HE_FaceHandle tri1, HE_FaceHandle tri2)
{
    std::shared_ptr<TriangleIntersection> ret = TriangleIntersectionComputation::intersect(tri1, tri2);
    return ret;
}


void BooleanMeshOps::subtract(HE_Mesh& keep, HE_Mesh& subtracted, HE_Mesh& result)
{

    BooleanMeshOps subtract(keep, subtracted, BoolOpType::DIFFERENCE);
    return subtract.perform(result);
}


void BooleanMeshOps::createIntersectionSegmentSoup(Face2Soup& fracture_soup_keep, Face2Soup& fracture_soup_subtracted, AABB_Tree<HE_FaceHandle>& keep_aabb)
{
    BOOL_MESH_OPS_DEBUG_PRINTLN("=====================================");
    BOOL_MESH_OPS_DEBUG_PRINTLN("=== createIntersectionSegmentSoup ===");
    long totalTriTriIntersectionComputations = 0;
    long totalTriTriIntersections = 0;

    for (int f = 0 ; f < subtracted.faces.size() ; f++)
    {
        HE_FaceHandle face_subtracted(subtracted, f);

        BoundingBox tribbox = face_subtracted.bbox();
        std::vector<HE_FaceHandle> intersectingBboxFaces;
        keep_aabb.getIntersections(tribbox, intersectingBboxFaces);
        std::vector<std::pair<BoundingBox, HE_FaceHandle>> intersectingBboxFaces2;
        keep_aabb.getIntersections(tribbox, intersectingBboxFaces2);

        totalTriTriIntersectionComputations += intersectingBboxFaces.size();

        for (const HE_FaceHandle intersectingBboxFace : intersectingBboxFaces)
        {

            HE_FaceHandle tri1 = HE_FaceHandle(intersectingBboxFace);
            HE_FaceHandle& tri2 = face_subtracted;

            std::shared_ptr<TriangleIntersection> triangleIntersection = getIntersection(tri1, tri2);

    //            if (!triangleIntersection->from || !triangleIntersection->to)
            if (triangleIntersection->intersectionType != IntersectionType::LINE_SEGMENT)
                continue; // they don't intersect!
            else
            {
                hashMapInsert(fracture_soup_keep, tri1, tri2, *triangleIntersection);
                hashMapInsert(fracture_soup_subtracted, tri2, tri1, *triangleIntersection);
            }

        }
    }
    BOOL_MESH_OPS_DEBUG_SHOW(totalTriTriIntersectionComputations);
    BOOL_MESH_OPS_DEBUG_SHOW(totalTriTriIntersections);
    BOOL_MESH_OPS_DEBUG_PRINTLN("average # intersection computations per triangle: "<< float(totalTriTriIntersectionComputations) / subtracted.faces.size());
    BOOL_MESH_OPS_DEBUG_PRINTLN("\t( brute force would be: "<<keep.faces.size() * subtracted.faces.size() << ")");
    BOOL_MESH_OPS_DEBUG_PRINTLN("average # intersections per triangle: "<< float(totalTriTriIntersections) / subtracted.faces.size());
    BOOL_MESH_OPS_DEBUG_PRINTLN("=====================================");

};



void BooleanMeshOps::hashMapInsert(std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>>& fractures, HE_FaceHandle tri1, HE_FaceHandle tri2, TriangleIntersection& triangleIntersection)
{
    typedef std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>> Map;

    std::unordered_map<HE_FaceHandle, TriangleIntersection> fracture;
    std::pair<Map::iterator, bool> inserted = fractures.emplace(tri1, fracture);
    if (inserted.second)
    {
        inserted.first->second.emplace(tri2, triangleIntersection);
    } else
    {
        inserted.first->second.emplace(tri2, triangleIntersection);
    }
}













struct NeverEqual
{
    template<typename T>
    bool operator()(const T& p1, const T& p2) const { return false; };
};


void findCloseIntersectionPoints(std::unordered_map<Point3, std::pair<TriangleIntersection,bool>, PointHasher, NeverEqual>& point2segment, Point3& p, std::vector<std::pair<TriangleIntersection,bool>>& ret)
{
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++)
            {
                int i = point2segment.bucket(getRealtiveForHash(p, Point3(x,y,z)));
                for ( auto local_it = point2segment.begin(i); local_it!= point2segment.end(i); ++local_it )
                    ret.push_back(local_it->second);
            }
    // fallback strategy: add all points!
    if (ret.empty())
    {
        for ( auto it = point2segment.begin(); it != point2segment.end(); ++it )
            ret.push_back(it->second);
    }
};








typedef std::unordered_map<IntersectionPoint , Node*, IntersectionPointHasher> Point2fracNode;


void addVertexPointsToFracture(
    std::unordered_map<HE_FaceHandle, TriangleIntersection>& segments,
    std::unordered_map<IntersectionPoint , Node*, IntersectionPointHasher>& point2fracNode,
    FractureLinePart& frac
    )
{
    // add all vertex points to graph (without connections)
    for (std::pair<HE_FaceHandle, TriangleIntersection> tri_n_line : segments)
    {
        HE_FaceHandle tri_subtracted = tri_n_line.first;
        TriangleIntersection& line = tri_n_line.second;

        auto handleFromOrTo = [&](bool from)
        {
            IntersectionPoint intersectionPoint = (from)? *line.from : *line.to;
            if (intersectionPoint.type == IntersectionPointType::VERTEX && point2fracNode.find(intersectionPoint) == point2fracNode.end())
            {
                Point zero(0,0,0);
                Node* node = frac.fracture.addNode(zero);
                HE_VertexHandle v = intersectionPoint.vertex;

                int n_connected_segments = 0;

                HE_EdgeHandle out_edge = v.someEdge();
                do
                {
                    HE_FaceHandle connected_face = out_edge.face();
                    auto connected_segment_found = segments.find(connected_face);
                    if (connected_segment_found != segments.end())
                    {
                        TriangleIntersection& connected_segment = connected_segment_found->second;
                        IntersectionPoint* closest_to_vert = nullptr;
                        { // get closest point
                            if ( (connected_segment.from->p() - v.p()).vSize() < (connected_segment.to->p() - v.p()).vSize() )
                                closest_to_vert = &*connected_segment.from;
                            else
                                closest_to_vert = &*connected_segment.to;
                        }
                        point2fracNode.emplace(*closest_to_vert, node);
                        node->data += closest_to_vert->p();
                        n_connected_segments++;
                    }
                    out_edge = out_edge.converse().next();
                } while (out_edge != v.someEdge());

                node->data /= n_connected_segments; // average over all connected IntersectionPoints

            }
        };
        handleFromOrTo(true);
        handleFromOrTo(false);
    }
};


void addUnhandledNonVertexPointsToFracture (
    std::unordered_map<HE_FaceHandle, TriangleIntersection>& segments,
    std::unordered_map<IntersectionPoint , Node*, IntersectionPointHasher>& point2fracNode,
    FractureLinePart& frac,
    HE_FaceHandle tri_keep
    )
{
    // add all unhandled non-vertex points (IntersectionPointType::NEW) to graph nodes
    for (std::pair<HE_FaceHandle, TriangleIntersection> tri_n_line : segments)
    {
        HE_FaceHandle tri_subtracted = tri_n_line.first;
        TriangleIntersection& line = tri_n_line.second;

        auto handleFromOrTo = [&](bool from)
        {
            IntersectionPoint intersectionPoint = (from)? *line.from : *line.to;
            if (intersectionPoint.type == IntersectionPointType::NEW && point2fracNode.find(intersectionPoint) == point2fracNode.end())
            {
                Node* node = frac.fracture.addNode(intersectionPoint.p());
                point2fracNode.emplace(intersectionPoint, node);

//BOOL_MESH_OPS_DEBUG_PRINTLN("adding new node for ");
//intersectionPoint.debugOutput();
//
                HE_EdgeHandle e = intersectionPoint.edge;

                if (e.face() == tri_keep) return;

                int n_connected_segments = 1;

                HE_FaceHandle connected_face = e.converse().face();
                auto connected_segment_found = segments.find(connected_face);
                if (connected_segment_found != segments.end())
                {
                    TriangleIntersection& connected_segment = connected_segment_found->second;
                    IntersectionPoint* closest_to_intersectionPoint = nullptr;
                    { // get closest point
                        if ( (connected_segment.from->p() - intersectionPoint.p()).vSize() < (connected_segment.to->p() - intersectionPoint.p()).vSize() )
                            closest_to_intersectionPoint = &*connected_segment.from;
                        else
                            closest_to_intersectionPoint = &*connected_segment.to;
                    }
//BOOL_MESH_OPS_DEBUG_PRINTLN("and for ");
//closest_to_intersectionPoint->debugOutput();
                    point2fracNode.emplace(*closest_to_intersectionPoint, node);
                    node->data += closest_to_intersectionPoint->p();
                    n_connected_segments++; // n_connected_segments = 2;
                } else
                {
                    BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! couldn't find intersectionintersection of connected face!  >> search nearby nodes?"); // TODO use findCloseIntersectionPoints

                }
                node->data /= n_connected_segments; // average over all connected IntersectionPoints

            }
        };
        handleFromOrTo(true);
        handleFromOrTo(false);
    }
};

void connectNodesInFracture (
    std::unordered_map<HE_FaceHandle, TriangleIntersection>& segments,
    std::unordered_map<IntersectionPoint , Node*, IntersectionPointHasher>& point2fracNode,
    FractureLinePart& frac,
    HE_FaceHandle tri_keep
    )
{
    // connect all nodes (and add nodes for non-vertex IntersectionPoints)
    for (std::pair<HE_FaceHandle, TriangleIntersection> tri_n_line : segments)
    {
        HE_FaceHandle tri_subtracted = tri_n_line.first;
        TriangleIntersection& line = tri_n_line.second;

        auto getNode = [&](bool from)
        {
            IntersectionPoint& intersectionPoint = (from)? *line.from : *line.to;

            bool averageGraphNodePosition = false;
            Point2fracNode::iterator point_n_fracNode_found = point2fracNode.find(intersectionPoint);
            Node* ret = nullptr;
            if (point_n_fracNode_found != point2fracNode.end())
            {
                ret = point_n_fracNode_found->second;
            } else
            {
                BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! (?) couldn't find node in hashmap!!!!!!!!! : ");
                intersectionPoint.debugOutput();
                //std::exit(0);
                ret = frac.fracture.addNode(intersectionPoint.p());
                averageGraphNodePosition = true;
            }
            return ret;
        };
        Node* from_node = getNode(true);
        Node* to_node = getNode(false);

        frac.fracture.connect(*from_node, *to_node, line);

    }
};


void getFace2fractures(std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>>& fracture_soup_keep, std::unordered_map<HE_FaceHandle, FractureLinePart>& face2fractures_keep)
{
    for (std::pair<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>> soup: fracture_soup_keep)
    {
        typedef std::unordered_map<HE_FaceHandle, std::unordered_map<HE_FaceHandle, TriangleIntersection>> Map;
        typedef std::unordered_map<HE_FaceHandle, FractureLinePart> FracMap;

        HE_FaceHandle tri_keep = soup.first;
        std::unordered_map<HE_FaceHandle, TriangleIntersection> segments = soup.second;


        // std::pair<TriangleIntersection,bool> :: intersection segment and a bool saying whether the point is the [from] of the segment
        typedef std::unordered_map<Point3, std::pair<TriangleIntersection,bool>, PointHasher, NeverEqual> Point2segment;
        Point2segment point2segment;

        auto getIntersectionPoint = [](std::pair<TriangleIntersection,bool>& p)
        {
            if (p.second)
                return *p.first.from;
            else
                return *p.first.to;
        };

        { // make mapping from location to line segment
            for (std::pair<HE_FaceHandle, TriangleIntersection> tri_n_line : segments)
            {
                HE_FaceHandle tri_subtracted = tri_n_line.first;
                TriangleIntersection& line = tri_n_line.second;

                std::pair<Point2segment::iterator, bool> emplaced_f = point2segment.emplace(line.from->p(), std::pair<TriangleIntersection, bool>(line, true));
                std::pair<Point2segment::iterator, bool> emplaced_t = point2segment.emplace(line.to->p(), std::pair<TriangleIntersection, bool>(line, false));
                if (!emplaced_f.second)
                {
                    BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! couldn't insert point in point2segment, cause a point already exists there!");
                    BOOL_MESH_OPS_DEBUG_PRINTLN("new : " << line.from->p()<< " existing: "<< emplaced_f.first->first);

                }
                if (!emplaced_t.second)
                {
                    BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! couldn't insert point in point2segment, cause a point already exists there!");
                    BOOL_MESH_OPS_DEBUG_PRINTLN("new : " << line.to->p()<< " existing: "<< emplaced_t.first->first);
                }
            }
        }




        FractureLinePart frac_ff(tri_keep);
        std::pair<FracMap::iterator, bool> inserted = face2fractures_keep.emplace(tri_keep, frac_ff);
        FractureLinePart& frac  = inserted.first->second;



        // bool is whether we take the [from] point of the segment
        std::unordered_set<std::pair<TriangleIntersection, bool>, IntersectionPointHasher> already_connected_points;

        Point2fracNode point2fracNode;

        { // add all intersections to graph

            addVertexPointsToFracture(segments, point2fracNode, frac);

            addUnhandledNonVertexPointsToFracture(segments, point2fracNode, frac, tri_keep);

            connectNodesInFracture(segments, point2fracNode, frac, tri_keep);
        }




    }
};




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

BOOL_MESH_OPS_DEBUG_PRINTLN("constructing AABB-tree...");
    AABB_Tree<HE_FaceHandle> keep_aabb(keep_faces.begin(), keep_faces.end());
BOOL_MESH_OPS_DEBUG_PRINTLN("finished constructing AABB-tree");

    Face2Soup fracture_soup_keep;
    Face2Soup fracture_soup_subtracted;

    createIntersectionSegmentSoup(fracture_soup_keep, fracture_soup_subtracted, keep_aabb);


    std::unordered_map<HE_FaceHandle, FractureLinePart> face2fractures_keep;
    std::unordered_map<HE_FaceHandle, FractureLinePart> face2fractures_subtracted;


    getFace2fractures(fracture_soup_keep, face2fractures_keep);
    getFace2fractures(fracture_soup_subtracted, face2fractures_subtracted);

    auto removeConnection = [](Arrow* a, Graph<Point, TriangleIntersection>& fracture)
    {
        Node* from = a->from;
        Node* to = a->to;
        fracture.disconnect(a);
        fracture.removeNodeIfLonely(from);
        fracture.removeNodeIfLonely(to);
    };
    auto removeEdgeTriangleIntersectionsTouchingMainFace = [](HE_FaceHandle face, FractureLinePart& frac)
    {
        BOOL_MESH_OPS_DEBUG_PRINTLN("TODO! find duplicate edges; remove both if different direction; remove only one if same direction");
    };
    auto checkEdgeTriangleIntersectionsConnectedToCoplanarFaces = [](HE_FaceHandle face, FractureLinePart& frac)
    {
        BOOL_MESH_OPS_DEBUG_PRINTLN("TODO! find edges with edgeOfTriangle1TouchingTriangle2 or edgeOfTriangle2TouchingTriangle1 \n"
            << ">> check whether it should be removed ");
    };
//    auto removeSuperfluousMainFaceEdgeTriangleIntersections = [this](HE_FaceHandle face, FractureLinePart& frac)
//    {
//        BOOL_MESH_OPS_DEBUG_PRINTLN("TODO! do this? (removeSuperfluousMainFaceEdgeTriangleIntersections)");
//
//    };
    for (std::pair<HE_FaceHandle, FractureLinePart> face_n_frac : face2fractures_keep)
    {
        HE_FaceHandle face = face_n_frac.first;
        FractureLinePart& frac = face_n_frac.second;

        removeEdgeTriangleIntersectionsTouchingMainFace(face, frac);
        checkEdgeTriangleIntersectionsConnectedToCoplanarFaces(face, frac);
//        removeSuperfluousMainFaceEdgeTriangleIntersections(face, frac);
    }
    for (std::pair<HE_FaceHandle, FractureLinePart> face_n_frac : face2fractures_subtracted)
    {
        HE_FaceHandle face = face_n_frac.first;
        FractureLinePart& frac = face_n_frac.second;

        removeEdgeTriangleIntersectionsTouchingMainFace(face, frac);
        checkEdgeTriangleIntersectionsConnectedToCoplanarFaces(face, frac);
//        removeSuperfluousMainFaceEdgeTriangleIntersections(face, frac);
    }

    auto removeEmptyFractureLines = [](std::unordered_map<HE_FaceHandle, FractureLinePart>& face2fractures) {};

    removeEmptyFractureLines(face2fractures_keep);
    removeEmptyFractureLines(face2fractures_subtracted);

debug_csv(face2fractures_keep, "WHOLE_keep.csv");
debug_csv(face2fractures_subtracted, "WHOLE_subtracted.csv");


//debug_csv(face2fractures, "WHOLE.csv");

BOOL_MESH_OPS_DEBUG_SHOW(keep.faces.size());

}














void BooleanMeshOps::debug_export_difference_mesh(HE_FaceHandle originalFace, IntersectionPoint& connectingPoint, TriangleIntersection& triangleIntersection, HE_FaceHandle newFace)
{

    BOOL_MESH_OPS_DEBUG_PRINTLN("WARNING! large difference between endpoint of first triangle intersection and start point of second! :");
    BOOL_MESH_OPS_DEBUG_SHOW((connectingPoint.p() - triangleIntersection.from->p()).vSize());
    //BOOL_MESH_OPS_DEBUG_DO(
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
    BOOL_MESH_OPS_DEBUG_PRINTLN("ERROR! : no intersection!!! type = " << (triangleIntersection->intersectionType));
    connectingPoint.debugOutput();
    BOOL_MESH_OPS_DEBUG_PRINTLN("saving mesh with problem to problem.stl");
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
    BOOL_MESH_OPS_DEBUG_PRINTLN("saving done..");
    exit(0);
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


} // namespace boolOps
