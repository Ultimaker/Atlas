#include "supportClassification.h"
#include "utils/intpoint.h"
#include <iostream> /* cerr */
#include <math.h> /* isfinite */
#include <cassert>
#include <fstream> // write to file

#include "utils/logoutput.h" // logError

#include <stdlib.h>

#include <algorithm> // std::binary_search

#include "mesh/HalfEdgeMesh.h"


#include "MACROS.h" // debug
// enable/disable debug output
#define ADV_SUP_DEBUG 0

#define ADV_SUP_DEBUG_SHOW(x) DEBUG_SHOW(x)
#define ADV_SUP_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#if ADV_SUP_DEBUG == 1
#  define ADV_SUP_DEBUG_DO(x) DEBUG_DO(x)
#else
#  define ADV_SUP_DEBUG_DO(x)
#endif

namespace atlas {

SupportChecker SupportChecker::getSupportRequireds(FVMesh& mesh, double maxAngle)
{

    SupportChecker supporter(mesh, maxAngle);
    ADV_SUP_DEBUG_DO(
        std::cerr << "-----------------------\n--getSupportRequireds--\n-----------------------" << std::endl;
        std::cerr << "cosMaxAngle = " << supporter.cosMaxAngle << std::endl;
        std::cerr << "cosMaxAngleNormal = " << supporter.cosMaxAngleNormal << std::endl;
    )

    for (int f = 0 ; f < supporter.mesh.faces.size() ; f++)
    {
        supporter.faceIsBad[f] = supporter.faceNeedsSupport(supporter.mesh, f);
    }

    for (int e = 0 ; e < supporter.mesh.edges.size() ; e++)
    {
        bool bad = supporter.edgeNeedsSupport(supporter.mesh, e);
        supporter.edgeIsBad[e] = bad;
        if (bad)
        {
            supporter.edgeIsBad[supporter.mesh.edges[e].converse_edge_idx] = bad;
        }
    }

    for (int v = 0 ; v < supporter.mesh.vertices.size() ; v++)
    {
        supporter.vertexIsBad[v] = supporter.vertexNeedsSupport(supporter.mesh, v);
    }

    ADV_SUP_DEBUG_DO( std::cerr << "------------------------\n--END SupportRequireds--\n------------------------" << std::endl; )
    return supporter;
}

bool SupportChecker::faceNeedsSupport(HE_Mesh& mesh, int face_idx)
{
    Point3 normal = mesh.getNormal(face_idx);

    faceNormals[face_idx] = normal;

    double cosAngle = double(normal.z) / double(normal.vSize()); // fabs


    if (cosAngle < cosMaxAngleNormal) return true;
    else return false;
}

bool debug_support_edges_only = false;

// supposes all faces have already been checked
bool SupportChecker::edgeNeedsSupport(HE_Mesh& mesh, int edge_idx)
{
    ADV_SUP_DEBUG_DO( std::cerr << "edge " << edge_idx << " : "; )

    HE_EdgeHandle edge(mesh, edge_idx);
    HE_EdgeHandle backEdge(edge); edge.converse();

//    HE_Edge& edge = mesh.edges[edge_idx];
//    HE_Edge& backEdge = mesh.edges[edge.converse_edge_idx];

    if (edgeIsBad[backEdge.idx])
    {
        ADV_SUP_DEBUG_DO( std::cerr << " converse is bad" << std::endl; )
        return true;
    }

    bool bad1 = faceIsBad[edge.face().idx];
    bool bad2 = faceIsBad[backEdge.face().idx];

    Point3 face_1_normal = faceNormals[edge.face().idx];
    Point3 face_2_normal = faceNormals[backEdge.face().idx];


    if (bad1 != bad2) // xor : one bad one not... the edge of a bad area is always bad
    {
        Point3 good_face_normal = (bad1)? face_2_normal : face_1_normal;
        ADV_SUP_DEBUG_DO( std::cerr << " on one bad face" << std::endl;)
        if (edgeOnBoundaryNotBadWhenFullySupported && good_face_normal.z >= 0)
            return false;
        else return true;
    }



    if (!bad1 && !bad2) // == !(bad1 && bad2) , since (bad1 != bad2) is already checked above
    { // check if angle with Z-axis is great enough to require support
        Point3 vect = edge.p1() - edge.p0();
        double absCosAngle = fabs( (double)vect.z / (double)vect.vSize() );


        ADV_SUP_DEBUG_DO( std::cerr << " (absCosAngle = " << absCosAngle << ")" << std::endl; )

        if (absCosAngle > cosMaxAngle)
        {
            ADV_SUP_DEBUG_DO(                 if (debug_support_edges_only) std::cerr << " absCosAngle > cosMaxAngle ; vect.z = " <<vect.z << " ; vect.vSize() = " << vect.vSize() << std::endl;)
            return false;
        }
    }


    { // check if edge is top edge (normal pointing up)

        double zNormal = double(face_1_normal.z) / double(face_1_normal.vSize()) + double(face_2_normal.z) / double(face_2_normal.vSize()); // (unnormalized)


        if (zNormal > 0)
        {
            ADV_SUP_DEBUG_DO(                 if (debug_support_edges_only) std::cerr << "edge is top edge" << std::endl; )
            return false; // edge is a top edge
        }
    }

    return edgeIsBelowFaces(mesh, edge);


}

bool SupportChecker::edgeIsBelowFaces(HE_Mesh& mesh, HE_EdgeHandle& edge)
{
    Point3 a = edge.p0();
    Point3 dac =  edge.p1() - a;
    if (dac.x ==0 && dac.y ==0)
    {
        ADV_SUP_DEBUG_DO(                 if (debug_support_edges_only) std::cerr << "edge is vertical" << std::endl; )
        return false;
    }
    double denom = 1. / (dac.x*dac.x + dac.y*dac.y);


    if (!std::isfinite(denom)) // then |dac.x| == |dac.y|
    {
        return edgeIsBelowFacesDiagonal(mesh, edge, a, dac);
    } else
    {
        return edgeIsBelowFacesNonDiagonal(mesh, edge, a, dac, denom);
    }
}

double distanceWhichIsBasicallyZero = .01; // cos of angle which still counts as vertical. 0 < a << 1

bool SupportChecker::edgeIsBelowFacesDiagonal(HE_Mesh& mesh, HE_EdgeHandle& edge, Point3& a, Point3& dac)
{
    assert(dac.x == dac.y || dac.x == -dac.y);

    ADV_SUP_DEBUG_DO( std::cerr << "edge is diagonal" << std::endl; )

    short sign = (dac.x == -dac.y)? -1 : 1;
    double denom = 1. / (2 * dac.x);

    // first face
    Point3 b = edge.next().p1(); // mesh.getTo(*mesh.getNext(edge))->p;
    Point3 dab =  b - a;
    if (!edgeIsBelowSingleFaceDiagonal(dab, dac, denom, sign)) return false;

    // second face
    Point3 b2 = edge.converse().next().p1(); //mesh.getTo(*mesh.getNext(*mesh.getConverse(edge)))->p;
    Point3 dab2 =  b2 - a;
    if (!edgeIsBelowSingleFaceDiagonal(dab2, dac, denom, sign)) return false;

    ADV_SUP_DEBUG_DO( std::cerr << "edge lower than both faces (diagonal)" << std::endl; )
    return true; // edge is 'lower' than both faces

}

int64_t vSize(int32_t x, int32_t y) // TODO: move / remove!
{
int64_t vSize2 = x*x +y*y;
return sqrt(vSize2);
}

bool SupportChecker::edgeIsBelowSingleFaceDiagonal(Point3& dab, Point3& dac, double denom, short sign)
{

    double projectedZ_dab = dac.z * (dab.x + sign * dab.y) * denom;

    if (dab.z < projectedZ_dab + distanceWhichIsBasicallyZero * vSize(dab.x, dab.y)) // instead of dab.z/size < projZ_dab/size + dist
    {
        ADV_SUP_DEBUG_DO(
            if (debug_support_edges_only)
            {
                std::cerr << "edge is above a face" << std::endl;
                std::cerr << "denom = " << denom << std::endl;
                std::cerr << "projectedZ_dab = " << projectedZ_dab << std::endl;
                std::cerr << "dab.z  = " << dab.z  << std::endl;
            }
        )
        return false;
    }
    return true;
}



bool SupportChecker::edgeIsBelowFacesNonDiagonal(HE_Mesh& mesh, HE_EdgeHandle& edge, Point3& a, Point3& dac, double denom)
{
    // first face
    Point3 b = edge.next().p1(); // mesh.getTo(*mesh.getNext(edge))->p;
    Point3 dab =  b - a;
    if (!edgeIsBelowSingleFaceNonDiagonal(dab, dac, denom))
    {
        return false;
    }

    // second face
    Point3 b2 = edge.converse().next().p1(); // mesh.getTo(*mesh.getNext(*mesh.getConverse(edge)))->p;
    Point3 dab2 =  b2 - a;
    if (!edgeIsBelowSingleFaceNonDiagonal(dab2, dac, denom))
    {
        return false;
    }
    ADV_SUP_DEBUG_DO(
        std::cerr << "denom = " << denom << std::endl;
        std::cerr << "edge lower than both faces (nonDiagonal)" << std::endl;
    )
    return true; // edge is 'lower' than both faces

}

bool SupportChecker::edgeIsBelowSingleFaceNonDiagonal(Point3& dab, Point3& dac, double denom)
{

    double projectedZ_dab =  dac.z * (double(dac.x*dab.x) + double(dac.y*dab.y)) * denom;

    if (dab.z < projectedZ_dab + distanceWhichIsBasicallyZero * vSize(dab.x, dab.y))
    {
        ADV_SUP_DEBUG_DO(
            if (debug_support_edges_only)
            {
                std::cerr << "edge is above a face" << std::endl;
                std::cerr << "denom = " << denom << std::endl;
                std::cerr << "projectedZ_dab = " << projectedZ_dab << std::endl;
                std::cerr << "dab.z  = " << dab.z  << std::endl;
            }
        )
        return false;
    }
    else return true;
}


bool SupportChecker::vertexNeedsSupport(HE_Mesh& mesh, int vertex_idx)
{
    ADV_SUP_DEBUG_DO( std::cerr << "vertex " << vertex_idx << " : "; )
//    HE_Vertex& vertex = mesh.vertices[vertex_idx];
//    HE_Edge& someEdge = *mesh.getSomeEdge(vertex);
    HE_VertexHandle vertex = HE_VertexHandle(mesh, vertex_idx);
    HE_EdgeHandle someEdge = vertex.someEdge();

    if (faceNormals[someEdge.face().idx].z > 0)
    {
        return false; // vertex can at most be the bottom of a concave dimple
    }

    bool first = true;
    int wtfCounter = 0;

    HE_EdgeHandle* departing_edge = new HE_EdgeHandle(someEdge);
    do
    //for (HE_Edge* departing_edge = &someEdge ; first || departing_edge != &someEdge ; departing_edge = mesh.getNext(*mesh.getConverse(*departing_edge)) )
    {
        if (departing_edge->p1().z < vertex.p().z) return false;
        first = false;
        wtfCounter++;
        if (wtfCounter>200)
        {
            atlas::logError("Cannot find starting edge when moving along all edges connected to a vertex.\n");
            std::exit(EXIT_FAILURE);
        }
        HE_EdgeHandle nxt = departing_edge->converse().next();
        departing_edge->set(nxt);
    } while (*departing_edge != someEdge);
    ADV_SUP_DEBUG_DO( std::cerr << std::endl; )

    ADV_SUP_DEBUG_DO( std::cerr << " vertex is bad" << std::endl; )

    return true;
}

void SupportChecker::testSupportChecker(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    {
        //HE_Mesh mesh(model->meshes[mi]);
        SupportChecker supporter = SupportChecker::getSupportRequireds(model->meshes[mi], .785); // 45/180*M_PI


        std::cerr << "faces badness:" << std::endl;
        for (int f = 0 ; f < supporter.mesh.faces.size() ; f++)
        {
            std::cerr << f << " " << (supporter.faceIsBad[f]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "edges badness:" << std::endl;
        for (int e = 0 ; e < supporter.mesh.edges.size() ; e++)
        {
            std::cerr << e << " " << (supporter.edgeIsBad[e]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "vertices badness:" << std::endl;
        for (int v = 0 ; v < supporter.mesh.vertices.size() ; v++)
        {
            std::cerr << v << " " << (supporter.vertexIsBad[v]? "TRUE" : "f") << std::endl;
        }
        //for (int f = 0; f < mesh.faces.size(); f++)
        //    std::cerr << mesh.faces[f].cosAngle() << std::endl;
    }
    std::cerr << "=============================================\n" << std::endl;

}

SupportChecker::~SupportChecker()
{
    //dtor
}


void SupportChecker::debugGenerateOverhangFVMesh(FVMesh& result)
{
   Point dz(0,0,0);

    for (int f = 0 ; f < mesh.faces.size() ; f++)
    {
        Point d(15000,0,0);
        if (!faceIsBad[f]) continue; // face is not bad at all

        HE_Face& face = mesh.faces[f];

        Point3 p0_top = mesh.getTo(mesh.edges[face.edge_idx[0]])->p + d;
        Point3 p1_top = mesh.getTo(mesh.edges[face.edge_idx[1]])->p + d;
        Point3 p2_top = mesh.getTo(mesh.edges[face.edge_idx[2]])->p + d;

        result.addFace(p0_top, p1_top, p2_top);

    }

    for (int e = 0 ; e < mesh.edges.size() ; e++)
    {
        HE_Edge& edge = mesh.edges[e];
        int f0 = edge.face_idx;
        //int f1 = mesh.getConverse(edge)->face_idx;

        //if (!  edgeIsBad[e] ) continue;
        if (faceIsBad[f0]) continue; // edge is boundary edge or halfedge of bad edge
        if (! edgeIsBad[e] &&
            ! faceIsBad[mesh.edges[edge.converse_edge_idx].face_idx]) continue;

        // face f0 is good and (converse face bad or edge bad)

        Point3 p0_top = mesh.getFrom(edge)->p + dz;
        Point3 p1_top = mesh.getTo(edge)->p + dz;

        Point3 p2 = p0_top + Point(100,100,100);
        //Point3 p3 = p1_top + Point(100,100,100);

        result.addFace(p0_top, p1_top, p2);
        //result.addFace(p0_top, p1_top, p3);


    }
    for (int v = 0 ; v < mesh.vertices.size() ; v++)
    {
        if (!vertexIsBad[v]) continue; // vertex is not bad at all

        spaceType vertexSupportPillarRadius = 1000;
        // pillar is triagular!
        spaceType pillarDx = vertexSupportPillarRadius *0.5;
        spaceType pillarDy = vertexSupportPillarRadius *std::sqrt(.75);

        Point p0_top (-vertexSupportPillarRadius,0,-15000);
        Point p1_top (pillarDx, pillarDy, -15000);
        Point p2_top (pillarDx, -pillarDy, -15000);
        p0_top += mesh.vertices[v].p;
        p1_top += mesh.vertices[v].p;
        p2_top += mesh.vertices[v].p;

        //pillar creation:
        result.addFace(p0_top, p1_top, p2_top); // top



    }

    result.finish();
}











/*
PPP    OO  III  N   N TTTTT     GGG   EEEE N   N
P  P  O  O  I   NN  N   T      G      E    NN  N
PPP   O  O  I   N N N   T      G  GG  EEE  N N N
P     O  O  I   N  NN   T      G   G  E    N  NN
P      OO  III  N   N   T       GGGG  EEEE N   N
*/




SupportPointsGenerator::SupportPointsGenerator(SupportChecker& supportChecker, int32_t vertexOffset, int32_t edgeOffset, int32_t faceOffset, int32_t gridSize)
    : supportChecker(supportChecker)
    , vertexOffset(vertexOffset)
    , edgeOffset(edgeOffset)
    , faceOffset(faceOffset)
    , gridSize(gridSize)
{
    // vertices
    for (int v = 0 ; v < supportChecker.vertexIsBad.size() ; v++)
    {
        if (supportChecker.vertexIsBad[v])
        {
            supportPoints.push_back(AdvSupportPoint(supportChecker.mesh.vertices[v].p - Point3(0, 0, vertexOffset), "vertex", v));
        }
    }

    // edges
    for (int e = 0 ; e < supportChecker.edgeIsBad.size() ; e++)
    {
        if (supportChecker.edgeIsBad[e] && supportChecker.mesh.edges[e].converse_edge_idx < e) // process half-edges only once!
        {
            addSupportPointsEdge(e);
        }
    }

    // faces
    for (int f = 0 ; f < supportChecker.faceIsBad.size() ; f++)
    {
        if (supportChecker.faceIsBad[f])
        {
            addSupportPointsFace(f);
        }
    }
}


void SupportPointsGenerator::addSupportPointsEdge(int edge_idx)
{
    HE_Edge& edge = supportChecker.mesh.edges[edge_idx];
    Point3 minV = supportChecker.mesh.getFrom(edge)->p;
    Point3 maxV = supportChecker.mesh.getTo(edge)->p;

    { // points falling on vertical lines
        if (minV.x > maxV.x) std::swap(minV, maxV);

        int32_t xmax = maxV.x;
        int32_t xmin = minV.x;

        double dzdx = static_cast<double>(maxV.z - minV.z) / static_cast<double>(xmax - xmin);
        double dydx = static_cast<double>(maxV.y - minV.y) / static_cast<double>(xmax - xmin);

        for (int32_t dx = gridSize - xmin % gridSize ; dx < xmax - xmin ; dx += gridSize)
        {
            supportPoints.push_back(AdvSupportPoint(Point3(xmin+dx, minV.y + dx*dydx, minV.z + dx*dzdx - edgeOffset) , "edge", edge_idx) );
        }
    }

    { // points falling on horizontal lines
        if (minV.y > maxV.y) std::swap(minV, maxV);

        int32_t ymax = maxV.y;
        int32_t ymin = minV.y;

        double dzdy = static_cast<double>(maxV.z - minV.z) / static_cast<double>(ymax - ymin);
        double dxdy = static_cast<double>(maxV.x - minV.x) / static_cast<double>(ymax - ymin);

        for (int32_t dy = gridSize - ymin % gridSize ; dy < ymax - ymin ; dy += gridSize)
        {
            supportPoints.push_back(AdvSupportPoint(Point3(minV.x + dy*dxdy, ymin+dy, minV.z + dy*dzdy - edgeOffset) , "edge", edge_idx) );
        }
    }

}

void SupportPointsGenerator::addSupportPointsFace(int face_idx)
{
    HE_Face& face = supportChecker.mesh.faces[face_idx];
    Point3 minXv = supportChecker.mesh.getTo(supportChecker.mesh.edges[face.edge_idx[0]])->p;
    Point3 maxXv = supportChecker.mesh.getTo(supportChecker.mesh.edges[face.edge_idx[1]])->p;
    if (minXv.x > maxXv.x) std::swap(minXv, maxXv);
    Point3 midXv = supportChecker.mesh.getTo(supportChecker.mesh.edges[face.edge_idx[2]])->p;
    if (midXv.x < minXv.x) std::swap (minXv, midXv);
    else if (midXv.x > maxXv.x) std::swap(maxXv, midXv);


    if (maxXv.x - minXv.x == 0) return;

    double dzdxMinMax = static_cast<double>(maxXv.z - minXv.z) / static_cast<double>(maxXv.x - minXv.x);
    double dydxMinMax = static_cast<double>(maxXv.y - minXv.y) / static_cast<double>(maxXv.x - minXv.x);

    double dzdxMinMid = static_cast<double>(midXv.z - minXv.z) / static_cast<double>(midXv.x - minXv.x);
    double dydxMinMid = static_cast<double>(midXv.y - minXv.y) / static_cast<double>(midXv.x - minXv.x);

    double dzdxMidMax = static_cast<double>(maxXv.z - midXv.z) / static_cast<double>(maxXv.x - midXv.x);
    double dydxMidMax = static_cast<double>(maxXv.y - midXv.y) / static_cast<double>(maxXv.x - midXv.x);

    double dzdy = (dzdxMinMid - dzdxMinMax) / (dydxMinMid - dydxMinMax);

    if (!std::isfinite(dzdy))
    {
        dzdy = (dzdxMidMax - dzdxMinMax) / (dydxMidMax - dydxMinMax); // hopefully less rounding problems
        if (!std::isfinite(dzdy)) // if still
        {
            dzdy = (dzdxMidMax - dzdxMidMax) / (dydxMidMax - dzdxMidMax); // hopefully less rounding problems
            if (!std::isfinite(dzdy))
            {
                return; // face has no area! all three sides have the same direction (?)
            }
        }
    }


    if (midXv.x - minXv.x != 0) // if left side is present
    { // generating points on grid for left half of triangle (between the leftmost point and the middle point)
        double dydxLowerY = dydxMinMax;
        double dzdxLowerY = dzdxMinMax;
        double dydxHigherY = dydxMinMid;
        if (dydxLowerY > dydxHigherY)
        {
            std::swap(dydxLowerY, dydxHigherY);
            dzdxLowerY = dzdxMinMid;
        }
        //if (face_idx == 1 )
        ADV_SUP_DEBUG_DO(std::cerr << "!!!!!!> start: " << gridSize - minXv.x % gridSize <<", end: "<<  midXv.x - minXv.x  << std::endl; )

        for (int32_t dx = gridSize - minXv.x % gridSize ; minXv.x + dx <= midXv.x ; dx += gridSize)
        {
            int32_t ymin = minXv.y + dx*dydxLowerY;
            int32_t ymax = minXv.y + dx*dydxHigherY;

            //if (face_idx == 1 )
            ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!> Y start: " << gridSize - ymin % gridSize <<", end: "<<  ymax - ymin  << std::endl; )

            for (int32_t dy = gridSize - ymin % gridSize ; dy <= ymax - ymin ; dy += gridSize)
            {
                Point3 p(minXv.x + dx, ymin+dy, minXv.z + dx * dzdxLowerY + dy * dzdy - faceOffset);
                supportPoints.push_back(AdvSupportPoint(p, "face", face_idx) );
                ADV_SUP_DEBUG_DO( std::cerr << "==========>   added (" << p.x<<", "<<p.y<<", "<<p.z<<")" <<std::endl; )
            }

        }
    }

    if (maxXv.x - midXv.x != 0) // if right side is present
    { // generating points on grid for right half of triangle (between the middle point and the rightmost point)
        double dydxLowerY = dydxMinMax;
        double dydxHigherY = dydxMidMax;
        double dzdxLowerY = dzdxMinMax;
        if (dydxLowerY < dydxHigherY)
        {
            std::swap(dydxLowerY, dydxHigherY);
            dzdxLowerY = dzdxMidMax;
        }

        ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!2> dydxLowerY: " << dydxLowerY <<", dydxHigherY: "<<  dydxHigherY << std::endl; )
        ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!2> minXv.y : " << minXv.y  <<", midXv.y: "<<  midXv.y << std::endl; )

        //if (face_idx == 0 )
        ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!2> start: " << - maxXv.x % gridSize <<", end: "<<   midXv.x - maxXv.x  << std::endl; )
        // from right to left:
        for (int32_t dx = - maxXv.x % gridSize; maxXv.x + dx >= midXv.x ; dx -= gridSize)
        {
            int32_t ymin = maxXv.y + dx*dydxLowerY;
            int32_t ymax = maxXv.y + dx*dydxHigherY;

            //if (face_idx == 0 )
            ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!2> Y start: " << gridSize - ymin % gridSize <<", end: "<<  ymax - ymin  << std::endl; )
            ADV_SUP_DEBUG_DO( std::cerr << "!!!!!!2> ymin: " << ymin <<", ymax: "<<  ymax << std::endl; )
            for (int32_t dy = gridSize - ymin % gridSize ; dy <= ymax - ymin ; dy += gridSize)
            {
                Point3 p(maxXv.x + dx, ymin+dy, maxXv.z + dx * dzdxLowerY + dy * dzdy - faceOffset);
                supportPoints.push_back(AdvSupportPoint(p, "face", face_idx) );
                ADV_SUP_DEBUG_DO( std::cerr << "==========>   added (" << p.x<<", "<<p.y<<", "<<p.z<<")" <<std::endl; )
            }
        }
    }
}



void SupportPointsGenerator::testSupportPointsGenerator(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    {
        Point3 minn = model->meshes[mi].bbox.min;
        for (int p = 0 ; p < model->meshes[mi].vertices.size() ; p++)
        {
            model->meshes[mi].vertices[p].p -= minn;
        }

        //HE_Mesh mesh(model->meshes[mi]);
        SupportChecker supporter = SupportChecker::getSupportRequireds(model->meshes[mi], .1);//.785); // 45/180*M_PI


        std::cerr << "faces badness:" << std::endl;
        for (int f = 0 ; f < supporter.mesh.faces.size() ; f++)
        {
            std::cerr << f << " " << (supporter.faceIsBad[f]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "edges badness:" << std::endl;
        for (int e = 0 ; e < supporter.mesh.edges.size() ; e++)
        {
            std::cerr << e << " " << (supporter.edgeIsBad[e]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "vertices badness:" << std::endl;
        for (int v = 0 ; v < supporter.mesh.vertices.size() ; v++)
        {
            std::cerr << v << " " << (supporter.vertexIsBad[v]? "TRUE" : "f") << std::endl;
        }
        std::cerr << "=============================================\n" << std::endl;

        SupportPointsGenerator pg(supporter, 1, 2, 3, 300);
        std::cerr << "n points generated: " << pg.supportPoints.size() << std::endl;
        std::ofstream out("bs/supportClassification.obj");
        for (int p = 0; p < pg.supportPoints.size() ; p++)
        {
            Point3 pp = pg.supportPoints[p].p;
            // std::cerr << "v "<<pp.x<<" "<<pp.y<<" "<<pp.z<<" # " << pg.supportPoints[p].fromType << std::endl;
            out << "v "<<pp.x*.001<<" "<<pp.y*.001<<" "<<pp.z*.001<<" # " << pg.supportPoints[p].fromType << " " << pg.supportPoints[p].idx << std::endl;
        }
        out.close();

    }
    std::cerr << "=============================================\n" << std::endl;

}




} // namespace atlas
