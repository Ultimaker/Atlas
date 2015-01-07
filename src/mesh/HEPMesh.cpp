#include "HEPMesh.h"
#include "../utils/logoutput.h"


// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif



#include <iostream>



void HEP_Mesh::debugOuputBasicStats(std::ostream& out)
{
    out << "faces: " << faces.size() << std::endl;
    out << "vertices: " << vertices.size() << std::endl;
    out << "halfedges: " << edges.size() << std::endl;

}



void HEP_Mesh::connectEdgesPrevNext(HEP_Edge* prev, HEP_Edge* next) { prev->next_edge = next; } //edges[next].prev_edge = prev; }
void HEP_Mesh::connectEdgesConverse(HEP_Edge* e1, HEP_Edge* e2) { e1->converse_edge = e2; e2->converse_edge = e1; }

HEP_Vertex* HEP_Mesh::createVertex(Point p)
{
    HEP_Vertex vert(p, nullptr);
    vertices.push_back(vert);
    return &vertices[vertices.size()-1];
}

HEP_Edge* HEP_Mesh::createConverse(HEP_Edge* e)
{
    HEP_Vertex* v1 = e->next_edge->from_vert; //edges[e].to_vert;
    HEP_Edge new_e(v1, nullptr);
    edges.push_back(new_e);
    connectEdgesConverse(e, &edges[edges.size()-1]);
    return &edges[edges.size()-1];
}

HEP_Face* HEP_Mesh::createFace(HEP_Vertex* v0, HEP_Vertex* v1, HEP_Vertex* v2)
{
        HEP_Face f;
        faces.push_back(f);
        HEP_Face* fP = &faces[faces.size()-1];

        HEP_Edge e0 (v0, fP);
        HEP_Edge e1 (v1, fP);
        HEP_Edge e2 (v2, fP);
        edges.push_back(e0); HEP_Edge* e0P = &edges[edges.size()-1];
        edges.push_back(e1); HEP_Edge* e1P = &edges[edges.size()-1];
        edges.push_back(e2); HEP_Edge* e2P = &edges[edges.size()-1];

        connectEdgesPrevNext(e0P, e1P);
        connectEdgesPrevNext(e1P, e2P);
        connectEdgesPrevNext(e2P, e0P);

        v0->someEdge = e0P;
        v1->someEdge = e1P;
        v2->someEdge = e2P;

        fP->edge[0] = e0P;
        fP->edge[1] = e1P;
        fP->edge[2] = e2P;

        return &faces[faces.size()-1];
}

//int HEP_Mesh::createFaceWithEdge(int e0, int v2)
//{
//        HEP_Face f;
//        int f = faces.size();
//        faces.push_back(f);
//
//        HEP_Edge& e0 = edges[e0];
//        int v0 = e0.from_vert;
//        int v1 = e0.to_vert;
//
//        HEP_Edge e1 (v1, v2, f);
//        HEP_Edge e2 (v2, v0, f);
//
//        int e1 = edges.size() + 0;
//        int e2 = edges.size() + 1;
//
//        edges.push_back(e1);
//        edges.push_back(e2);
//
//        connectEdgesPrevNext(e0, e1);
//        connectEdgesPrevNext(e1, e2);
//        connectEdgesPrevNext(e2, e0);
//
//        vertices[v0].someEdge = e0;
//        vertices[v1].someEdge = e1;
//        vertices[v2].someEdge = e2;
//
//        f.edge[0] = e0;
//        f.edge[1] = e1;
//        f.edge[2] = e2;
//
//        return f;
//}


BoundingBox HEP_Mesh::bbox()
{
    BoundingBox ret(vertices[0].p, vertices[0].p);
    for (HEP_Vertex& v : vertices)
    {
        ret.min.x = std::min(ret.min.x, v.p.x);
        ret.min.y = std::min(ret.min.y, v.p.y);
        ret.min.z = std::min(ret.min.z, v.p.z);

        ret.max.x = std::max(ret.max.x, v.p.x);
        ret.max.y = std::max(ret.max.y, v.p.y);
        ret.max.z = std::max(ret.max.z, v.p.z);

    }
    return ret;
}
BoundingBox HEP_Mesh::computeFaceBbox(HEP_Face* f)
{
//    HEP_Vertex& v0 = *getTo(edges[faces[f].edge[0]]);
//    HEP_Vertex& v1 = *getTo(edges[faces[f].edge[1]]);
//    HEP_Vertex& v2 = *getTo(edges[faces[f].edge[2]]);
//
//    return BoundingBox(v0.p, v1.p) + v2.p;

    Point p0 = HEP_FaceHandle(*this, f).p0();
    Point p1 = HEP_FaceHandle(*this, f).p1();
    Point p2 = HEP_FaceHandle(*this, f).p2();
    return BoundingBox(p0, p1) + p2;
}

//Point3 HEP_Mesh::getNormal(HEP_Face& face) const
//{
//    Point3 p0 = vertices[edges[face.edge[0]].from_vert].p;
//    Point3 p1 = vertices[edges[face.edge[1]].from_vert].p;
//    Point3 p2 = vertices[edges[face.edge[2]].from_vert].p;
//    return FPoint3::cross(p1-p0, p2-p0).normalized().toPoint3();
//};
Point3 HEP_Mesh::getNormal(HEP_Face* f) //const
{
    Point p0 = HEP_FaceHandle(*this, f).p0();
    Point p1 = HEP_FaceHandle(*this, f).p1();
    Point p2 = HEP_FaceHandle(*this, f).p2();
    return FPoint3::cross(p1-p0, p2-p0).normalized().toPoint3();
};

void HEP_Mesh::addFace(Point& p0, Point& p1, Point& p2)
{} // TODO
void HEP_Mesh::finish()
{} // TODO

void HEP_Mesh::clear()
{
    vertices.clear();
    edges.clear();
    faces.clear();
}

HEP_Mesh::~HEP_Mesh()
{
    clear();
}

HEP_Mesh::HEP_Mesh(FVMesh& mesh)
: Mesh(nullptr)
{
    for (int vIdx = 0 ; vIdx < mesh.vertices.size() ; vIdx++)
    {
        vertices.push_back(HEP_Vertex(mesh.vertices[vIdx].p, nullptr));
    }

    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        FVMeshFace& face = mesh.faces[fIdx];
        HEP_Face heFace;
        faces.push_back(heFace);
        HEP_Face* heFaceP = &faces[faces.size()-1];

        HEP_Edge edge0(&vertices[face.vertex_index[0]], heFaceP); // vertices in face are ordered counter-clockwise
        HEP_Edge edge1(&vertices[face.vertex_index[1]], heFaceP);
        HEP_Edge edge2(&vertices[face.vertex_index[2]], heFaceP);

        edges.push_back(edge0); HEP_Edge* edge0P = &edges[edges.size()-1];
        edges.push_back(edge1); HEP_Edge* edge1P = &edges[edges.size()-1];
        edges.push_back(edge2); HEP_Edge* edge2P = &edges[edges.size()-1];


        vertices[ face.vertex_index[0] ].someEdge = edge0P; // overwrites existing data, if present
        vertices[ face.vertex_index[1] ].someEdge = edge1P;
        vertices[ face.vertex_index[2] ].someEdge = edge2P;

        edge0.next_edge = edge1P;
        edge1.next_edge = edge2P;
        edge2.next_edge = edge0P;

        heFace.edge[0] = edge0P;
        heFace.edge[1] = edge1P;
        heFace.edge[2] = edge2P;

    }


    // connect half-edges:

    bool faceEdgeIsConnected[mesh.faces.size()][3] = {}; // initialize all as false


    // for each edge of each face : if it doesn't have a converse then find the converse in the edges of the opposite face
    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        FVMeshFace& face = mesh.faces[fIdx];

        HEP_Face* heFaceP = &faces[faces.size()-1];

        HEP_FaceHandle fnew(*this, heFaceP); // face index on FVMesh corresponds to index in HEP_Mesh

//        HEP_EdgeHandle edge = fnew.edge0();
//        HEP_EdgeHandle edgeStart = edge;
//        do //
        for (int eIdx = 0; eIdx < 3; eIdx++)
        {
            if (faceEdgeIsConnected[fIdx][eIdx])
                { // edge.next();
                continue; }

            HEP_Edge* edge = heFaceP->edge[eIdx];

            int face2 = face.connected_face_index[eIdx]; // connected_face X is connected via vertex X and vertex X+1

            if (face2 < 0)
            {
                atlas::logError("Incorrect model: disconnected faces. Support generation aborted.\n");
                exit(1); // TODO: not exit, but continue without support!
            }

            for (int e2 = 0; e2 < 3; e2++)
            {
                if (&vertices[  mesh.faces[face2].vertex_index[e2]  ] == edge->next_edge->from_vert)
                {

                    faces[face2].edge[e2]->converse_edge = edge;
                    edge->converse_edge = faces[face2].edge[e2];
                    faceEdgeIsConnected[face2][e2] = true; // the other way around doesn't have to be set; we will not pass the same edge twice
                    break;
                }
                if (e2 == 2) std::cerr << "Couldn't find converse of edge !!!!!" << std::endl;
            }


            //edge = edge.next();
        } //while (edge != edgeStart)

    }

    HEP_MESH_DEBUG_DO(
        std::cerr <<  "============================" << std::endl;
        std::cerr <<  "mesh: " << std::endl;
        std::cerr <<  "faces: "+ std::to_string(mesh.faces.size()) << std::endl;
        std::cerr << "vertices: "+std::to_string(mesh.vertices.size()) << std::endl;


        std::cerr <<  "============================" << std::endl;
        std::cerr <<  ("half-edge mesh: ") << std::endl;
        std::cerr <<  ("faces: ") << std::endl;
        for (int f = 0; f < faces.size(); f++)
            std::cerr << f << " " <<(faces[f].toString()) << std::endl;
        std::cerr << ("edges: ") << std::endl;
        for (int f = 0; f < edges.size(); f++)
            std::cerr << f << " " <<(edges[f].toString()) << std::endl;
        std::cerr << ("vertices: ") << std::endl;
        for (int f = 0; f < vertices.size(); f++)
            std::cerr << f << " " <<(vertices[f].toString()) << std::endl;
        std::cerr <<  "============================" << std::endl;
     )

}



