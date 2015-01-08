#include "HalfEdgeMesh.h"
#include "../utils/logoutput.h"

#include "../modelFile/modelFile.h" // PrintObject

// enable/disable debug output
#define HE_MESH_DEBUG 0

#define HE_MESH_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#define HE_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HE_MESH_DEBUG_DO(x)
#endif



#include <iostream>



void HE_Mesh::debugOuputBasicStats(std::ostream& out)
{
    out << "faces: " << faces.size() << std::endl;
    out << "vertices: " << vertices.size() << std::endl;
    out << "halfedges: " << edges.size() << std::endl;

}


HE_Edge* HE_Mesh::getNext(HE_Edge& edge) { return &edges[edge.next_edge_idx]; }
//HE_Edge* HE_Mesh::getPrev(HE_Edge& edge) { return &edges[edge.prev_edge_idx]; }
HE_Edge* HE_Mesh::getConverse(HE_Edge& edge) { return &edges[edge.converse_edge_idx]; }
HE_Vertex* HE_Mesh::getTo(HE_Edge& edge) { return &HE_EdgeHandle(*this, edge.next_edge_idx).from_vert().vertex(); } // vertices[edge.to_vert_idx]; }
HE_Vertex* HE_Mesh::getFrom(HE_Edge& edge) { return &vertices[edge.from_vert_idx]; }


HE_Edge* HE_Mesh::getSomeEdge(HE_Face& face) { return &edges[face.edge_idx[0]]; }
HE_Edge* HE_Mesh::getSomeEdge(HE_Vertex& vertex) { return &edges[vertex.someEdge_idx]; }

HE_Face* HE_Mesh::getFace(HE_Edge& edge) { return &faces[edge.face_idx]; }


void HE_Mesh::connectEdgesPrevNext(int prev, int next) { edges[prev].next_edge_idx = next; } //edges[next].prev_edge_idx = prev; }
void HE_Mesh::connectEdgesConverse(int e1, int e2) { edges[e1].converse_edge_idx = e2; edges[e2].converse_edge_idx = e1; }

int HE_Mesh::createVertex(Point p)
{
    vertices.push_back(HE_Vertex(p, -1));
    return vertices.size()-1;
}

int HE_Mesh::createConverse(int e_idx)
{
    int v1_idx = HE_EdgeHandle(*this, e_idx).next().from_vert().idx; //edges[e_idx].to_vert_idx;
    HE_Edge e(v1_idx, -1);
    int new_e_idx = edges.size();
    edges.push_back(e);
    connectEdgesConverse(e_idx, new_e_idx);
    return new_e_idx;
}

int HE_Mesh::createFace(int v0_idx, int v1_idx, int v2_idx)
{
        HE_Face f;
        int f_idx = faces.size();

        HE_Edge e0 (v0_idx, f_idx);
        HE_Edge e1 (v1_idx, f_idx);
        HE_Edge e2 (v2_idx, f_idx);

        int e0_idx = edges.size() + 0;
        int e1_idx = edges.size() + 1;
        int e2_idx = edges.size() + 2;

        edges.push_back(e0);
        edges.push_back(e1);
        edges.push_back(e2);

        connectEdgesPrevNext(e0_idx, e1_idx);
        connectEdgesPrevNext(e1_idx, e2_idx);
        connectEdgesPrevNext(e2_idx, e0_idx);

        vertices[v0_idx].someEdge_idx = e0_idx;
        vertices[v1_idx].someEdge_idx = e1_idx;
        vertices[v2_idx].someEdge_idx = e2_idx;

        f.edge_idx[0] = e0_idx;
        f.edge_idx[1] = e1_idx;
        f.edge_idx[2] = e2_idx;

        faces.push_back(f);

        return f_idx;
}

//int HE_Mesh::createFaceWithEdge(int e0_idx, int v2_idx)
//{
//        HE_Face f;
//        int f_idx = faces.size();
//        faces.push_back(f);
//
//        HE_Edge& e0 = edges[e0_idx];
//        int v0_idx = e0.from_vert_idx;
//        int v1_idx = e0.to_vert_idx;
//
//        HE_Edge e1 (v1_idx, v2_idx, f_idx);
//        HE_Edge e2 (v2_idx, v0_idx, f_idx);
//
//        int e1_idx = edges.size() + 0;
//        int e2_idx = edges.size() + 1;
//
//        edges.push_back(e1);
//        edges.push_back(e2);
//
//        connectEdgesPrevNext(e0_idx, e1_idx);
//        connectEdgesPrevNext(e1_idx, e2_idx);
//        connectEdgesPrevNext(e2_idx, e0_idx);
//
//        vertices[v0_idx].someEdge_idx = e0_idx;
//        vertices[v1_idx].someEdge_idx = e1_idx;
//        vertices[v2_idx].someEdge_idx = e2_idx;
//
//        f.edge_idx[0] = e0_idx;
//        f.edge_idx[1] = e1_idx;
//        f.edge_idx[2] = e2_idx;
//
//        return f_idx;
//}


BoundingBox HE_Mesh::bbox()
{
    BoundingBox ret(vertices[0].p, vertices[0].p);
    for (HE_Vertex& v : vertices)
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
BoundingBox HE_Mesh::computeFaceBbox(int f)
{
//    HE_Vertex& v0 = *getTo(edges[faces[f].edge_idx[0]]);
//    HE_Vertex& v1 = *getTo(edges[faces[f].edge_idx[1]]);
//    HE_Vertex& v2 = *getTo(edges[faces[f].edge_idx[2]]);
//
//    return BoundingBox(v0.p, v1.p) + v2.p;

    Point p0 = HE_FaceHandle(*this, f).p0();
    Point p1 = HE_FaceHandle(*this, f).p1();
    Point p2 = HE_FaceHandle(*this, f).p2();
    return BoundingBox(p0, p1) + p2;
}

//Point3 HE_Mesh::getNormal(HE_Face& face) const
//{
//    Point3 p0 = vertices[edges[face.edge_idx[0]].from_vert_idx].p;
//    Point3 p1 = vertices[edges[face.edge_idx[1]].from_vert_idx].p;
//    Point3 p2 = vertices[edges[face.edge_idx[2]].from_vert_idx].p;
//    return FPoint3::cross(p1-p0, p2-p0).normalized().toPoint3();
//};
Point3 HE_Mesh::getNormal(int f) //const
{
    Point p0 = HE_FaceHandle(*this, f).p0();
    Point p1 = HE_FaceHandle(*this, f).p1();
    Point p2 = HE_FaceHandle(*this, f).p2();
    return FPoint3::cross(p1-p0, p2-p0).normalized().toPoint3();
};

void HE_Mesh::addFace(Point& p0, Point& p1, Point& p2)
{} // TODO
void HE_Mesh::finish()
{} // TODO

void HE_Mesh::clear()
{
    vertices.clear();
    edges.clear();
    faces.clear();
}

HE_Mesh::~HE_Mesh()
{
    clear();
}

HE_Mesh::HE_Mesh(FVMesh& mesh)
: Mesh(nullptr)
{
    for (int vIdx = 0 ; vIdx < mesh.vertices.size() ; vIdx++)
    {
        vertices.push_back(HE_Vertex(mesh.vertices[vIdx].p, -1));
    }

    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        FVMeshFace& fvFace = mesh.faces[fIdx];
        HE_Face heFace;
        HE_Edge edge0(fvFace.vertex_index[0], fIdx); // vertices in face are ordered counter-clockwise
        HE_Edge edge1(fvFace.vertex_index[1], fIdx);
        HE_Edge edge2(fvFace.vertex_index[2], fIdx);

        int newEdgeIndex = edges.size();

        vertices[ fvFace.vertex_index[0] ].someEdge_idx = newEdgeIndex+0; // overwrites existing data, if present
        vertices[ fvFace.vertex_index[1] ].someEdge_idx = newEdgeIndex+1;
        vertices[ fvFace.vertex_index[2] ].someEdge_idx = newEdgeIndex+2;

        edge0.next_edge_idx = newEdgeIndex+1;
        edge1.next_edge_idx = newEdgeIndex+2;
        edge2.next_edge_idx = newEdgeIndex+0;

        edges.push_back(edge0);
        edges.push_back(edge1);
        edges.push_back(edge2);

        heFace.edge_idx[0] = newEdgeIndex+0;
        heFace.edge_idx[1] = newEdgeIndex+1;
        heFace.edge_idx[2] = newEdgeIndex+2;
        faces.push_back(heFace);

    }


    // connect half-edges:

    bool faceEdgeIsConnected[mesh.faces.size()][3] = {}; // initialize all as false


    // for each edge of each face : if it doesn't have a converse then find the converse in the edges of the opposite face
    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        FVMeshFace& fvFace = mesh.faces[fIdx];
        //FVMeshFaceHandle fh(mesh, fIdx);

        HE_FaceHandle fnew(*this, fIdx); // face index on FVMesh corresponds to index in HE_Mesh

//        HE_EdgeHandle edge = fnew.edge0();
//        HE_EdgeHandle edgeStart = edge;
//        do //
        for (int eIdx = 0; eIdx < 3; eIdx++)
        {
            if (faceEdgeIsConnected[fIdx][eIdx])
                { // edge.next();
                continue; }

            int edge_idx = faces[fIdx].edge_idx[eIdx];
            HE_Edge& edge = edges[ edge_idx ];
            int face2 = fvFace.connected_face_index[eIdx]; // connected_face X is connected via vertex X and vertex X+1

            if (face2 < 0)
            {
                atlas::logError("Incorrect model: disconnected faces. Support generation aborted.\n");
                exit(1); // TODO: not exit, but continue without support!
            }

            for (int e2 = 0; e2 < 3; e2++)
            {
                if (mesh.faces[face2].vertex_index[e2] == edges[edge.next_edge_idx].from_vert_idx)
                {
                    edges[ faces[face2].edge_idx[e2] ].converse_edge_idx = edge_idx;
                    edge.converse_edge_idx = faces[face2].edge_idx[e2];
                    faceEdgeIsConnected[face2][e2] = true; // the other way around doesn't have to be set; we will not pass the same edge twice
                    break;
                }
                if (e2 == 2) std::cerr << "Couldn't find converse of edge " << std::to_string(edge_idx) <<"!!!!!" << std::endl;
            }


            //edge = edge.next();
        } //while (edge != edgeStart)

    }

    HE_MESH_DEBUG_DO(
        mesh.debugOutputWholeMesh();
     )

}

void HE_Mesh::debugOutputWholeMesh()
{
    std::cerr <<  "============================" << std::endl;
    std::cerr <<  "mesh: " << std::endl;
    std::cerr <<  "faces: "+ std::to_string(faces.size()) << std::endl;
    std::cerr <<  "edges: "+ std::to_string(edges.size()) << std::endl;
    std::cerr << "vertices: "+std::to_string(vertices.size()) << std::endl;


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
}


void HE_Mesh::makeManifold(FVMesh& correspondingFVMesh)
{
    int size = vertices.size();
    for (int v = 0; v < size; v++)
    {
        FVMeshVertexHandle fvvh(correspondingFVMesh, v);
        //HE_MESH_DEBUG_SHOW(v);
        HE_VertexHandle(*this, v).splitWhenNonManifold(fvvh);
    }

}


void HE_Mesh::testMakeManifold(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    {
        FVMesh& fvMesh = model->meshes[mi];
        HE_Mesh mesh(fvMesh);
        mesh.debugOutputWholeMesh();

        std::cerr << "=============================================\n" << std::endl;

        mesh.makeManifold(fvMesh);

        mesh.debugOutputWholeMesh();

        //for (int f = 0; f < mesh.faces.size(); f++)
        //    std::cerr << mesh.faces[f].cosAngle() << std::endl;
    }
    std::cerr << "=============================================\n" << std::endl;

}
