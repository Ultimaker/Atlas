#include "supportGeneration.h"

#include "modelFile/modelFile.h"

#include <fstream> // ofstream

SupportBlockGenerator::~SupportBlockGenerator()
{
    //dtor
}

  struct FF
{
    int face; int group;
    FF(int f_idx_orr, int g) : face(f_idx_orr), group(g) {};
}; struct EE
{
    int edge; int group;
    EE(int f_idx_orr, int g) : edge(f_idx_orr), group(g) {};
}; struct VV
{
    int vertex; int group;
    VV(int f_idx_orr, int g) : vertex(f_idx_orr), group(g) {};
};

void SupportBlockGenerator::test(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    //for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    int mi = 0;
    {
        Point3 minn = model->meshes[mi].min();
        for (int p = 0 ; p < model->meshes[mi].vertices.size() ; p++)
        {
            model->meshes[mi].vertices[p].p -= minn;
        }

        std::cerr << " >>>>>>>>>>>>> HE_Mesh generation " << std::endl;

        HE_Mesh mesh(model->meshes[mi]);
        std::cerr << " >>>>>>>>>>>>> support checker " << std::endl;

        SupportChecker supporter = SupportChecker::getSupportRequireds(model->meshes[mi], .785); // 45/180*M_PI

        mesh.debugOuputBasicStats(std::cerr);





        // overhang classification

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





        // support block generation

        SupportBlockGenerator g(supporter, mesh);

        Mesh newMesh(nullptr);

        std::cerr << " >>>>>>>>>>>>> generating support blocks " << std::endl;

        g.generateSupportBlocks(newMesh);

        std::cerr << " >>>>>>>>>>>>> saving to file " << std::endl;
        saveMeshToFile(newMesh, "blockSupport.stl");

        newMesh.debugOuputBasicStats(std::cerr);





        // support points generation
//        SupportPointsGenerator pg(supporter, 1, 2, 3, 300);
//        std::cerr << "n points generated: " << pg.supportPoints.size() << std::endl;
//        std::ofstream out("supportClassification.obj");
//        for (int p = 0; p < pg.supportPoints.size() ; p++)
//        {
//            Point3 pp = pg.supportPoints[p].p;
//            // std::cerr << "v "<<pp.x<<" "<<pp.y<<" "<<pp.z<<" # " << pg.supportPoints[p].fromType << std::endl;
//            out << "v "<<pp.x*.001<<" "<<pp.y*.001<<" "<<pp.z*.001<<" # " << pg.supportPoints[p].fromType << " " << pg.supportPoints[p].idx << std::endl;
//        }
//        out.close();

    }
    std::cerr << "=============================================\n" << std::endl;

}

void SupportBlockGenerator::generateSupportBlocks(Mesh& result)
{
    Point dz(0,0,dZ_to_object);
    auto projectDown = [this](Point& p) { return Point(p.x,p.y,bbox.min.z - 10); }; // TODO: remove -10

    for (int f = 0 ; f < mesh.faces.size() ; f++)
    {
        if (!checker.faceIsBad[f]) continue; // face is not bad at all

        HE_Face& face = mesh.faces[f];

        Point3 p0_top = mesh.getTo(mesh.edges[face.edge_index[0]])->p + dz;
        Point3 p1_top = mesh.getTo(mesh.edges[face.edge_index[1]])->p + dz;
        Point3 p2_top = mesh.getTo(mesh.edges[face.edge_index[2]])->p + dz;

        Point3 p0_bottom = projectDown(p0_top);
        Point3 p1_bottom = projectDown(p1_top);
        Point3 p2_bottom = projectDown(p2_top);

        result.addFace(p2_top, p1_top, p0_top); // top is flipped
        result.addFace(p0_bottom, p1_bottom, p2_bottom);

    }

    for (int e = 0 ; e < mesh.edges.size() ; e++)
    {
        HE_Edge& edge = mesh.edges[e];
        int f0 = edge.face_idx;
        //int f1 = mesh.getConverse(edge)->face_idx;

        if (checker.faceIsBad[f0]) continue; // edge is boundary edge or halfedge of bad edge
        if (! checker.edgeIsBad[e] &&
            ! checker.faceIsBad[mesh.edges[edge.converse_edge_idx].face_idx]) continue;

        // face f0 is good and (converse face bad or edge bad)

        Point3 p0_top = mesh.getFrom(edge)->p + dz;
        Point3 p1_top = mesh.getTo(edge)->p + dz;
        Point3 p0_bottom = projectDown(p0_top);
        Point3 p1_bottom = projectDown(p1_top);


        if (p1_top.z < p0_top.z) // draw diagonal along the shortest crosssection of the quadrilateral
        {
            result.addFace(p0_top, p1_top, p0_bottom);
            result.addFace(p1_top, p1_bottom, p0_bottom);
        } else
        {
            result.addFace(p0_top, p1_top, p1_bottom);
            result.addFace(p0_top, p1_bottom, p0_bottom);
        }

    }
    for (int v = 0 ; v < mesh.vertices.size() ; v++)
    {
        if (!checker.vertexIsBad[v]) continue; // vertex is not bad at all


        bool is_isolated = true;
        int edge = mesh.vertices[v].someEdge_idx;
        int startEdge = edge;
        do {
            if (checker.edgeIsBad[edge]) { is_isolated = false; break; }
            if (checker.faceIsBad[mesh.edges[edge].face_idx]) { is_isolated = false; break; }
            edge = mesh.getConverse(mesh.edges[edge])->next_edge_idx;
        }   while (edge != startEdge);
        if (!is_isolated) continue;


        Point p0_top (-vertexSupportPillarRadius,0,dZ_to_object);
        Point p1_top (pillarDx, pillarDy, dZ_to_object);
        Point p2_top (pillarDx, -pillarDy, dZ_to_object);
        p0_top += mesh.vertices[v].p;
        p1_top += mesh.vertices[v].p;
        p2_top += mesh.vertices[v].p;


        Point3 p0_bottom = projectDown(p0_top);
        Point3 p1_bottom = projectDown(p1_top);
        Point3 p2_bottom = projectDown(p2_top);

        //pillar creation:
        result.addFace(p0_top, p1_top, p2_top); // top
        result.addFace(p2_bottom, p1_bottom, p0_bottom); // bottom

        result.addFace(p0_top, p1_top, p0_bottom); // top diagonal triangles
        result.addFace(p1_top, p2_top, p1_bottom);
        result.addFace(p2_top, p0_top, p2_bottom);

        result.addFace(p1_top, p1_bottom, p0_bottom); // bottom diagonal triangles
        result.addFace(p2_top, p2_bottom, p1_bottom);
        result.addFace(p0_top, p0_bottom, p2_bottom);


    }

    // : \/ inserts the original model in the outputted model!
    //for (HE_Face face : mesh.faces)
    //    result.addFace(mesh.vertices[mesh.edges[face.edge_index[0]].to_vert_idx].p, mesh.vertices[mesh.edges[face.edge_index[1]].to_vert_idx].p, mesh.vertices[mesh.edges[face.edge_index[2]].to_vert_idx].p);

    result.finish();

    rebaseSupportBlocksOnModel(result);
}

/*
void SupportBlockGenerator::generateSupportBlocks_HE_Mesh(vector<HE_Mesh>& result)
{

    vector<int> badFaceIdxs;
    vector<int> badEdgeIdxs;
    vector<int> badVertIdxs;

    for (int f_idx_orr = 0 ; f_idx_orr < mesh.faces.size() ; f_idx_orr++)
        if (checker.faceIsBad[f_idx_orr]) badFaceIdxs.push_back(f_idx_orr);

    for (int e = 0 ; e < mesh.edges.size() ; e++)
        if (checker.edgeIsBad[e]) badEdgeIdxs.push_back(e);

    for (int v = 0 ; v < mesh.vertices.size() ; v++)
        if (checker.vertexIsBad[v]) badVertIdxs.push_back(v);

//    bool badFaceChecked[badFaceIdxs.size()] = {false};
//    bool badEdgeChecked[badEdgeIdxs.size()] = {false};
//    bool badVertChecked[badVertIdxs.size()] = {false};
//


//    FF* mapFace[badFaceIdxs.size()]; // mapping from the original mesh to the new mesh (which is storing the connected overhang)
//    EE* mapEdge[badEdgeIdxs.size()];
//    VV* mapVert[badVertIdxs.size()];
    std::vector<FF*> mapFace(badFaceIdxs.size()); // mapping from the original mesh to the new mesh (which is storing the connected overhang)
    std::vector<EE*> mapEdge(badEdgeIdxs.size());
    std::vector<VV*> mapVert(badVertIdxs.size());

    enum GroupType { EDGE, AREA };
    struct OverhangGroup
    {
        HE_Mesh connectedOverhang;
        //std::vector<HE_Edge*> boundary;
        GroupType groupType;
        OverhangGroup(GroupType gt) : groupType(gt) {};
    };

    std::vector<OverhangGroup> groups;

    //std::vector<std::vector<HE_Edge*>> boundaries; // a boundary for each overhang area

    for (int f_idx_orr = 0 ; f_idx_orr < mesh.faces.size() ; f_idx_orr++)
    {
        if (mapFace[f_idx_orr] != nullptr) continue; // face is already added to a connected overhang group
        if (!checker.faceIsBad[f_idx_orr]) continue; // face is not bad at all

        HE_Face& face_orr = mesh.faces[f_idx_orr];

        OverhangGroup new_group(EDGE);
        HE_Mesh& connectedOverhang = new_group.connectedOverhang;
        int group = groups.size();

        groups.push_back(new_group);

        //std::vector<HE_Edge*> boundary = new_group.boundary;


        std::vector<int> todo; // edges in original model to check the opposite face of

        int edge0_orr = face_orr.edge_index[0];
        int edge1_orr = face_orr.edge_index[1];
        int edge2_orr = face_orr.edge_index[2];

        todo.push_back(edge0_orr);
        todo.push_back(edge1_orr);
        todo.push_back(edge2_orr);

        int face_idx = addFace(f_idx_orr, mapFace, mapEdge, mapVert, connectedOverhang, group);

        while (todo.size() > 0)
        {
            int edge_idx_orr = todo.back();
            todo.pop_back();

            int converse_edge_idx_orr = mesh.edges[edge_idx_orr].converse_edge_idx;
            HE_Edge& conv_edge_orr = mesh.edges[converse_edge_idx_orr];

            if (checker.faceIsBad[conv_edge_orr.face_idx])
            {
                int new_face_idx = addFace(conv_edge_orr.face_idx, mapFace, mapEdge, mapVert, connectedOverhang, group);

                int new_conv_edge_idx = mapEdge[converse_edge_idx_orr]->edge;
                int new_edge_idx = mapEdge[edge_idx_orr]->edge;
                connectedOverhang.connectEdgesConverse(new_edge_idx, new_conv_edge_idx);

                todo.push_back(conv_edge_orr.next_edge_idx);
                todo.push_back(mesh.getNext(conv_edge_orr)->next_edge_idx);
            } else
            {
                int new_conv_edge_idx = addEdge(converse_edge_idx_orr, mapEdge, mapVert, connectedOverhang, group);
                int new_edge_idx = mapEdge[edge_idx_orr]->edge;
                connectedOverhang.edges[new_conv_edge_idx].converse_edge_idx = new_edge_idx; // connect overhang edge with converse
                connectedOverhang.edges[new_edge_idx].converse_edge_idx = new_conv_edge_idx;
                //boundary.push_back(&connectedOverhang.edges[new_conv_edge_idx]);
            }
        }

        //boundaries.push_back(boundary);

    }



    for (int e = 0 ; e < mesh.edges.size() ; e++)
    {
        if (mapEdge[e] != nullptr) continue;
        if (!checker.edgeIsBad[e]) continue; // edge is not bad at all

        HE_Edge* edge = &mesh.edges[e]; // is going to be the edge for which the from-vertex belongs to the group


        int from_vert_idx = edge.from_vert_idx;
        int to_vert_idx = edge.to_vert_idx;

        int group_idx;
        OverhangGroup* group_ptr;
        if (mapVert[from_vert_idx] != nullptr)
        {
            group_idx = mapVert[from_vert_idx].group;
            group_ptr = &groups.[group_idx];
        }
        else if (mapVert[to_vert_idx] != nullptr)
        {
            group_idx = mapVert[to_vert_idx].group;
            group_ptr = &groups.[group_idx];
            edge = mesh.getConverse(*edge);
        }
        else
        {
            OverhangGroup new_group;
            group_ptr = &new_group;
            HE_Mesh& connectedOverhang = new_group.connectedOverhang;
            group_idx = groups.size();

            groups.push_back(group);
        }

        OverhangGroup& group = * group_ptr;


        HE_Mesh& group_mesh = group.connectedOverhang;

        OverhangGroup new_group(AREA);
        HE_Mesh& connectedOverhang = new_group.connectedOverhang;
        int group_idx = groups.size();
        groups.push_back(new_group);

        // add edge and its converse to the group!

        int e0_idx = addEdge(e, mapEdge, mapVert, connectedOverhang, group_idx);
        int e1_idx = addEdge(mesh.edges[e].converse_edge_idx, mapEdge, mapVert, connectedOverhang, group_idx);

        HE_Edge& e0 = connectedOverhang.edges[e0_idx];
        HE_Edge& e1 = connectedOverhang.edges[e1_idx];
        e0.next_edge_idx = e1_idx;
        e1.next_edge_idx = e0_idx;
        e0.prev_edge_idx = e1_idx;
        e1.prev_edge_idx = e0_idx;

    }

    for (int v = 0 ; v < mesh.vertices.size() ; v++)
    {
        if (mapVert[v] != nullptr) continue;
        if (!checker.vertexIsBad[v]) continue; // face is not bad at all

        OverhangGroup new_group(EDGE);
        HE_Mesh& connectedOverhang = new_group.connectedOverhang;
        int group_idx = groups.size();
        groups.push_back(new_group);

        Point p0 (-vertexSupportPillarRadius,0,0);
        Point p1 (pillarDx, pillarDy, 0);
        Point p2 (pillarDx, -pillarDy, 0);
        p0 += mesh.vertices[v].p;
        p1 += mesh.vertices[v].p;
        p2 += mesh.vertices[v].p;

        int v0_idx = connectedOverhang.createVertex(p0);
        int v1_idx = connectedOverhang.createVertex(p1);
        int v2_idx = connectedOverhang.createVertex(p2);

        mapVert[v] = new VV(v0_idx, group_idx);

        int f_idx = connectedOverhang.createFace(v0_idx, v1_idx, v2_idx);

        int e0_idx = connectedOverhang.faces[f_idx].edge_index[0];
        int e1_idx = connectedOverhang.faces[f_idx].edge_index[1];
        int e2_idx = connectedOverhang.faces[f_idx].edge_index[2];

        HE_Edge e0_c (v1_idx, v0_idx, f_idx);
        HE_Edge e1_c (v2_idx, v1_idx, f_idx);
        HE_Edge e2_c (v0_idx, v2_idx, f_idx);

        int e0_c_idx = connectedOverhang.createConverse(e0_idx);
        int e1_c_idx = connectedOverhang.createConverse(e1_idx);
        int e2_c_idx = connectedOverhang.createConverse(e2_idx);

    }

    // connect to ground! \/

    for (OverhangGroup group : groups)
    {
        HE_Mesh& overhang = group.connectedOverhang;

        if (group.groupType == EDGE)
        {
            HE_Edge& e0 = overhang.edges[0]; // works best when this edge points downwards (better triangulation) (TODO)
            HE_Edge& conv = * overhang.getConverse(e0);
            HE_Vertex& v0 = overhang.vertices[e0.from_vert_idx];
            HE_Vertex& v1 = overhang.vertices[e0.to_vert_idx];

            int v0p_i = overhang.createVertex(Point(v0.p.x, v0.p.x, bbox.min.z - 10)); // projected down TODO remove -10
            int v1p_i = overhang.createVertex(Point(v1.p.x, v1.p.x, bbox.min.z - 10)); // projected down

            int f0_idx = overhang.createFaceWithEdge(0, v0p_i);
            int f1_idx = overhang.createFaceWithEdge(e0.converse_edge_idx, v0p_i);
            overhang.connectEdgesConverse(e0.prev_edge_idx, conv.next_edge_idx);

            int f2_idx = overhang.createFace(e0.to_vert_idx, v1p_i, v0p_i); // front
            int f3_idx = overhang.createFace(e0.to_vert_idx, v0p_i, v1p_i); // back

            HE_Face& f0 = overhang.faces[f0_idx];
            HE_Face& f1 = overhang.faces[f1_idx];
            HE_Face& f2 = overhang.faces[f2_idx];
            HE_Face& f3 = overhang.faces[f3_idx];

            overhang.connectEdgesConverse(f0.edge_index[1], f2.edge_index[2]); // front diagonal
            overhang.connectEdgesConverse(f1.edge_index[2], f3.edge_index[0]); // back diagonal

            overhang.connectEdgesConverse(f2.edge_index[0], f3.edge_index[2]);
            overhang.connectEdgesConverse(f2.edge_index[1], f3.edge_index[1]);

        }
        else if (group.groupType == AREA)
        {

            for (int e_idx = 0; e_idx < overhang.edges.size(); e_idx++)
            {

            }

        }

    }
}
*/

int SupportBlockGenerator::addVert(int v, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group)
{
   if (mapVert[v] != nullptr) return mapVert[v]->vertex;

    int pos = connectedOverhang.vertices.size();
    HE_Vertex new_vert(mesh.vertices[v].p, -1);
    connectedOverhang.vertices.push_back(new_vert);
    mapVert[v] = new VV(pos, group);
    return pos;
}

int SupportBlockGenerator::addEdge(int e, vector<EE*>& mapEdge, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group)
{
    if (mapEdge[e] != nullptr) return mapEdge[e]->edge;

    int pos = connectedOverhang.edges.size();
    HE_Edge& edge = mesh.edges[e];

    int v0 = addVert(edge.from_vert_idx, mapVert, connectedOverhang, group);
    int v1 = addVert(mesh.getNext(edge)->from_vert_idx, mapVert, connectedOverhang, group);

    mesh.vertices[v1].someEdge_idx = pos; // reverse the edge!!
    HE_Edge new_edge(v1, v0, -1);// reverse the edge!!

    connectedOverhang.edges.push_back(new_edge);
    mapEdge[e] = new EE(pos, group);
    return pos;
}

int SupportBlockGenerator::addFace(int f_idx_orr, vector<FF*>& mapFace, vector<EE*>& mapEdge, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group)
{
    if (mapFace[f_idx_orr] != nullptr) return mapFace[f_idx_orr]->face;

    int pos = connectedOverhang.faces.size();
    HE_Face& face_orr = mesh.faces[f_idx_orr];

    int e0 = addEdge(face_orr.edge_index[0], mapEdge, mapVert, connectedOverhang, group);
    mesh.edges[e0].face_idx = pos;
    int e1 = addEdge(face_orr.edge_index[1], mapEdge, mapVert, connectedOverhang, group);
    mesh.edges[e1].face_idx = pos;
    int e2 = addEdge(face_orr.edge_index[2], mapEdge, mapVert, connectedOverhang, group);
    mesh.edges[e2].face_idx = pos;


    HE_Face new_face(e2, e1, e0); // flip the face!

    mesh.edges[e0].next_edge_idx = e2;
    mesh.edges[e1].next_edge_idx = e0;
    mesh.edges[e2].next_edge_idx = e1;

    mesh.edges[e0].prev_edge_idx = e1;
    mesh.edges[e1].prev_edge_idx = e2;
    mesh.edges[e2].prev_edge_idx = e0;

    connectedOverhang.faces.push_back(new_face);
    mapFace[f_idx_orr] = new FF(pos, group);
    return pos;
}


void SupportBlockGenerator::rebaseSupportBlocksOnModel(Mesh& result)
{




}
