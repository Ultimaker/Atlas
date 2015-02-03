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
        Point3 minn = model->meshes[mi].bbox.min;
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

        FVMesh newFVMesh(nullptr);

        std::cerr << " >>>>>>>>>>>>> generating support blocks " << std::endl;

        g.generateSupportBlocks(newFVMesh);

        std::cerr << " >>>>>>>>>>>>> saving to file " << std::endl;
        saveFVMeshToFile(newFVMesh, "blockSupport.stl");

        newFVMesh.debugOuputBasicStats(std::cerr);





        // support points generation
//        SupportPointsGenerator pg(supporter, 1, 2, 3, 300);
//        std::cerr << "n points generated: " << pg.supportPoints.size() << std::endl;
//        std::ofstream out("bs/supportClassification.obj");
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



void SupportBlockGenerator::generateSupportBlocks(FVMesh& result)
{

    for (int f = 0 ; f < mesh.faces.size() ; f++)
    {
        if (!checker.faceIsBad[f]) continue; // face is not bad at all

        supportFace(f, result);

    }

    for (int e = 0 ; e < mesh.edges.size() ; e++)
    {
        HE_Edge& edge = mesh.edges[e];
        int f0 = edge.face_idx;
        //int f1 = mesh.getConverse(edge)->face_idx;

        if (checker.faceIsBad[f0]) continue; // edge is boundary edge or halfedge of bad edge
        if (! checker.edgeIsBad[e] &&
            ! checker.faceIsBad[mesh.edges[edge.converse_edge_idx].face_idx]) continue;

        supportEdge(e, result);

    }
    for (int v = 0 ; v < mesh.vertices.size() ; v++)
    {
        if (!checker.vertexIsBad[v]) continue; // vertex is not bad at all

        supportVert(v, result);


    }

    // : \/ inserts the original model in the outputted model!
    //for (HE_Face face : mesh.faces)
    //    result.addFace(mesh.vertices[mesh.edges[face.edge_idx[0]].to_vert_idx].p, mesh.vertices[mesh.edges[face.edge_idx[1]].to_vert_idx].p, mesh.vertices[mesh.edges[face.edge_idx[2]].to_vert_idx].p);

    result.finish();

    rebaseSupportBlocksOnModel(result);
}





void SupportBlockGenerator::supportFace(int f, FVMesh& result)
{

    auto projectDown = [this](Point& p) { return Point(p.x,p.y,bbox.min.z + dZ_to_object); }; // TODO: remove everything below bbox.min.z

    HE_Face& face = mesh.faces[f];

    Point3 p0_top = mesh.getTo(mesh.edges[face.edge_idx[0]])->p + dz;
    Point3 p1_top = mesh.getTo(mesh.edges[face.edge_idx[1]])->p + dz;
    Point3 p2_top = mesh.getTo(mesh.edges[face.edge_idx[2]])->p + dz;

    Point3 p0_bottom = projectDown(p0_top);
    Point3 p1_bottom = projectDown(p1_top);
    Point3 p2_bottom = projectDown(p2_top);

    result.addFace(p2_top, p1_top, p0_top); // top is flipped
    result.addFace(p0_bottom, p1_bottom, p2_bottom);
}

void SupportBlockGenerator::supportEdge(int e, FVMesh& result)
{
    auto projectDown = [this](Point& p) { return Point(p.x,p.y,bbox.min.z + dZ_to_object); }; // TODO: remove everything below bbox.min.z
    // face f0 is good and (converse face bad or edge bad)

    HE_Edge& edge = mesh.edges[e];


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

void SupportBlockGenerator::supportVert(int v, FVMesh& result)
{

    auto projectDown = [this](Point& p) { return Point(p.x,p.y,bbox.min.z + dZ_to_object); }; // TODO: remove everything below bbox.min.z

    { // check whether vertex is isolated from connected overhang
        int startEdge = mesh.vertices[v].someEdge_idx;
        int edge = startEdge;
        do {
            if (checker.edgeIsBad[edge]) return;
            if (checker.faceIsBad[mesh.edges[edge].face_idx]) return;
            edge = mesh.getConverse(mesh.edges[edge])->next_edge_idx;
        }   while (edge != startEdge);
    }

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


















void SupportBlockGenerator::rebaseSupportBlocksOnModel(FVMesh& result)
{




}
