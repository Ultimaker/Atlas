#include "supportGeneration.h"


SupportBlockGenerator::~SupportBlockGenerator()
{
    //dtor
}


vector<HE_Mesh> SupportBlockGenerator::generateSupportBlocks()
{

    vector<int> badFaceIdxs;
    vector<int> badEdgeIdxs;
    vector<int> badVertIdxs;

    for (int f = 0 ; f < mesh.faces.size() ; f++)
        if (checker.faceIsBad[f]) badFaceIdxs.push_back(f);

    for (int e = 0 ; e < mesh.edges.size() ; e++)
        if (checker.edgeIsBad[e]) badEdgeIdxs.push_back(e);

    for (int v = 0 ; v < mesh.vertices.size() ; v++)
        if (checker.vertexIsBad[v]) badVertIdxs.push_back(v);

    bool badFaceChecked[badFaceIdxs.size()] = {false};
    bool badEdgeChecked[badEdgeIdxs.size()] = {false};
    bool badVertChecked[badVertIdxs.size()] = {false};

    vector<HE_Mesh> ret;

    for (int f = 0 ; f < mesh.faces.size() ; f++)
    {
        if (badFaceChecked[f]) continue;

        //HE_Mesh connectedOverhang;

    }


    for (int e = 0 ; e < mesh.edges.size() ; e++)
    {
        if (badEdgeChecked[e]) continue;
    }

    for (int v = 0 ; v < mesh.vertices.size() ; v++)
    {
        if (badVertChecked[v]) continue;
    }


    return ret;
}



void SupportBlockGenerator::test(PrintObject* model)
{

    std::cerr << "=============================================\n" << std::endl;

    for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    {
        Point3 minn = model->meshes[mi].min();
        for (int p = 0 ; p < model->meshes[mi].vertices.size() ; p++)
        {
            model->meshes[mi].vertices[p].p -= minn;
        }

        HE_Mesh mesh(model->meshes[mi]);
        SupportChecker supporter = SupportChecker::getSupportRequireds(mesh, .1);//.785); // 45/180*M_PI


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


        SupportBlockGenerator b(supporter, supporter.mesh);


        b.generateSupportBlocks();



    }
    std::cerr << "=============================================\n" << std::endl;

}
