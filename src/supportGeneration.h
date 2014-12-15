#ifndef SUPPORT_H
#define SUPPORT_H

#include "Kernel.h"

#include "mesh.h"

#include "supportClassification.h"
#include <math.h>

using namespace std;
using namespace atlas;

struct VV;
struct EE;
struct FF;

class SupportBlockGenerator
{
    spaceType vertexSupportPillarRadius = 100;
    // pillar is triagular!
    spaceType pillarDx = vertexSupportPillarRadius *0.5;
    spaceType pillarDy = vertexSupportPillarRadius *std::sqrt(.75);
    spaceType dZ_to_object = -100;

    public:


        SupportBlockGenerator(SupportChecker& checker, HE_Mesh& mesh) : checker(checker), mesh(mesh), bbox(mesh.bbox()) {};
        virtual ~SupportBlockGenerator();


        SupportChecker checker;
        HE_Mesh mesh;
        BoundingBox bbox;

        void generateSupportBlocks(Mesh& result); //!< main function of this class
        // void generateSupportBlocks_HE_Mesh(vector<HE_Mesh>& result); //!< main function of this class

        void rebaseSupportBlocksOnModel(Mesh& result); //!< does something similar to subtracting the model solid from the support block solid

        static void test(PrintObject* model);
    protected:


    private:
        void groupOverhangAreas(vector<HE_Mesh>& result); //!< makes new (incomplete!) meshes for each connected group of overhang

        inline int addVert(int v, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group);
        inline int addEdge(int e, vector<EE*>& mapEdge, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group);
        inline int addFace(int f, vector<FF*>& mapFace, vector<EE*>& mapEdge, vector<VV*>& mapVert, HE_Mesh& connectedOverhang, int group);

};

#endif // SUPPORT_H
