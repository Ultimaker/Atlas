#ifndef SUPPORT_H
#define SUPPORT_H

#include "Kernel.h"

#include "mesh/FVMesh.h"

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
    Point dz;

    public:


        SupportBlockGenerator(SupportChecker& checker, HE_Mesh& mesh) : dz(0,0,dZ_to_object), checker(checker), mesh(mesh), bbox(mesh.bbox()) {};
        virtual ~SupportBlockGenerator();


        SupportChecker checker;
        HE_Mesh mesh;
        BoundingBox bbox;

        void generateSupportBlocks(FVMesh& result); //!< main function of this class
        // void generateSupportBlocks_HE_Mesh(vector<HE_Mesh>& result); //!< main function of this class

        void rebaseSupportBlocksOnModel(FVMesh& result); //!< does something similar to subtracting the model solid from the support block solid

        static void test(PrintObject* model);
    protected:


    private:
        void groupOverhangAreas(vector<HE_Mesh>& result); //!< makes new (incomplete!) meshes for each connected group of overhang

        inline void supportFace(int f, FVMesh& result);
        inline void supportEdge(int e, FVMesh& result);
        inline void supportVert(int v, FVMesh& result);

};

#endif // SUPPORT_H
