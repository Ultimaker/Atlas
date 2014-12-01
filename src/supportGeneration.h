#ifndef SUPPORT_H
#define SUPPORT_H

#include "supportClassification.h"

using namespace std;
using namespace atlas;


class SupportBlockGenerator
{
    public:
        SupportBlockGenerator(SupportChecker& checker, HE_Mesh& mesh) : checker(checker), mesh(mesh) {};
        virtual ~SupportBlockGenerator();


        SupportChecker checker;
        HE_Mesh mesh;

        vector<HE_Mesh> generateSupportBlocks(); // main function of this class

        void rebaseSupportBlocksOnModel(vector<HE_Mesh> blocks); // does something like subtract the model from the support

        static void test(PrintObject* model);
    protected:


    private:
};

#endif // SUPPORT_H
