#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include <algorithm>
#include <sstream>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "commandSocket.h"

#include "mesh/HalfEdgeMesh.h"

#include "supportClassification.h"
#include "supportGeneration.h"

#include "boolMesh.h"

#include <iostream>


namespace atlas {

//FusedFilamentFabrication processor.
class fffProcessor : public SettingsBase
{
private:
    int maxObjectHeight;
    int fileNr;
    TimeKeeper timeKeeper;
    CommandSocket* commandSocket;

public:
    fffProcessor()
    {
        fileNr = 1;
        maxObjectHeight = 0;
        commandSocket = nullptr;
    }

    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
    }


    bool processFiles(const std::vector<std::string> &files)
    {
        timeKeeper.restart();
        PrintObject* model = nullptr;

        model = new PrintObject(this);
        for(std::string filename : files)
        {
            log("Loading %s from disk...\n", filename.c_str());

            FMatrix3x3 matrix;
            if (!loadFVMeshFromFile(model, filename.c_str(), matrix))
            {
                logError("Failed to load model: %s\n", filename.c_str());
                return false;
            }
        }
        model->finalize();

        log("Loaded from disk in %5.3fs\n", timeKeeper.restart());
        return processModel(model);
    }

    bool processModel(PrintObject* model)
    {
        timeKeeper.restart();
        if (!model)
            return false;

        TimeKeeper timeKeeperTotal;



        std::cerr << "starting Test..." << std::endl;
        //SupportChecker::testSupportChecker(model);
        //SupportPointsGenerator::testSupportPointsGenerator(model);
        //SupportBlockGenerator::test(model);
        BooleanFVMeshOps::test();

        std::cerr << std::endl << " Test finished!" << std::endl << std::endl;





        logProgress("process", 1, 1);//Report the GUI that a file has been fully processed.
        log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());

        return true;
    }


};

}//namespace atlas

#endif//FFF_PROCESSOR_H
