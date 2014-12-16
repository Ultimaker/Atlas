#include "utils/logoutput.h"
#include "commandSocket.h"
#include "fffProcessor.h"
#include <cstring> // strlen

namespace atlas {

#define CURA_IDENTIFIER "Atlas"
const static int CMD_REQUEST_IDENTIFIER = 0x00100000;
const static int CMD_IDENTIFIER_REPLY = 0x00100001;
const static int CMD_REQUEST_VERSION = 0x00100002;
const static int CMD_VERSION_REPLY = 0x00100003;

const static int CMD_SETTING = 0x00100004;
const static int CMD_MATRIX = 0x00300002;
const static int CMD_OBJECT_COUNT = 0x00300003;
const static int CMD_OBJECT_LIST = 0x00200000;
const static int CMD_MESH_LIST = 0x00200001;
const static int CMD_VERTEX_LIST = 0x00200002;
const static int CMD_NORMAL_LIST = 0x00200003;

const static int CMD_PROGRESS_REPORT = 0x00300001;

CommandSocket::CommandSocket(int portNr)
{
    socket.connectTo("127.0.0.1", portNr);
    object_count = 1;
    current_object_number = 0;
}

void CommandSocket::handleIncommingData(fffProcessor* processor)
{
    std::vector<PrintObject*> object_list;
    PrintObject* object = NULL;
    FVMesh* mesh = NULL;
    FMatrix3x3 matrix;


    while(true)
    {
        int command = socket.recvInt32();
        int dataSize = socket.recvInt32();
        if (dataSize < 0)
            break;
        switch(command)
        {
        case CMD_REQUEST_IDENTIFIER:
            socket.sendInt32(CMD_IDENTIFIER_REPLY);
            socket.sendInt32(strlen(CURA_IDENTIFIER) + 1);
            socket.sendAll(CURA_IDENTIFIER, strlen(CURA_IDENTIFIER) + 1);
            break;
        case CMD_SETTING:
            {
                char buffer[dataSize+1];
                buffer[dataSize] = '\0';
                socket.recvAll(buffer, dataSize);
                char* value = (buffer + strlen(buffer)) + 1;
                if ((value - buffer) < dataSize)
                {
                    //processor->getSetting(buffer);
                    if (mesh)
                        mesh->setSetting(buffer, value);
                    else if (object)
                        object->setSetting(buffer, value);
                    else
                        processor->setSetting(buffer, value);
                }
            }
            break;
        case CMD_MATRIX:
            {
                for(int x=0; x<3; x++)
                    for(int y=0; y<3; y++)
                        matrix.m[x][y] = socket.recvFloat32();
            }
            break;
        case CMD_OBJECT_COUNT:
            object_count = socket.recvInt32();
            current_object_number = 0;
            break;
        case CMD_OBJECT_LIST:
            socket.recvInt32(); //Number of following CMD_MESH_LIST commands
            if (object)
            {
                object->finalize();
                object_list.push_back(object);
            }
            object = new PrintObject(processor);
            mesh = nullptr;
            break;
        case CMD_MESH_LIST:
            socket.recvInt32(); //Number of following CMD_?_LIST commands that fill this mesh with data
            if (object)
            {
                object->meshes.emplace_back(object);
                mesh = &object->meshes[object->meshes.size()-1];
            }
            break;
        case CMD_VERTEX_LIST:
            if (mesh)
            {
                int faceCount = dataSize / 4 / 3 / 3;
                logError("Reading %i faces\n", faceCount);
                for(int n=0; n<faceCount; n++)
                {
                    FPoint3 fv[3];
                    socket.recvAll(fv, 4 * 3 * 3);
                    Point3 v[3];
                    v[0] = matrix.apply(fv[0]);
                    v[1] = matrix.apply(fv[1]);
                    v[2] = matrix.apply(fv[2]);
                    mesh->addFace(v[0], v[1], v[2]);
                }
                mesh->finish();
            }else{
                for(int n=0; n<dataSize; n++)
                    socket.recvAll(&command, 1);
            }
            break;
        default:
            logError("Unknown command: %04x (%i)\n", command, dataSize);
            for(int n=0; n<dataSize; n++)
                socket.recvAll(&command, 1);
            break;
        }
    }
    if (object)
        delete object;
}


void CommandSocket::sendProgress(float amount)
{
    socket.sendInt32(CMD_PROGRESS_REPORT);
    socket.sendInt32(4);
    socket.sendFloat32(float(current_object_number) / float(object_count) + amount / float(object_count));
}


}//namespace atlas
