#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "settings.h"

namespace atlas {

class fffProcessor;
class CommandSocket
{
private:
    ClientSocket socket;

    int object_count;
    int current_object_number;
public:
    CommandSocket(int portNr);

    void handleIncommingData(fffProcessor* processor);

    void sendLayerInfo(int layer_nr, int32_t z, int32_t height);
    void sendProgress(float amount);
};

}//namespace atlas

#endif//COMMAND_SOCKET_H
