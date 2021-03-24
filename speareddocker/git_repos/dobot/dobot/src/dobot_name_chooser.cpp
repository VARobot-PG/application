//
// Created by lars on 03.05.19.
//

#include "DobotDll_x64/DobotDll.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "[USAGE]: Application portName dobot_name" << std::endl;
        return -1;
    }

    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
            break;
        case DobotConnect_NotFound:
            std::cerr << "Dobot not found!" << std::endl;
            return -2;
            break;
        case DobotConnect_Occupied:
            std::cerr << "Invalid port name or Dobot is occupied by other application!" << std::endl;
            return -3;
            break;
        default:
            break;
    }

    //name dobot
    SetDeviceName(argv[2]);

    //check if named
    char deviceName[256];
    result = GetDeviceName(deviceName, sizeof(deviceName));
    if(result != DobotConnect_NoError) return -4;
    std::cout << std::string(deviceName) << std::endl;

    // Disconnect Dobot
    SetQueuedCmdForceStopExec();
    DisconnectDobot();

    return 0;
}
