// #include <vector>
// #include <iostream>
// #include <cmath>
// #include <chrono>
// #include <thread>

// #include <Eigen/Core>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include "RPCServerInterface.h"
      

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;        
    yarp::os::Port client_port;
    RPCServerInterface gaze_controller;
    
    std::string server_name = "/Components/GazeController";
    std::string client_name = "/BT/GazeController";

    client_port.open(client_name);

    if (!yarp.connect(client_name,server_name))
    {
        std::cout << "Error! Could not connect to server " << server_name << '\n';
        exit(1);
    }

    gaze_controller.yarp().attachAsClient(client_port);

    double f = 1;
    double A = 0.1;
    double t = 0;
    std::vector<double> root_to_camera{0.074927, -0.011469, 1.523281 - 0.9};

    while(true)
    {
        double y = std::cos(2 * M_PI * f * t) * A;
        double z = std::sin(2 * M_PI * f * t) * A;

        std::vector<double> point{0.1, y, z};

        for(int i=0; i<3; i++){
            point[i] += root_to_camera[i];
        }
        

        gaze_controller.look_at(point);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t += 0.001;
    }
    
    return 0;
	
}
