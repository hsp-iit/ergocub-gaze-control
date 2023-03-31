    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                    Demonstration of bimanual grasping with the ergoCub robot                  //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include "GazeControl.h"


// These are used for setting the length of trajetories
double longTime =  5.0;
double shortTime = 2.0;

// These are reference points with respect to the robot
double graspWidth    = 0.15;
double graspDist     = 0.45;
double graspRest     = 0.35;
double torsoHeight   = 0.90;
double nominalHeight = torsoHeight + 0.35;
double graspHeight   = nominalHeight;
double torsoDist     = 0.40;

// These are used for creating a Payload object but not really important
double mass = 0.1;
Eigen::Matrix<double,3,3> inertia = (Eigen::MatrixXd(3,3) << 1e-06,   0.0,   0.0,
                                                               0.0, 1e-06,   0.0,
                                                               0.0,   0.0, 1e-06).finished();
		                      

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                              Main                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Ensure correct number of arguments
	if(argc != 3)
	{
		std::cerr << "[ERROR] [ERGOCUB GRASP DEMO] Path to URDF, and port name are required. "
                          << "Usage: './ergocub_grasp_demo /portName /path/to/model.urdf' " << std::endl;
                
                return 1;                                                                           // Close with error
	}
	else
	{
		std::string portName   = argv[1];
		std::string pathToURDF = argv[2];

		// List of joints to control                                           
		std::vector<std::string> jointList = {"neck_roll", "neck_pitch", "neck_yaw", "camera_tilt"};
		
		// List of ports to connect
		std::vector<std::string> portList = {"/" + portName + "/head"};

		// 
		yarp::os::Network yarp;                                                             // First connect to the network
		GazeControl gazeControl(pathToURDF, jointList, portList);  
		
		// Configure communication across the yarp network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/GazeController");                                                              // Open the port with the name '/command'
		yarp::os::Bottle input;                                                             // Store information from the user input
		yarp::os::Bottle output;                                                            // Store information to send to the user
		std::string command;                                                                // Response message, command from user
		
		// Run the control loop
		bool active = true;
		
		while(active)
		{
 			output.clear();                                                             // Clear any previous information
			port.read(input,true);                                                      // Get the input from the /command port
			command = input.toString();                                                 // Convert to a string	

			if(command == "look_at"){
				// gazeControl.move_to_pose()
			}

			port.reply(output);
		}
		
		return 0;                                                                           // No problems with main
	}
}
