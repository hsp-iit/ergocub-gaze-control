/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */
    ///////////////////////////////////////////////////////////////////////////////////////////////////
  //                    Demonstration of bimanual grasping with the ergoCub robot                  //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include "GazeControl.h"


// These are used for setting the length of trajectories
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

		// Sample time
		double sample_time = 0.01;
		// 
		yarp::os::Network yarp;                                                       // First connect to the network
		GazeControl gazeControl(pathToURDF, jointList, portList, sample_time);  
		gazeControl.set_cartesian_gains(1);
		
		// Configure communication across the yarp network
		yarp::os::RpcServer port;                                                      // Create a port for sending / receiving info
		port.open("/GazeController/command");                                                  // Open the port with the name '/command'
		yarp::os::Bottle input;                                                        // Store information from the user input
		yarp::os::Bottle output;                                                       // Store information to send to the user
		std::string command;                                                           // Response message, command from user
		
		// Run the control loop
		bool active = true;

		command = "look_at";

		double f = 0.5;
		double A = 0.1;
		double t = 0;
		Eigen::Vector3d root_to_camera = Eigen::Vector3d(0.074927, -0.011469, 1.523281 - 0.9);

		while(active)
		{
			double y = std::cos(2 * M_PI * f * t) * A;
			double z = std::sin(2 * M_PI * f * t) * A;

			gazeControl.set_gaze(Eigen::Vector3d(0.5, 0.0, z) + root_to_camera);
			gazeControl.step();
			std::this_thread::sleep_for(std::chrono::milliseconds(int(sample_time * 1000.0)));
			t += 0.001;
		}
		
		return 0;
	}
}
