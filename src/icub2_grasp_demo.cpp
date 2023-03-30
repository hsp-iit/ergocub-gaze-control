  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Two-handed grasping demo with the iCub2.5                             //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCub2.h>                                                                                  // Custom class for controlling iCub2
#include <yarp/os/Property.h>                                                                       // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

/*
double long_time  = 4.0;
double short_time = 2.0;

double mass = 0.1;
Eigen::Matrix<double,3,3> inertia = (Eigen::MatrixXd(3,3) << 1e-06,   0.0,   0.0,
                                                               0.0, 1e-06,   0.0,
                                                               0.0,   0.0, 1e-06).finished();
*/
std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};


int main(int argc, char* argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 4)
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] Port name and path to URDF are required. "
		          << "Usage: ./icub2_grasp_demo /portName /path/to/model.urdf /path/to/config.ini" << std::endl;
		
		return 1;                                                                           // Close with error
	}

	std::string portName     = argv[1];                                                         // Get the port names
	std::string pathToURDF   = argv[2];                                                         // Get the file path
	std::string pathToConfig = argv[3];                                                         // Path to the configuration file
	
	// Generate port list prefixes
	std::vector<std::string> portList;
	portList.push_back(portName + "/torso");
	portList.push_back(portName + "/left_arm");
	portList.push_back(portName + "/right_arm");
	
	yarp::os::Property parameter; parameter.fromConfigFile(pathToConfig);                       // Load the properties from the config file
	
	yarp::os::Bottle bottle; bottle.add(parameter.find("list"));
	std::cout << "This bottle has " << bottle.size() << " element(s).\n";
	
	try // To start up the robot
	{
		iCub2 robot(pathToURDF, jointList, portList);                                       // Start up the robot
		
		// Set the Cartesian gains
		double kp = parameter.findGroup("CARTESIAN").find("proportional").asFloat64();
		double kd = parameter.findGroup("CARTESIAN").find("derivative").asFloat64();	
		if(not robot.set_cartesian_gains(kp,kd)) return 1;
		
		// Set the joint gains
		kp = parameter.findGroup("JOINT").find("proportional").asFloat64();
		kd = parameter.findGroup("JOINT").find("derivative").asFloat64();	
		if(not robot.set_joint_gains(kp,kd)) return 1;
		
		robot.close();
		
		return 0;                                                                           // No problems with main
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] There was a problem with initialization. "
		          << "See the error message below." << std::endl;
		          
		std::cout << exception.what() << std::endl;                                         // Inform the user
		
		return 1;                                                                           // Close with error
	}
}
/*
int main(int argc, char *argv[])
{

	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 3)									
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] Port name and path to URDF are requred. "
			  << " Usage: './grasp-demo /portName /path/to/model.urdf'" << std::endl;
		return 1;                                                                           // Close with error
	}
	else
	{
		std::string portName   = argv[1];
		std::string pathToURDF = argv[2];
		
		std::vector<std::string> portList;
		portList.push_back(portName + "/torso");
		portList.push_back(portName + "/left_arm");
		portList.push_back(portName + "/right_arm");
	
		// Create the robot model
		iCub2 robot(pathToURDF, jointList, portList);
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                             // First connect to the network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/command");                                                              // Open the port with the name '/command'
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
			
			if(command == "aft")
			{
				
				output.addString("Indietro");
				
				robot.translate(Eigen::Vector3d(-0.075, 0.0, 0.0),
					        Eigen::Vector3d(-0.075, 0.0, 0.0),
					        short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "close")
			{
				output.addString("Arrivederci");
				robot.halt();                                                       // Stop any control threads
				active = false;
			}
			else if(command == "down")
			{
				output.addString("Giu'");
				
				robot.translate(Eigen::Vector3d(0.0, 0.0, -0.075),
					        Eigen::Vector3d(0.0, 0.0, -0.075),
					        short_time);
					       
				yarp::os::Time::delay(short_time);
			}
			else if(command == "fake grasp")
			{
				if(not robot.is_grasping())
				{
					output.addString("Grazie");
					
					robot.move_to_pose(Eigen::Isometry3d(Eigen::Translation3d(0.30, 0.15,0.65)),
							   Eigen::Isometry3d(Eigen::Translation3d(0.30,-0.15,0.65)),
							   short_time);
					
					yarp::os::Time::delay(short_time);
				}
				else	output.addString("Non posso!");
			}
			else if(command == "fore")
			{
				output.addString("Avanti");
				
				robot.translate(Eigen::Vector3d(0.075, 0.0, 0.0),
					        Eigen::Vector3d(0.075, 0.0, 0.0),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
			}		
			else if(command == "grasp")
			{
				if(not robot.is_grasping())
				{
					output.addString("Grazie");
					
					robot.move_to_pose(Eigen::Isometry3d(Eigen::Translation3d(0.30, 0.15,0.65)),
							   Eigen::Isometry3d(Eigen::Translation3d(0.30,-0.15,0.65)),
							   short_time);
							   
					yarp::os::Time::delay(1.1*short_time);
					// Box is 295mm (0.295m) wide
					Eigen::Isometry3d boxPose(Eigen::Translation3d(0.3,0.0,0.65)); // Pose of box relative to robot
					
					robot.grasp_object( Payload( robot.left_hand_pose().inverse()*boxPose, mass, inertia ) );
				}
			}			
			else if(command == "home")
			{
				output.addString("Casa");
				
				robot.move_to_position(home, short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "in")
			{
				
	//			if(not robot.is_grasping() )
	//			{
					output.addString("Capito");
				
					robot.translate(Eigen::Vector3d(0.0,-0.075, 0.0),
							Eigen::Vector3d(0.0, 0.075, 0.0),
							short_time);
					
					yarp::os::Time::delay(short_time);     
	//			}
	//			else	output.addString("Non posso!");
			}
			else if(command == "left")
			{
				output.addString("Sinistra");
				
				robot.translate(Eigen::Vector3d(0.0, 0.075, 0.0),
					        Eigen::Vector3d(0.0, 0.075, 0.0),
					        short_time);
				
				yarp::os::Time::delay(short_time);     
			}
			else if(command == "left hand pose")
			{
				output.addString("Check the other terminal.");
				robot.print_hand_pose("left");
			}
			else if(command == "out")
			{
	//			if(not robot.is_grasping())
	//			{
					output.addString("Capito");
					
					robot.translate(Eigen::Vector3d(0.0, 0.075, 0.0),
							Eigen::Vector3d(0.0,-0.075, 0.0),
							short_time);
				
					yarp::os::Time::delay(short_time);
	//			}
	//			else	output.addString("Non posso!");
			}
			else if(command == "ready")
			{
				output.addString("Pronto");
				robot.move_to_position(ready, short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "release")
			{
				if(robot.is_grasping())
				{
					output.addString("Capito");
					
					robot.release_object();
					
					std::vector<Eigen::VectorXd> waypoints;
					waypoints.push_back(ready);
					waypoints.push_back(home);
					
					std::vector<double> times;
					times.push_back(2.0);
					times.push_back(4.0);
					
					robot.move_to_positions(waypoints,times);
					
					yarp::os::Time::delay(times.back());
				}

			}			
			else if(command == "right")
			{
				output.addString("Destra");
				
				robot.translate(Eigen::Vector3d(0.0,-0.075, 0.0),
					        Eigen::Vector3d(0.0,-0.075, 0.0),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
			}
			else if(command == "right hand pose")
			{
				output.addString("Check the other terminal.");
				robot.print_hand_pose("right");
			}
			else if(command == "shake")
			{
				output.addString("Piacere");
				robot.move_to_position(shake, short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "stop")
			{
				output.addString("Fermare");
				robot.halt();
			}
			else if(command == "up")
			{
				output.addString("Su");

				robot.translate(Eigen::Vector3d(0.0, 0.0, 0.075),
					        Eigen::Vector3d(0.0, 0.0, 0.075),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
			}
			else if(command == "wave")
			{
				output.addString("Ciao");
				
				std::vector<Eigen::VectorXd> wave;
				wave.push_back(wave1);
				wave.push_back(wave2);
				wave.push_back(wave1);
				wave.push_back(wave2);
				wave.push_back(home);
				
				std::vector<double> times;
				times.push_back(2.0);
				times.push_back(3.0);
				times.push_back(4.0);
				times.push_back(5.0);
				times.push_back(8.0);
				
				robot.move_to_positions(wave,times);
				
				yarp::os::Time::delay(times.back());
			}
			else
			{
				output.addString("Cosa");
			}
			
			port.reply(output);                                                         // Send the output
		}
		
		robot.close();                                                                      // Close the device drivers

	return 0;
	}
}
*/
