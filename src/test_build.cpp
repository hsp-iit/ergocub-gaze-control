
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Test build for code                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include<iCubBase.h>
#include <optional>

std::vector<std::string> portList = {"/icubSim/torso", "/icubSim/left_arm", "/icubSim/right_arm"};

std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			              "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		                      "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};

int main(int argc, char* argv[])
{	          
	try
	{
		JointInterface(jointList, portList);
		
		std::cout << "Worker bees can leave.\n"
			  << "Even drones can fly away.\n"
			  << "The Queen is their slave.\n";
		
		return 0;
	}
	catch(std::exception &error)
	{
		std::cerr << "[ERROR] There was a problem. See the error message below.\n";
		std::cerr << error.what() << std::endl;

		return 1;
	}
}

/*
#include <iCub2.h>

std::vector<std::string> portList = {"/icubSim/torso", "/icubSim/left_arm", "/icubSim/right_arm"};

std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};
		        
int main(int argc, char *argv[])
{	          
	if(argc != 3)
	{
		std::cerr << "[ERROR] Not enough input arguments. Usage: "
		          << "./test_build /portName path/to/model.urdf\n";
		
		return 1;
	}
	
	std::string portName   = argv[1];
/*	std::vector<string> portList;
	portList.push_back(portName+"/torso");
	portList.push_back(portName+"/left_arm");
	portList.push_back(portName+"/right_arm");
	
	std::string pathToURDF = argv[2];

	try { iCub2 robot(pathToURDF,portList,jointList); }
	catch(std::exception &error)
	{
		std::cout << "[ERROR] Unable to instantiate robot control class. "
		          << "See the error message below for the problem.\n";
		          
		std::cerr << error.what() << std::endl;
		
		return 1;
	}
	
	std::cout << "Success!\n";
	
	return 0;                                                                                   // No problems with main
}
*/
