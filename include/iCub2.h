  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB2_H_
#define ICUB2_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d and the like
#include <PositionControl.h>                                                                        // Custom class: most functions defined here
		        
class iCub2 : public PositionControl
{
	public:
		iCub2(const std::string              &pathToURDF,
		      const std::vector<std::string> &jointNames,
		      const std::vector<std::string> &portNames);
	
	private:
		// Shoulder constraints
		Eigen::MatrixXd A;
		Eigen::Matrix<double,10,1> b;
		
		// General constraint matrices for QP solver
		Eigen::MatrixXd B;
		Eigen::VectorXd z;
		
		Eigen::VectorXd setPoint;                                                           // Desired joint configuration
		
		void run();                                                                         // Main control loop
			
};                                                                                                  // Semicolon needed after class declaration

#endif
