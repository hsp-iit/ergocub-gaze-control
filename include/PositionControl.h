    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Position control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_

#include <iCubBase.h>

class PositionControl : public iCubBase
{
	public:
		PositionControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointList,
			        const std::vector<std::string> &portList,
			        const Eigen::Isometry3d        &torsoPose,
			        const std::string              &robotModel)
		:
	        iCubBase(pathToURDF, jointList, portList, torsoPose, robotModel) {}

		// NOTE: THESE ARE INHERITED FROM ICUBBASE
		bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
	protected:
		Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors

		// NOTE; THESE ARE INHERITED FROM PERIODICTHREAD
		bool threadInit();
		void threadRelease();
//		void run() {} <--- TO BE DEFINED IN ANY CHILD CLASS
};                                                                                                  // Semicolon needed after class declaration

#endif
