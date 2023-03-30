    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A custom class for 2-handed control of the ergoCub                        //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ERGOCUB_H_
#define ERGOCUB_H_

#include <PositionControl.h>

class ergoCub : public PositionControl
{
	public:
		ergoCub(const std::string &pathToURDF,
		        const std::vector<std::string> &jointNames,
		        const std::vector<std::string> &portNames);
	
	private:
	
		Eigen::VectorXd setPoint =
		(Eigen::VectorXd(17) << 0.0,  0.0,  0.00,
                                       -0.2,  0.4,  0.00,  0.8, -0.4,  0.0,  0.0,
                                       -0.2,  0.4,  0.00,  0.8, -0.4,  0.0,  0.0).finished();
		
		void run();                                                                         // This is the main control loop
		
};                                                                                                  // Semicolon needed after class declaration

#endif
