#include <ergoCub.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
ergoCub::ergoCub(const std::string &pathToURDF,
		 const std::vector<std::string> &jointNames,
		 const std::vector<std::string> &portNames)
		 :
		 PositionControl(pathToURDF,
				 jointNames,
				 portNames,
				 Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.96)),
				 "ergocub")
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void ergoCub::run()
{
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since activation of control loop
	
	if(elapsedTime >= this->endTime) this->isFinished = true;
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->n);
		
		for(int i = 0; i < this->n; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
			if(qd(i) < this->pLim[i][0]) qd(i) = this->pLim[i][0] + 0.001;              // Just above the lower limit
			if(qd(i) > this->pLim[i][1]) qd(i) = this->pLim[i][1] - 0.001;              // Just below the upper limit
		}
		
		this->qRef = qd;                                                                    // Reference position for joint motors
	}
	else
	{
		Eigen::VectorXd dq(this->n);                                                        // We want to solve this
		Eigen::VectorXd redundantTask = 0.01*(this->setPoint - this->q);
		Eigen::VectorXd q0(this->n);
		
		// Calculate instantaneous joint limits
		Eigen::VectorXd lowerBound(this->n), upperBound(this->n);
		for(int i = 0; i < this->n; i++)
		{
			double lower, upper;
			compute_joint_limits(lower,upper,i);
			
			lowerBound(i) = lower;
			upperBound(i) = upper;
			q0(i) = 0.5*(lower + upper);
		}
		
		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Get the desired Cartesian motion
				
		try // to solve the joint motion
		{
			dq = least_squares(redundantTask,                                           // Redundant task,
		                           this->M,                                                 // Weight the joint motion by the inertia,
		                           dx,                                                      // Constraint vector
		                           this->J,                                                 // Constraint matrix
		                           lowerBound,
		                           upperBound,
		                           q0);                                                     // Start point
		}
		catch(const char* error_message)
		{
			std::cout << error_message << std::endl;
			dq.setZero();
		}
		
		// Resolve the last subject to grasp constraints
/*		if(this->isGrasping)
		{
			Eigen::MatrixXd Jc = this->C*this->J;                                       // Constraint matrix
			
			try
			{
				dq = solve( dq,
			  		    this->M,
			                    Eigen::VectorXd::Zero(6),
			                    Jc,
			                    lower,
			                    upper,
			                    0.5*(lower + upper) );
			}
			catch(const char* error_message)
			{
				std::cout << error_message << std::endl;
				dq.setZero();
			}
		}
*/
		this->qRef += dq;
	}

	for(int i = 0; i < this->n; i++) send_joint_command(i,this->qRef[i]);
}
