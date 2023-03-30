#include <PositionControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::threadInit()
{
	if(isRunning())
	{
		std::cout << "[ERROR] [POSITION CONTROL] threadInit(): "
		          << "A control thread is still running!\n";
		return false;
	}
	else
	{
		// Reset values
		clear_last_solution();                                                              // In the QP solver
		this->isFinished = false;                                                           // New action started
		this->qRef = this->q;                                                               // Start from current joint position
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		return true;                                                                        // jumps immediately to run()
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	send_joint_commands(this->q);                                                               // Maintain current joint positions
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->numJoints)
	{
		std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
		          << "Range of joint indices is 0 to " << this->numJoints - 1 << ", "
		          << "but you called for " << jointNum << ".\n";

		return false;
	}
	else
	{
		lower = this->positionLimit[jointNum][0] - this->qRef[jointNum];
		upper = this->positionLimit[jointNum][1] - this->qRef[jointNum];
		
		if(lower >= upper)
		{
			std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
				  << "Lower limit " << lower << " is greater than upper limit " << upper << ". "
				  << "How did that happen???\n";
			
			return false;
		}
		else	return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;                                                         // Desired velocity & acceleration
	
	if(this->isGrasping)
	{
		this->payloadTrajectory.get_state(pose,vel,acc,time);                               // Get the desired object state for the given time              
		
		dx = this->G.transpose()*(this->dt*vel + this->K*pose_error(pose,this->payload.pose())); // Feedforward + feedback control		
	}
	else
	{
		this->leftTrajectory.get_state(pose,vel,acc,time);                                  // Desired state for the left hand
		dx.head(6) = this->dt*vel + this->K*pose_error(pose,this->leftPose);                // Feedforward + feedback on the left hand

		this->rightTrajectory.get_state(pose,vel,acc,time);                                 // Desired state for the right hand
		dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);               // Feedforward + feedback on the right hand
	}
	
	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PositionControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
	return dq;
}

