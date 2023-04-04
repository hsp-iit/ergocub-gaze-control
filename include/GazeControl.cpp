#include "GazeControl.h"

GazeControl::GazeControl(const std::string &pathToURDF,
                         const std::vector<std::string> &jointList,
                         const std::vector<std::string> &portList) : 
						 yarp::os::PeriodicThread(0.01),                                                 // Thread running at 100Hz
						 numJoints(jointList.size()),                                                    // Set number of joints
						 q(Eigen::VectorXd::Zero(this->numJoints)),                                      // Set the size of the position vector
						 qdot(Eigen::VectorXd::Zero(this->numJoints)),                                   // Set the size of the velocity vector
						 J(Eigen::MatrixXd::Zero(6,this->numJoints)),                                    // Set the size of the Jacobian matrix
						M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                       // Set the size of the inertia matrix
						invM(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints))                     // Set the size of the inverse inertia
{
    this->jointInterface = new JointInterface(jointList, portList);
    this->solver = new QPSolver();

    iDynTree::ModelLoader loader;
	
	std::string message = "[ERROR] [ICUB BASE] Constructor: ";
	
	if(not loader.loadReducedModelFromFile(pathToURDF, jointList, "urdf"))
	{
		message += "Could not load model from the path " + pathToURDF + ".";
		throw std::runtime_error(message);
	}
	else 
    {
        iDynTree::Model temp = loader.model();
		
        // Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			message += "Could not generate iDynTree::KinDynComputations object from the model "
			              + loader.model().toString() + ".";

			throw std::runtime_error(message);
		}
		else
        {
			// Resize vectors and matrices based on number of joints
			this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control		
			// if(not update_state()) throw std::runtime_error(message + "Unable to read initial joint state from the encoders.");
			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from " << pathToURDF << ".\n";
        }
    }
};



bool GazeControl::update_state()
{
	if(this->jointInterface->read_encoders(this->q, this->qdot))
	{		
		// Put data in iDynTree class to compute in
		// (there is probably a smarter way but I keep getting errors otherwise)
		iDynTree::VectorDynSize tempPosition(this->numJoints);
		iDynTree::VectorDynSize tempVelocity(this->numJoints);

		for(int i = 0; i < this->numJoints; i++)
		{
			tempPosition(i) = this->q(i);
			tempVelocity(i) = this->qdot(i);
		}

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->computer.setRobotState(tempPosition,                                       // Joint positions
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
			// Get the Jacobian for the hands
			Eigen::MatrixXd temp(6,6+this->numJoints);                                  // Temporary storage
			
			this->computer.getFrameFreeFloatingJacobian("realsense_rgb_frame",temp);    // Compute camera Jacobian "realsense_rgb_frame"
			this->J.block(0,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Remove floating base
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute inertia matrix for joints & base
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			this->invM = this->M.partialPivLu().inverse();                              // We will need the inverse late
			
	// 		// Update camera pose
			this->cameraPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("realsense_rgb_frame"));  // realsense_rgb_frame
			

			return true;
		}
		else
		{
			std::cerr << "[ERROR] [ICUB BASE] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [ICUB BASE] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d GazeControl::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move the gaze to the desired pose                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
// bool GazeControl::move_to_pose(const Eigen::Isometry3d &cameraPose,
//                                const double &time)
// {
// 	// Put them in to std::vector objects and pass onward
// 	std::vector<Eigen::Isometry3d> cameraPoses(1,cameraPose);
// 	std::vector<double> times(1,time);
	
// 	return move_to_poses(cameraPoses, times);                                           // Call full function
// }


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move the gaze through multiple poses                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::move_to_pose(const Eigen::Isometry3d& desiredCameraPose,
                               const double& duration)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlSpace = cartesian;                                                             // Switch to Cartesian control mode
	
	// Set up the times for the trajectory
	//std::vector<double> t; t.push_back(0.0);                                                    // Start immediately
	//t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for each hand
	//std::vector<Eigen::Isometry3d> cameraPoints; cameraPoints.push_back(this->cameraPose);      // First waypoint is current pose
	//cameraPoints.insert(cameraPoints.end(),camera.begin(),camera.end());
	
	try
	{
		//this->cameraTrajectory  = CartesianTrajectory(cameraPoints,t);                          // Assign new trajectory for camera hand
		this->desiredCameraPose = desiredCameraPose;
		this->endTime = duration;                                                           // For checking when done
		
		start();                                                                                // Go to threadInit();
		
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_poses(): "
		          << "Unable to set new Cartesian trajectories.\n";
		
		std::cout << exception.what() << std::endl;

		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> GazeControl::pose_error(const Eigen::Isometry3d &desired,
                                               const Eigen::Isometry3d &actual)
{
	Eigen::Matrix<double,6,1> error;                                                            // Value to be computed
	
	error.head(3) = desired.translation() - actual.translation();                               // Position / translation error
	
	Eigen::Matrix<double,3,3> R = desired.rotation()*actual.rotation().inverse();               // Rotation error as SO(3)
	
	// "Unskew" the rotation error
	error(3) = R(2,1);
	error(4) = R(0,2);
	error(5) = R(1,0);
	
	return error;
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Set the Cartesian gains                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::set_cartesian_gains(const double &proportional)
{
	if(proportional < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_cartesian_gains(): "
		          << "Gains must be positive, but your inputs was " << proportional << ".\n";
		
		return false;
	}
	else{
		this->K = proportional * Eigen::MatrixXd::Identity(6, 6);
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the joint gains                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_joint_gains(): "
		          << "Gains must be positive, but your inputs were " << proportional
		          << " and " << derivative << ".\n";
		          
		return false;
	}
	else
	{
		this->kp = proportional;
		this->kd = derivative;
		
		return true;
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::run()
{
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since activation of control loop
	
	if(elapsedTime >= this->endTime) this->isFinished = true;
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->numJoints);
		
		for(int i = 0; i < this->numJoints; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
			if(qd(i) < this->jointInterface->positionLimit[i][0])
				qd(i) = this->jointInterface->positionLimit[i][0] + 0.001;              // Just above the lower limit
			if(qd(i) > this->jointInterface->positionLimit[i][1])
				qd(i) = this->jointInterface->positionLimit[i][1] - 0.001;              // Just below the upper limit
		}
		
		this->qRef = qd;                                                                    // Reference position for joint motors
	}
	else
	{
		Eigen::VectorXd dq(this->numJoints);                                                        // We want to solve this
		Eigen::VectorXd redundantTask = 0.01*(this->setPoint - this->q);
		Eigen::VectorXd q0(this->numJoints);
		
		// Calculate instantaneous joint limits
		Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);
		for(int i = 0; i < this->numJoints; i++)
		{
			double lower, upper;
			compute_joint_limits(lower,upper,i);
			
			lowerBound(i) = lower;
			upperBound(i) = upper;
			q0(i) = 0.5*(lower + upper);
		}
		
		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Get the desired Cartesian motion
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(this->J.rows(), this->J.rows()) * 0.1;
				
		try // to solve the joint motion
		{
			dq = this->solver->least_squares(
		                           dx,                                                      // Constraint vector
		                           this->J,                                                 // Constraint matrix
								   W,
		                           lowerBound,
		                           upperBound,
		                           q0);                                                     // Start point
		}
		catch(const char* error_message)
		{
			std::cout << error_message << std::endl;
			dq.setZero();
		}
		this->qRef += dq;
	}

	this->jointInterface->send_joint_commands(this->qRef);
}



  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
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
		lower = this->jointInterface->positionLimit[jointNum][0] - this->qRef[jointNum];
		upper = this->jointInterface->positionLimit[jointNum][1] - this->qRef[jointNum];
		
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
Eigen::Matrix<double,6,1> GazeControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	// Variables used in this scope
	Eigen::Matrix<double,6,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;                                                         // Desired velocity & acceleration
	
	this->cameraTrajectory.get_state(pose,vel,acc,time);                                        // Desired state for the left hand
	dx.head(6) = this->dt*vel + this->K*pose_error(pose,this->cameraPose);                      // Feedforward + feedback on the left hand

	// this->rightTrajectory.get_state(pose,vel,acc,time);                                      // Desired state for the right hand
	// dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);                    // Feedforward + feedback on the right hand

	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd GazeControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
	return dq;
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GazeControl::threadInit()
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
		this->solver->clear_last_solution();                                                 // In the QP solver
		this->isFinished = false;                                                           // New action started
		this->qRef = this->q;                                                               // Start from current joint position
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		return true;                                                                        // jumps immediately to run()
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::threadRelease()
{
	this->jointInterface->send_joint_commands(this->q);                                                               // Maintain current joint positions
}