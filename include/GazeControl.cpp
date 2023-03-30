#include "GazeControl.h"

GazeControl::GazeControl(const std::string &pathToURDF,
                         const std::vector<std::string> &jointList,
                         const std::vector<std::string> &portList) : 
						 numJoints(jointList.size()),
						 q(Eigen::VectorXd::Zero(this->numJoints)),
						 qdot(Eigen::VectorXd::Zero(this->numJoints)),
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
			while(true)
            	this->update_state(); //TODO remove
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
			
			this->computer.getFrameFreeFloatingJacobian("realsense_rgb_frame",temp);    // Compute camera Jacobian
			this->J.block(0,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Remove floating base
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute inertia matrix for joints & base
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			this->invM = this->M.partialPivLu().inverse();                              // We will need the inverse late
			
	// 		// Update camera pose
			this->cameraPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("realsense_rgb_frame"));

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
 //                                Move each hand to a desired pose                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_pose(const Eigen::Isometry3d &leftPose,
                            const Eigen::Isometry3d &rightPose,
                            const double &time)
{
	// Put them in to std::vector objects and pass onward
	std::vector<Eigen::Isometry3d> leftPoses(1,leftPose);
	std::vector<Eigen::Isometry3d> rightPoses(1,rightPose);
	std::vector<double> times(1,time);
	
	return move_to_poses(leftPoses,rightPoses,times);                                           // Call full function
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//                          Move both hands through multiple poses                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_poses(const std::vector<Eigen::Isometry3d> &left,
                             const std::vector<Eigen::Isometry3d> &right,
                             const std::vector<double> &times)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlSpace = cartesian;                                                             // Switch to Cartesian control mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0.0);                                                    // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for each hand
	std::vector<Eigen::Isometry3d> leftPoints; leftPoints.push_back(this->leftPose);            // First waypoint is current pose
	leftPoints.insert(leftPoints.end(),left.begin(),left.end());
	
	std::vector<Eigen::Isometry3d> rightPoints; rightPoints.push_back(this->rightPose);
	rightPoints.insert(rightPoints.end(), right.begin(), right.end());
	
	try
	{
		this->leftTrajectory  = CartesianTrajectory(leftPoints,t);                          // Assign new trajectory for left hand
		this->rightTrajectory = CartesianTrajectory(rightPoints,t);                         // Assign new trajectory for right hand
		
		this->endTime = times.back();                                                       // For checking when done
		
		run();                                                                              // Go to threadInit();
		
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
		this->qRef += dq;
	}

	for(int i = 0; i < this->n; i++) send_joint_command(i,this->qRef[i]);
}
