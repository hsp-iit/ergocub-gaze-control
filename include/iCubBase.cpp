#include <iCubBase.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCubBase::iCubBase(const std::string &pathToURDF,
                   const std::vector<std::string> &jointList,
                   const std::vector<std::string> &portList,
                   const Eigen::Isometry3d        &torsoPose,
                   const std::string              &robotModel)
                   :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointList, portList),                                             // Open communication with joint motors
                   _torsoPose(Eigen_to_iDynTree(torsoPose)),                                        // Pose of torso relative to world frame
                   _robotModel(robotModel),                                                         // iCub2, iCub3, ergoCub
                   q(Eigen::VectorXd::Zero(this->numJoints)),                                       // Set the size of the position vector
                   qdot(Eigen::VectorXd::Zero(this->numJoints)),                                    // Set the size of the velocity vector
                   J(Eigen::MatrixXd::Zero(12,this->numJoints)),                                    // Set the size of the Jacobian matrix
                   M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                       // Set the size of the inertia matrix
                   invM(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints))                     // Set the size of the inverse inertia
{
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
		
		// Add custom hand frames based on the model of the robot
		if(this->_robotModel == "iCub2")
		{
			temp.addAdditionalFrameToLink("l_hand", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,0.0),
			                                                  iDynTree::Position(0.05765, -0.00556, 0.01369)));
		
			temp.addAdditionalFrameToLink("r_hand", "right",
						      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,M_PI),
						                          iDynTree::Position(-0.05765, -0.00556, 0.01369)));
		}
		else if(this->_robotModel == "iCub3")
		{
			throw std::runtime_error(message + "Hand transforms for iCub3 have not been programmed yet!");
		}
		else if(this->_robotModel == "ergocub")
		{
			temp.addAdditionalFrameToLink("l_hand_palm", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
			                                                  iDynTree::Position(-0.00346, 0.00266, -0.0592)));
                	temp.addAdditionalFrameToLink("r_hand_palm", "right",
                				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
                				                          iDynTree::Position(-0.00387, -0.00280, -0.0597)));
		}
		else
		{	
			message += "Expected 'iCub2', 'iCub3' or 'ergoCub' for the robot _robotModel, "
			                "but your input was '" + robotModel + "'.";
			                            
			throw std::invalid_argument(message);
		}
		
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
						
			// Set the static parts of the grasp matrices
			
			// G = [    I    0     I    0 ]
			//     [ S(left) I S(right) I ]
			this->G.block(0,0,3,3).setIdentity();
			this->G.block(0,3,3,3).setZero();
			this->G.block(0,6,3,3).setIdentity();
			this->G.block(0,9,3,3).setZero();
			this->G.block(3,3,3,3).setIdentity();
			this->G.block(3,9,3,3).setIdentity();
			
			// C = [  I  -S(left) -I  S(right) ]
			//     [  0      I     0     -I    ]
			C.block(0,0,3,3).setIdentity();
			C.block(0,6,3,3) = -C.block(0,0,3,3);
			C.block(3,0,3,3).setZero();
			C.block(3,3,3,3).setIdentity();
			C.block(3,6,3,3).setZero();
			C.block(3,9,3,3) = -C.block(0,0,3,3);
			
			if(not update_state()) throw std::runtime_error(message + "Unable to read initial joint state from the encoders.");
			
			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from " << pathToURDF << ".\n";
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders(this->q, this->qdot))
	{		
		// Put data in iDynTree class to compute inverse dynamics
		// (there is probably a smarter way but I keep getting errors otherwise)
		iDynTree::VectorDynSize tempPosition(this->numJoints);
		iDynTree::VectorDynSize tempVelocity(this->numJoints);
		for(int i = 0; i < this->numJoints; i++)
		{
			tempPosition(i) = this->q(i);
			tempVelocity(i) = this->qdot(i);
		}

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->computer.setRobotState(this->_torsoPose,                                   // As it says on the label
		                                tempPosition,                                       // Joint positions
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
			// Get the Jacobian for the hands
			Eigen::MatrixXd temp(6,6+this->numJoints);                                  // Temporary storage
			
			this->computer.getFrameFreeFloatingJacobian("left",temp);                   // Compute left hand Jacobian
			this->J.block(0,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Assign to larger matrix
			
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->J.block(6,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Assign to larger matrix
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute inertia matrix for joints & base
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			this->invM = this->M.partialPivLu().inverse();                              // We will need the inverse late
			
			// Update hand poses
			this->leftPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("left"));
			this->rightPose = iDynTree_to_Eigen(this->computer.getWorldTransform("right"));
			
			// Update the grasp and constraint matrices
			if(this->isGrasping)
			{
				// Assume the payload is rigidly attached to the left hand
				this->payload.update_state(this->leftPose,
				                           iDynTree::toEigen(this->computer.getFrameVel("left"))); 

				// G = [    I    0     I    0 ]
				//     [ S(left) I S(right) I ]
				
				// C = [  I  -S(left)  -I  S(right) ]
				//     [  0      I      0    -I     ]
				
				Eigen::Matrix<double,3,3> S;                                        // Skew symmetric matrix
				
				// Left hand component
				Eigen::Vector3d r = this->leftPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0 ;
				
				this->G.block(3,0,3,3) =  S;
				this->C.block(0,3,3,3) =  S;
				
				// Right hand component
				r = this->rightPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0;
				     
				this->G.block(3,6,3,3) = S;
				this->C.block(0,9,3,3) =-S;
			}
			
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
 //                       Convert Eigen::Isometry3d to iDynTree::Transform                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Transform iCubBase::Eigen_to_iDynTree(const Eigen::Isometry3d &T)
{
	Eigen::Matrix<double,3,3> R = T.rotation();
	Eigen::Vector3d           p = T.translation();
	
	return iDynTree::Transform(iDynTree::Rotation(R),
	                           iDynTree::Position(p(0),p(1),p(2)));
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d iCubBase::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_position(const Eigen::VectorXd &position,
                                const double &time)
{
	if(position.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this robot has " << this->numJoints << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<Eigen::VectorXd> target; target.push_back(position);                    // Insert in to std::vector to pass onward
		std::vector<double> times; times.push_back(time);                                   // Time in which to reach the target position
		return move_to_positions(target,times);                                             // Call "main" function
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Stop the robot immediately                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	send_joint_commands(this->q);                                                               // Hold current joint positions
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_positions(const std::vector<Eigen::VectorXd> &positions,
                                 const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [ICUB BASE] move_to_positions(): "
		          << "Position array had " << positions.size() << " waypoints, "
		          << "but the time array had " << times.size() << " elements!" << std::endl;

		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control thread that might be running
		this->controlSpace = joint;                                                         // Switch to joint control mode
		int m = positions.size() + 1;                                                       // We need to add 1 extra waypoint for the start
		iDynTree::VectorDynSize waypoint(m);                                                // All the waypoints for a single joint
		iDynTree::VectorDynSize t(m);                                                       // Times to reach the waypoints
		
		for(int i = 0; i < this->numJoints; i++)                                            // For the ith joint...
		{
			for(int j = 0; j < m; j++)                                                  // ... and jth waypoint
			{
				if(j == 0)
				{
					waypoint[j] = this->q[i];                                   // Current position is start point
					t[j] = 0.0;                                                 // Start immediately
				}
				else
				{
					double target = positions[j-1][i];                          // Get the jth target for the ith joint
					
					     if(target < this->positionLimit[i][0]) target = this->positionLimit[i][0] + 0.001;
					else if(target > this->positionLimit[i][1]) target = this->positionLimit[i][1] - 0.001;
					
					waypoint[j] = target;                                       // Assign the target for the jth waypoint
					
					t[j] = times[j-1];                                          // Add on subsequent time data
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [ICUB BASE] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}
		}
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // Start the control thread
		return true;                                                                        // Success
	}
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
		
		start();                                                                            // Go to threadInit();
		
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
 //                               Grasp an object with two hands                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::grasp_object(const Payload &_payload)
{
	if(this->isGrasping)
	{
		std::cout << "[ERROR] [ICUB BASE] grasp_object(): "
		          << "Already grasping an object! "
		          << "Need to let go with release_object() before grasping again.\n";
		          
		return false;
	}
	else
	{
		this->isGrasping = true;                                                            // Set grasp constraint
		
		this->payload = _payload;                                                           // Transfer payload information
		
		if(update_state()) return move_object(this->payload.pose(), 5.0);                   // Update pose, grasp constraints, activate control
		else
		{
			std::cout << "[ERROR] [ICUB BASE] grasp_object(): "
			          << "Something went wrong.\n";
			          
			return false;
		}
	}
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Release an object                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::release_object()
{
	if(this->isGrasping)
	{
		this->isGrasping = false;
		
		// Move the hands apart?
		
		return true;
	}
	else
	{
		std::cout << "[INFO] [ICUB BASE] release_object(): "
		          << "You are not currently grasping!" << std::endl;
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Move the box to a given pose                                      //          
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const Eigen::Isometry3d &pose,
                           const double &time)
{
	if(time < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "Time of " << time << " cannot be negative!" << std::endl;
		          
		return false;
	}
	else
	{
		// Insert in to std::vector objects and pass on to spline generator
		std::vector<Eigen::Isometry3d> poses;
		poses.push_back(pose);
		
		std::vector<double> times;
		times.push_back(time);
		
		return move_object(poses,times);                                                    // Pass onward for spline generation
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Move the box through multiple poses                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const std::vector<Eigen::Isometry3d> &poses,
                           const std::vector<double> &times)
{
	if( isRunning() ) stop();                                                                   // Stop any control threads that are running
	
	this->controlSpace = cartesian;                                                             // Ensure that we are running in Cartesian mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0);                                                      // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for the object
	std::vector<Eigen::Isometry3d> waypoints; waypoints.push_back( this->payload.pose() );      // First waypoint is current pose
	waypoints.insert( waypoints.end(), poses.begin(), poses.end() );                            // Add on additional waypoints to the end
	
	try
	{
		this->payloadTrajectory = CartesianTrajectory(waypoints, t);                        // Create new trajectory to follow
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // go immediately to threadInit()
		
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "Could not assign a new trajectory for the object.\n";
		          
		std::cout << exception.what() << std::endl;
		
		return false;       
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> iCubBase::pose_error(const Eigen::Isometry3d &desired,
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
bool iCubBase::set_cartesian_gains(const double &proportional, const double &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_cartesian_gains(): "
		          << "Gains must be positive, but your inputs were " << proportional
		          << " and " << derivative << ".\n";
		
		return false;
	}
	else
	{
		this->K = proportional*this->gainTemplate;
		this->D = derivative*this->gainTemplate;

		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the joint gains                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
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

/*
iCubBase::iCubBase(const std::string              &pathToURDF,
                   const std::vector<std::string> &jointNames,
                   const std::vector<std::string> &portNames,
                   const Eigen::Isometry3d        &torsoPose,
                   const std::string              &robotName)
                   :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointNames, portNames),                                           // Communicates with joint motors
                   _torsoPose(Eigen_to_iDynTree(torsoPose)),
                   _robotModel(robotName)
{
	iDynTree::ModelLoader loader;
	
	std::string message = "[ERROR] [ICUB BASE] Constructor: ";
	
	if(not loader.loadReducedModelFromFile(pathToURDF, jointNames, "urdf"))
	{
		message += "Could not load model from the path " + pathToURDF + ".";
		throw std::runtime_error(message);
	}
	else
	{
		iDynTree::Model temp = loader.model();
		
		// Add custom hand frames based on the model of the robot
		if(this->_robotModel == "icub2")
		{
			temp.addAdditionalFrameToLink("l_hand", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,0.0),
			                                                  iDynTree::Position(0.05765, -0.00556, 0.01369)));
		
			temp.addAdditionalFrameToLink("r_hand", "right",
						      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,M_PI),
						                          iDynTree::Position(-0.05765, -0.00556, 0.01369)));
		}
		else if(this->_robotModel == "icub3")
		{
			throw std::runtime_error(message + "Hand transforms for iCub3 have not been programmed yet!");
		}
		else if(this->_robotModel == "ergocub")
		{
			temp.addAdditionalFrameToLink("l_hand_palm", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
			                                                  iDynTree::Position(-0.00346, 0.00266, -0.0592)));
                	temp.addAdditionalFrameToLink("r_hand_palm", "right",
                				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
                				                          iDynTree::Position(-0.00387, -0.00280, -0.0597)));
		}
		else
		{	
			message += "Expected 'icub2', 'icub3' or 'ergocub' for the robot _robotModel, "
			                "but your input was '" + robotName + "'.";
			                            
			throw std::invalid_argument(message);
		}
		
		// Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			message += "Could not generate iDynTree::KinDynComputations object from the model "
			              + loader.model().toString() + ".";

			throw std::runtime_error(message);
		}
		else
		{
/*
			// Resize vectors based on model information
			this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control
			this->q.resize(this->numJoints);                                            // Vector of measured joint positions
			this->qdot.resize(this->numJoints);                                         // Vector of measured joint velocities
			
			this->J.resize(12,this->numJoints);                                         // Resize the Jacobian
			this->M.resize(this->numJoints,this->numJoints);                            // Resize the inertia matrix
			this->Minv.resize(this->numJoints,this->numJoints);                         // Resize inverse inertia matrix			
			
			if(not update_state()) throw std::runtime_error(message + "Unable to read initial joint state from the encoders.");

			// Set the Cartesian control gains	
			this->gainTemplate << 1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
					      0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
					      0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
					      0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
					      0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
					      0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
					      
			this->K = 10*this->gainTemplate;                                            // Set the spring forces
			this->D =  5*this->gainTemplate;                                            // Set the damping forces
			
			// G = [    I    0     I    0 ]
			//     [ S(left) I S(right) I ]
			this->G.block(0,0,3,3).setIdentity();
			this->G.block(0,3,3,3).setZero();
			this->G.block(0,6,3,3).setIdentity();
			this->G.block(0,9,3,3).setZero();
			this->G.block(3,3,3,3).setIdentity();
			this->G.block(3,9,3,3).setIdentity();
			
			// C = [  I  -S(left) -I  S(right) ]
			//     [  0      I     0     -I    ]
			C.block(0,0,3,3).setIdentity();
			C.block(0,6,3,3) = -C.block(0,0,3,3);
			C.block(3,0,3,3).setZero();
			C.block(3,3,3,3).setIdentity();
			C.block(3,6,3,3).setZero();
			C.block(3,9,3,3) = -C.block(0,0,3,3);

			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from "
			          << pathToURDF << ".\n";
		}
	}
/*
	// Set the Cartesian control gains	
	this->gainTemplate << 1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		              0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		              0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		              0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
		              0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
		              0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
	
	this->K = 10*this->gainTemplate;                                                            // Set the spring forces
	this->D =  5*this->gainTemplate;                                                            // Set the damping forces
	
	this->J.resize(12,this->numJoints);
	this->M.resize(this->numJoints,this->numJoints);
	this->Minv.resize(this->numJoints,this->numJoints);

	// G = [    I    0     I    0 ]
	//     [ S(left) I S(right) I ]
	this->G.block(0,0,3,3).setIdentity();
	this->G.block(0,3,3,3).setZero();
	this->G.block(0,6,3,3).setIdentity();
	this->G.block(0,9,3,3).setZero();
	this->G.block(3,3,3,3).setIdentity();
	this->G.block(3,9,3,3).setIdentity();
	
	// C = [  I  -S(left) -I  S(right) ]
	//     [  0      I     0     -I    ]
	C.block(0,0,3,3).setIdentity();
	C.block(0,6,3,3) = -C.block(0,0,3,3);
	C.block(3,0,3,3).setZero();
	C.block(3,3,3,3).setIdentity();
	C.block(3,6,3,3).setZero();
	C.block(3,9,3,3) = -C.block(0,0,3,3);
	
	
	iDynTree::ModelLoader loader;                                                               // Load a model of the robot
	
	if(not loader.loadReducedModelFromFile(pathToURDF, jointNames, "urdf"))
	{
		std::string message = "[ERROR] [ICUB BASE] Constructor: "
		                      "Could not load a model from the path " + pathToURDF + ".";
		                      
		throw std::runtime_error(message);
	}
	else
	{
		iDynTree::Model temp = loader.model();                                              // Get the model
		
		// Add custom hand frames based on the model of the robot
		if(this->_robotModel == "icub2")
		{
			temp.addAdditionalFrameToLink("l_hand", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,0.0),
			                                                  iDynTree::Position(0.05765, -0.00556, 0.01369)));
		
			temp.addAdditionalFrameToLink("r_hand", "right",
						      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,M_PI),
						                          iDynTree::Position(-0.05765, -0.00556, 0.01369)));
		}
		else if(this->_robotModel == "icub3")
		{
			std::cerr << "This hasn't been programmed yet!" << std::endl;
		}
		else if(this->_robotModel == "ergocub")
		{
			temp.addAdditionalFrameToLink("l_hand_palm", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
			                                                  iDynTree::Position(-0.00346, 0.00266, -0.0592)));
                	temp.addAdditionalFrameToLink("r_hand_palm", "right",
                				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
                				                          iDynTree::Position(-0.00387, -0.00280, -0.0597)));
		}
		else
		{	std::string message = "[ERROR] [ICUB BASE] Constructor: "
                                              "Expected icub2, icub3 or ergocub for the robot _robotModel, "
                                              "but your input was " + robotName + ".";
                                              
			throw std::invalid_argument(message);
		}
		
		// Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			std::string message = "[ERROR] [ICUB BASE] Constructor: "
			                      "Could not generate iDynTree::KinDynComputations class from the model "
			                    + loader.model().toString() + ".";

			throw std::runtime_error(message);
		}
		else
		{
			// Resize vectors based on model information
			this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control
			this->q.resize(this->numJoints);                                            // Vector of measured joint positions
			this->qdot.resize(this->numJoints);                                         // Vector of measured joint velocities
			
			
			if(update_state())
			{
				std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model "
				          << "from " << pathToURDF + ".\n";
			}
			else
			{
				std::string message = "[ERROR] [ICUB BASE] Constructor: "
				                      "Successfully created model from " + pathToURDF + " "
				                      "but was unable to read joint encoders.";
				                      
				throw std::runtime_error(message);
			}
		}
	}
*/

/*

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Print the pose of a hand to the console                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::print_hand_pose(const std::string &which)
{
	if(which == "left" or which == "right")
	{
		std::cout << "Here is the " << which << " hand pose:" << std::endl;
		std::cout << this->computer.getWorldTransform(which).asHomogeneousTransform().toString() << std::endl;

		return true;
	}
	else
	{
		std::cout << "[ERROR] [ICUB BASE] print_hand_pose(): " 
		          << "Expected 'left' or 'right' as the argument, "
		          << "but the input was " << which << "." << std::endl;
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Set the gains for control in Cartesian space                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_cartesian_gains(const double &stiffness, const double &damping)
{
	if(stiffness <= 0 or damping <= 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_cartesian_gains(): "
		          << "Gains cannot be negative! "
		          << "You input " << stiffness << " for the stiffness gain, "
		          << "and " << damping << " for the damping gain." << std::endl;
		
		return false;
	}
	else
	{	
		this->K = stiffness*this->gainTemplate;
		this->D =   damping*this->gainTemplate;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the gains for control in the joint space                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional <= 0 or derivative <= 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_joint_gains(): "
                          << "Gains cannot be negative! "
                          << "You input " << proportional << " for the proportional gain, "
                          << "and " << derivative << " for the derivative gain." << std::endl;
                
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
 //                          Translate both hands by the given amount                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::translate(const Eigen::Vector3d &left,
                         const Eigen::Vector3d &right,
                         const double &time)
{
	if(this->isGrasping)
	{
		return move_object( this->payload.pose() * Eigen::Translation3d(left) , time );
	}
	else
	{
		Eigen::Isometry3d leftTarget  = this->leftPose  * Eigen::Translation3d(left);
		Eigen::Isometry3d rightTarget = this->rightPose * Eigen::Translation3d(right);
		
		return move_to_pose(leftTarget, rightTarget, time);
	}
}



  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders())
	{	
		// Get the values from the JointInterface class so we can use them here
		this->q = joint_positions();
		this->qdot = joint_velocities();
		
		iDynTree::VectorDynSize tempPosition(*this->q.data());
		iDynTree::VectorDynSize tempVelocity(*this->qdot.data());

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->computer.setRobotState(this->_torsoPose,                                   // As it says on the label
		                                tempPosition,                                       // Joint positions
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
		
			// Get the Jacobian for the hands
			Eigen::MatrixXd temp(6,6+this->numJoints);                                  // Temporary storage
			
			this->computer.getFrameFreeFloatingJacobian("left",temp);                   // Compute left hand Jacobian
			this->J.block(0,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Assign to larger matrix
			
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->J.block(6,0,6,this->numJoints) = temp.block(0,6,6,this->numJoints);   // Assign to larger matrix
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute full inertia matrix
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			this->Minv = this->M.partialPivLu().inverse();
			
			// Update hand poses
			this->leftPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("left"));
			this->rightPose = iDynTree_to_Eigen(this->computer.getWorldTransform("right"));
			
			// Update the grasp and constraint matrices
			if(this->isGrasping)
			{
				// Assume the payload is rigidly attached to the left hand
				this->payload.update_state(this->leftPose,
				                           iDynTree::toEigen(this->computer.getFrameVel("left"))); 

				// G = [    I    0     I    0 ]
				//     [ S(left) I S(right) I ]
				
				// C = [  I  -S(left)  -I  S(right) ]
				//     [  0      I      0    -I     ]
				
				Eigen::Matrix<double,3,3> S;                                        // Skew symmetric matrix
				
				// Left hand component
				Eigen::Vector3d r = this->leftPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0 ;
				
				this->G.block(3,0,3,3) =  S;
				this->C.block(0,3,3,3) =  S;
				
				// Right hand component
				r = this->rightPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0;
				     
				this->G.block(3,6,3,3) = S;
				this->C.block(0,9,3,3) =-S;
			}
			
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
*/
