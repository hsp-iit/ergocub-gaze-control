#include "GazeControl.h"
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>

GazeControl::GazeControl(const std::string &pathToURDF,
                         const std::vector<std::string> &jointList,
                         const std::vector<std::string> &portList,
						 const double& sample_time):
						 yarp::os::PeriodicThread(sample_time), 
						 numJoints(jointList.size()),                                                    // Set number of joints
						 q(Eigen::VectorXd::Zero(this->numJoints)),                                      // Set the size of the position vector
						 qdot(Eigen::VectorXd::Zero(this->numJoints)),                                   // Set the size of the velocity vector
						 J(Eigen::MatrixXd::Zero(6,this->numJoints)),                                    // Set the size of the Jacobian matrix
						 M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                       // Set the size of the inertia matrix
						 invM(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                    // Set the size of the inverse inertia
						 sample_time(sample_time)
{
    this->jointInterface = new JointInterface(jointList, portList);
    this->solver = new QPSolver();

	// Redundant Task
	redundantTask.resize(this->numJoints);
	redundantTask.setZero();

	// Debugging
	yarp::os::Network yarp;
	this->debugPort.open("/GazeController/debug:o");

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
			// this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control		
			// if(not update_state()) throw std::runtime_error(message + "Unable to read initial joint state from the encoders.");
			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from " << pathToURDF << ".\n";
        }
    }
	while (not this->update_state()){
		std::this_thread::sleep_for(std::chrono::milliseconds(int(sample_time * 1000.0)));
	}
	this->qRef = this->q;                                                               // Start from current joint position  
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
			this->J = temp.bottomRightCorner(3,this->numJoints);   // Remove floating base
			
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
bool GazeControl::set_gaze(const Eigen::Vector3d& desiredGaze)
{
	try
	{
		this->controlSpace = cartesian;  
		this->desiredGaze = desiredGaze;
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_poses(): "
		          << "Unable to set new Cartesian trajectories.\n";
		
		std::cout << exception.what() << std::endl;

		return false;
	}
	return true;
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,3,1> GazeControl::pose_error(const Eigen::Vector3d &desired,
                                               	  const Eigen::Isometry3d &actual)
{
	Eigen::Matrix<double,3,1> error;                                                            // Value to be computed

	Eigen::Vector3d desiredSight = this->desiredGaze - this->cameraPose.translation();
	Eigen::Vector3d actualSight = this->cameraPose.rotation().col(2);

	yarp::os::Bottle& debugOutput = this->debugPort.prepare();
	debugOutput.clear();

	// Create a 3D point
	yarp::os::Bottle& point = debugOutput.addList();
	point.addFloat64(desiredSight.normalized()(0));
	point.addFloat64(desiredSight.normalized()(1));
	point.addFloat64(desiredSight.normalized()(2));

	// Create a vector
	yarp::os::Bottle& vector = debugOutput.addList();
	vector.addFloat64(actualSight(0));
	vector.addFloat64(actualSight(1));
	vector.addFloat64(actualSight(2));

	this->debugPort.write();
	error = actualSight.cross(desiredSight);
	
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
		this->K = proportional * Eigen::MatrixXd::Identity(3, 3);
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

// double GazeControl::getPeriod()
// {
// 	return this->sample_time;
// }


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::run()
{

	this->solver->clear_last_solution();                                                        // In the QP solver
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since activation of control loop
	
	if(this->controlSpace == joint)
	{
		// Eigen::VectorXd qd(this->numJoints);
		
		// for(int i = 0; i < this->numJoints; i++)
		// {
		// 	qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
		// 	if(qd(i) < this->jointInterface->positionLimit[i][0])
		// 		qd(i) = this->jointInterface->positionLimit[i][0] + 0.001;              // Just above the lower limit
		// 	if(qd(i) > this->jointInterface->positionLimit[i][1])
		// 		qd(i) = this->jointInterface->positionLimit[i][1] - 0.001;              // Just below the upper limit
		// }
		
		// this->qRef = qd;                                                                    // Reference position for joint motors
	}
	else
	{
		Eigen::VectorXd dq(this->numJoints);                                                        // We want to solve this
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
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(this->J.cols(), this->J.cols());
		// W(0, 0) = 1000.0;
		// W(1, 1) = 1000.0;
		W(0, 0) = 0.1;
		W(1, 1) = 0.1;
		W(2, 2) = 0.1;
		W(3, 3) = 0.1;

		// std::cout << dx << std::endl;
				
		try // to solve the joint motion
		{
			for(int j=0;j<4;j++)
			{
				redundantTask(j) = 0.1*(0.5*(this->jointInterface->positionLimit[j][0] + this->jointInterface->positionLimit[j][1]) - this->q(j));
			}
			// redundantTask(0) = 1000000000*(0.5*(this->jointInterface->positionLimit[0][0] + this->jointInterface->positionLimit[0][1]) - this->q(0));
			//redundant_task = ;
			// dq = this->J.inverse() * dx;
			// redundantTask(0) = (0.0 - this->q(0));
			// redundantTask(1) = (0.0 - this->q(1));
			// redundantTask(3) = (0.0 - this->q(3));

			// std::cout << sqrt((J*J.transpose()).determinant()) << std::endl;

			dq = this->solver->least_squares(redundantTask,
		                           			 W,
								             dx,                                                      
		                                     this->J,                                                 // Constraint matrix
		                                     lowerBound,
		                                     upperBound,
		                                     q0);                                                     // Start point
			// dq = this->J.transpose()*(this->J*this->J.transpose()).partialPivLu().solve(dx);
		}
		catch(const char* error_message)
		{
			std::cout << error_message << std::endl;
			dq.setZero();
		}
		// std::cout << dq.transpose() * 180.0 / M_PI << std::endl;
		for(int i = 1; i < this->numJoints; i++){
			this->qRef[i] += dq[i] * sample_time;
		}
	}

	this->debugPort.write();

	for(int i = 0; i < this->numJoints; i++){
		if ((this->qRef[i] * 180.0 / M_PI ) - (this->q[i] * 180.0 / M_PI) > 10){
			throw std::runtime_error("Requested joint motion for joint " +  std::to_string(i) + "is greater than 5 degrees.");
		}
			
		if ((this->qRef[i] * 180.0 / M_PI ) - (this->q[i] * 180.0 / M_PI) < -10){
			throw std::runtime_error("Requested joint motion for joint " +  std::to_string(i) + "is greater than 5 degrees.");
		}
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
		// if (jointNum == 0){
		// 	lower = -0.1 - this->qRef[jointNum];
		// 	upper = 0.1 - this->qRef[jointNum];
		// }
		// else{
		lower = this->jointInterface->positionLimit[jointNum][0] - this->qRef[jointNum];
		upper = this->jointInterface->positionLimit[jointNum][1] - this->qRef[jointNum];
		// }
		
		// std::cout << this->jointInterface->positionLimit[jointNum][0] << " - "<< this->qRef[jointNum] << std::endl;
		// std::cout << lower << std::endl;
		// std::cout << this->jointInterface->positionLimit[jointNum][1] << " - "<< this->qRef[jointNum] << std::endl;
		// std::cout << upper << std::endl;
		
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
Eigen::Matrix<double,3,1> GazeControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	//this->cameraTrajectory.get_state(pose,vel,acc,time);                                        // Desired state for the left hand


	Eigen::Matrix<double,3,1> dx = this->K*pose_error(this->desiredGaze, this->cameraPose);                      // Feedforward + feedback on the left hand
	// std::cout << "Before clipping " << dx.transpose() << std::endl;q
	dx = dx.cwiseMin(2);
	// std::cout << dx.transpose() << std::endl;
	// this->rightTrajectory.get_state(pose,vel,acc,time);                                      // Desired state for the right hand
	// dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);                    // Feedforward + feedback on the right hand

	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
// Eigen::VectorXd GazeControl::track_joint_trajectory(const double &time)
// {
// 	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
// 	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
// 	return dq;
// }


bool GazeControl::threadInit()
{
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void GazeControl::threadRelease()
{
	// send_joint_commands(this->q);                                                               // Maintain current joint positions
}
