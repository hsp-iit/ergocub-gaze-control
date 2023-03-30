    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                       Base class for bimanual control of the iCub                              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUBBASE_H_
#define ICUBBASE_H_

#include <CartesianTrajectory.h>
#include <Eigen/Dense>                                                                              // Tensors and matrix decomposition
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen tensors
#include <iDynTree/Core/CubicSpline.h>                                                              // Uses for joint trajectories
#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations
#include <iDynTree/Model/Model.h>                                                                   // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Class for communicating with joint motors
#include <Payload.h>
#include <QPSolver.h>                                                                               // For control optimisation
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops

class iCubBase : public QPSolver,
		 public JointInterface,
		 public yarp::os::PeriodicThread
{
	public:
		iCubBase(const std::string              &pathToURDF,
			 const std::vector<std::string> &jointList,
		         const std::vector<std::string> &portList,
		         const Eigen::Isometry3d        &torsoPose,
		         const std::string              &robotModel);

		bool update_state();                                                                // Read encoders, compute kinematics & dynamics
		
		bool is_finished() const { return this->isFinished; }                               // Returns true when trajectory reaches its end
		
		void halt();                                                                        // Stops the robot immediately
		
		
		// Joint control functions
		bool move_to_position(const Eigen::VectorXd &position,
		                      const double &time);                                          // Move the joints to a given position
		                      
		bool move_to_positions(const std::vector<Eigen::VectorXd> &positions,
		                       const std::vector<double> &times);                           // Move the joints through several positions
		                       
		bool set_joint_gains(const double &proportional, const double &derivative);
		                       
		// Cartesian control functions
		bool is_grasping() const { return this->isGrasping; }                               // Check if grasp constraint is active
		
		bool move_to_pose(const Eigen::Isometry3d &leftPose,
		                  const Eigen::Isometry3d &rightPose,
		                  const double &time);                                              // Move each hand to a pose
		                  
		bool move_to_poses(const std::vector<Eigen::Isometry3d> &leftPoses,
		                   const std::vector<Eigen::Isometry3d> &rightPoses,
		                   const std::vector<double> &times);                               // Move hands through several poses
		                   
		bool translate(const Eigen::Vector3d &left,
		               const Eigen::Vector3d &right,
		               const double &time);                                                 // Translate the hands by a specific amount
		               
		Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
		                                     const Eigen::Isometry3d &actual);
		                                     
		bool set_cartesian_gains(const double &proportional, const double &derivative);
		               
		// Grasp control functions
		bool grasp_object(const Payload &_payload);                                         // As it says on the label
		
		bool release_object();                                                              // Deactivates grasp constraints
		
		bool move_object(const Eigen::Isometry3d &pose, const double &time);                // Move the grasped object to a single pose
		              
		bool move_object(const std::vector<Eigen::Isometry3d> &poses, const std::vector<double> &times); // Move object through multiple waypoints	       

	protected:
	
		enum ControlSpace {joint, cartesian} controlSpace;
	
		bool isFinished = false;                                                            // For regulating control actions
		
		double startTime, endTime;                                                          // For regulating the control loop
		
		double dt = 0.01;
		
		// Kinematics & dynamics
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
		Eigen::MatrixXd J, M, invM;                                                         // Jacobian, inertia, and inverse of inertia
		
		// Joint control
		double kp = 1.0;                                                                    // Default proportional gain
		double kd = 2*sqrt(this->kp);                                                       // Theoretically optimal damping
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // As it says
		
		// Cartesian control
		Eigen::Matrix<double,6,6> gainTemplate = (Eigen::MatrixXd(6,6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.1).finished();                    
		Eigen::Matrix<double,6,6> K = 1*this->gainTemplate;
		Eigen::Matrix<double,6,6> D = 2*this->gainTemplate;                                 // Theoretically optimal: kd >= 2*sqrt(kp)
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Trajectory generators for the hands
		Eigen::Isometry3d leftPose, rightPose;                                              // Left and right hand pose
		
		// Grasp control
		bool isGrasping = false;
		Eigen::Matrix<double,6,12> G;                                                       // Grasp matrix
		Eigen::Matrix<double,6,12> C;                                                       // Constraint matrix
		Payload payload;                                                                    // Class for representing object being held
		CartesianTrajectory payloadTrajectory;
		
	private:

		std::string _robotModel;                                                            // iCub2, iCub3, or ergoCub
		
		// Kinematic & dynamic modelling
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform _torsoPose;                                                     // Needed for inverse dynamics
		
		// Functions
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Converts from Eigen to iDynTree
		Eigen::Isometry3d   iDynTree_to_Eigen(const iDynTree::Transform &T);
		
		// NOTE: THESE FUNCTIONS MUST BE DECLARED IN ANY CHILD CLASS OF THIS ONE
		virtual bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum) = 0;
		virtual Eigen::VectorXd track_joint_trajectory(const double &time) = 0;
		virtual Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time) = 0;
	
		// NOTE: THESE FUNCTIONS ARE FROM THE PERIODICTHREAD CLASS AND NEED TO BE DEFINED
		// IN THE CHILD CLASSES OF THIS ONE
//		bool threadInit() { return true; }
//		void threadRelease() {}
//              void run() {}                                                                       // Defined in iCub2.h
};                                                                                                  // Semicolon needed after class declaration

#endif

/*
class iCubBase : public yarp::os::PeriodicThread,                                                   // Regulates the control loop
                 public JointInterface,                                                             // Communicates with motor controllers
                 public QPSolver                                                                    // Used to solve joint control
{
	public:
		iCubBase(const std::string              &pathToURDF,
		         const std::vector<std::string> &jointNames,
		         const std::vector<std::string> &portNames,
		         const Eigen::Isometry3d        &torsoPose,
		         const std::string              &robotName);
		
		
		bool is_finished() const { return this->isFinished; }
		
		bool set_cartesian_gains(const double &stiffness, const double &damping);
				       
		bool set_joint_gains(const double &proportional, const double &derivative);         // As it says on the label
		
		void halt();		                                                            // Stops the robot immediately
		
		// Joint Control Functions
		
		bool move_to_position(const Eigen::VectorXd &position,
		                      const double &time);

		bool move_to_positions(const std::vector<Eigen::VectorXd> &positions,               // Move joints through multiple positions
				       const std::vector<double> &times);
				       		
		// Cartesian Control Functions
		
		bool is_grasping() const { return this->isGrasping; }
		
		bool move_to_pose(const Eigen::Isometry3d &leftPose,
		                  const Eigen::Isometry3d &rightPose,
		                  const double &time);
		
		bool move_to_poses(const std::vector<Eigen::Isometry3d> &leftPoses,
		                   const std::vector<Eigen::Isometry3d> &rightPoses,
		                   const std::vector<double> &times);
		                   
		bool translate(const Eigen::Vector3d &left,                                         // Translate both hands by the given amount
		               const Eigen::Vector3d &right,
		               const double &time);
		
		bool move_object(const Eigen::Isometry3d &pose,
		                 const double &time);
		                 
		bool move_object(const std::vector<Eigen::Isometry3d> &poses,
		                 const std::vector<double> &times);
				       
		bool grasp_object(const Payload &_payload);

		bool release_object();
		               
		// Information
				       
		bool print_hand_pose(const std::string &whichHand);                                 // As it says on the label
		               
		Eigen::Isometry3d left_hand_pose()  const { return this->leftPose;  }
		
		Eigen::Isometry3d right_hand_pose() const { return this->rightPose; }

	protected:
	
		std::string name;                                                                   // As an internal reference
		
		Payload payload;                                                                    
	
		bool isGrasping = false;
		bool isFinished = false;
	
		double dt     = 0.01;                                                               // Default control frequency
		double hertz  = 1/this->dt;                                                         // Control frequency
		double maxAcc = 10;                                                                 // Limits the acceleration
		double startTime;                                                                   // Used for timing the control loop
		double endTime;                                                                     // Used for checking when actions are complete
		
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
		
		enum ControlSpace {joint, cartesian} controlSpace;
		
		// Joint control properties
		double kp = 10.0;                                                                   // Feedback on joint position error
		double kd =  5.0;                                                                   // Feedback on joint velocity error
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // Generates reference trajectories for joints
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Individual trajectories for left, right hand
		CartesianTrajectory payloadTrajectory;
		Eigen::Matrix<double,6,6> K;                                                        // Feedback on pose error
		Eigen::Matrix<double,6,6> D;                                                        // Feedback on velocity error
		Eigen::Matrix<double,6,6> gainTemplate;                                             // Structure for the Cartesian gains
		Eigen::MatrixXd J;                                                                  // Jacobian for both hands
		Eigen::MatrixXd M;                                                                  // Inertia matrix
		Eigen::MatrixXd Minv;                                                               // Inverse of the inertia
		Eigen::Isometry3d leftPose, rightPose;                                              // Pose of the left and right hands
		
		// Grasping
		Eigen::Matrix<double,6,12> G, C;                                                    // Grasp and constraint matrices
		
		// Kinematics & dynamics
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform          _torsoPose;                                            // Needed for inverse dynamics; not used yet
			                       	
		// Internal functions
				
		bool update_state();                                                                // Get new joint state, update kinematics
		
		Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
		                                     const Eigen::Isometry3d &actual);              // Get the error between 2 poses for feedback control
		                                     
		Eigen::Isometry3d iDynTree_to_Eigen(const iDynTree::Transform &T);                  // Convert iDynTree::Transform to Eigen::Isometry3d
		

		
		// Virtual functions to be overridden in derived class                            
		
		virtual void compute_joint_limits(double &lower, double &upper, const int &i) = 0;    
		virtual Eigen::VectorXd track_joint_trajectory(const double &time) = 0;                 // Solve feedforward + feedback control         
		virtual Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time) = 0;
		
		// Functions related to the PeriodicThread class
		bool threadInit() {return true;}                                                                  // Defined in PositionControl.h
		void threadRelease() {}                                                             // Defined in PositionContro.h
                void run() {}                                                                     // Defined in iCub2.h

};                                                                                                  // Semicolon needed after class declaration
*/
