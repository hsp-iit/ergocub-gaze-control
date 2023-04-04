#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations                                                                 // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>   
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops

#include "JointInterface.h"
#include "QPSolver.h"
#include <CartesianTrajectory.h>

class GazeControl: public yarp::os::PeriodicThread
{
    private:
        JointInterface* jointInterface;
        QPSolver* solver;


        iDynTree::KinDynComputations computer;

        unsigned int numJoints;                                                             // Number of joints being controlled
        
        // Utils
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Converts from Eigen to iDynTree
		Eigen::Isometry3d   iDynTree_to_Eigen(const iDynTree::Transform &T);                // Converts from iDynTree to Eigen

        // Main Control Loop
        Eigen::VectorXd setPoint =
		(Eigen::VectorXd(4) << 0.0,  0.0,  0.00, 0.00).finished(); 
        void run();    

        // PositionControl
        bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		Eigen::Matrix<double,6,1> track_cartesian_trajectory(const double &time);
		Eigen::VectorXd track_joint_trajectory(const double &time);
        Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors
        bool threadInit();
		void threadRelease();

        // Joint control
		double kp = 1.0;                                                                    // Default proportional gain
		double kd = 2*sqrt(this->kp);                                                       // Theoretically optimal damping
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // As it says
		

        enum ControlSpace {joint, cartesian} controlSpace;
        bool isFinished = false;                                                            // For regulating control actions
		double startTime, endTime;                                                          // For regulating the control loop
		double dt = 0.01;

        Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
                                        const Eigen::Isometry3d &actual);                             
		 
		
        

    protected:
        // Kinematics & dynamics
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
        Eigen::MatrixXd J, M, invM;                                                         // Jacobian, inertia and its inverse
        Eigen::Isometry3d cameraPose;                                                       // Camera pose
        Eigen::Isometry3d desiredCameraPose;
        CartesianTrajectory cameraTrajectory;                                               // Trajectory generators for the hands
        Eigen::Matrix<double,6,6> K;                                                        // Feedback on pose error

	public:
		GazeControl(const std::string &pathToURDF,
                    const std::vector<std::string> &jointList,
                    const std::vector<std::string> &portList);

		bool update_state();

        bool set_cartesian_gains(const double &proportional);
        bool set_joint_gains(const double &proportional, const double &derivative);
        
        // Public Controller Interface
        // bool move_to_pose(const Eigen::Isometry3d &cameraPose,
		//                   const double &time);                                              // Move the gaze 
		                  
		bool move_to_pose(const Eigen::Isometry3d& desiredCameraPose,
                          const double& duration);                        // Move the gaze through several points
		                   
};
