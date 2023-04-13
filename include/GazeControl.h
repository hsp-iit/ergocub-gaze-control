#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations                                                                 // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>   
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops
// #include <yarp/os/RFModule.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include "JointInterface.h"
#include "QPSolver.h"

class GazeControl: public yarp::os::PeriodicThread
{
    private:
        JointInterface* jointInterface;
        QPSolver* solver;

        yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

        iDynTree::KinDynComputations computer;

        unsigned int numJoints;                                                             // Number of joints being controlled
        unsigned int numHeadJoints;
        // Utils
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Converts from Eigen to iDynTree
		Eigen::Isometry3d   iDynTree_to_Eigen(const iDynTree::Transform &T);                // Converts from iDynTree to Eigen

        // Main Control Loop
        Eigen::VectorXd redundantTask;

        // PositionControl
        bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		Eigen::Matrix<double,2,1> track_cartesian_trajectory(const double &time);
		Eigen::VectorXd track_joint_trajectory(const double &time);
        Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors
        bool threadInit();
		void threadRelease();

        // Joint control
		double kp = 1.0;                                                                    // Default proportional gain
		double kd = 2*sqrt(this->kp);                                                       // Theoretically optimal damping
		// std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // As it says
		

        enum ControlSpace {joint, cartesian} controlSpace;
        bool isFinished = false;                                                            // For regulating control actions
		double startTime, endTime;                                                          // For regulating the control loop
		double sample_time;

        Eigen::Matrix<double,2,1> pose_error(const Eigen::Vector3d &desired);                             
		 
        

    protected:
        // Kinematics & dynamics
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
        double image_width = 640;
        double image_height = 480;
        // Intrinsic Matrix
        Eigen::Matrix<double,3, 3> C = (Eigen::MatrixXd(3, 3) << 570.3422241210938, 0.0, 319.5,
                                                                 0.0, 570.3422241210938, 239.5,
                                                                 0.0, 0.0, 1.0).finished();
        Eigen::Matrix<double,6,4> J_R;                                                      // Camera Jacobian
        Eigen::Matrix<double,6,6> M = Eigen::MatrixXd::Zero(6,6);
        Eigen::Matrix<double,2,6> J_I = (Eigen::MatrixXd(2,6) << 1.0, 0.0, 0.0,  0.0, 1.0, 0.0,
                                                                 0.0, 1.0, 0.0, -1.0, 0.0, 0.0).finished();
        Eigen::Matrix<double,2,4> J;
        Eigen::Isometry3d cameraPose;                                                       // Camera pose
        Eigen::Vector3d desiredGaze = (Eigen::Vector3d() << 0.074927 + 0.1, -0.011469, 1.523281 - 0.9).finished();
        Eigen::Matrix<double, 2, 2> K;                                                      // Gain

	public:
		GazeControl(const std::string &pathToURDF,
                    const std::vector<std::string> &jointList,
                    const std::vector<std::string> &portList,
                    const double& sample_time);

		bool update_state();

        bool set_cartesian_gains(const double &proportional);
        bool set_joint_gains(const double &proportional, const double &derivative);
        
 
		bool set_gaze(const Eigen::Vector3d& desiredGaze);                        // Set the gaze


        void run() override;
        // double getPeriod() override;	

};
