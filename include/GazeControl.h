#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations
// #include <iDynTree/Model/Model.h>                                                                   // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>   

#include "JointInterface.h"
#include "QPSolver.h"
#include "PositionControl.h"

class GazeControl
{
    private:
        JointInterface* jointInterface;
        QPSolver* solver;
        PositionControl* positionControl;


        iDynTree::KinDynComputations computer;

        unsigned int numJoints;                                                             // Number of joints being controlled
        
        // Functions
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Converts from Eigen to iDynTree
		Eigen::Isometry3d   iDynTree_to_Eigen(const iDynTree::Transform &T);

    protected:
        // Kinematics & dynamics
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
        Eigen::MatrixXd J, M, invM; 
        Eigen::Isometry3d cameraPose;                                                       // Left and right hand pose

	public:
		GazeControl(const std::string &pathToURDF,
                    const std::vector<std::string> &jointList,
                    const std::vector<std::string> &portList);

		bool update_state();
        
};
