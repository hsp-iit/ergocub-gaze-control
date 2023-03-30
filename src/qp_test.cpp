    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                             Testing the grasp force solution with QP                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <QPSolver.h>
#include <time.h>

clock_t timer;
double T;

int main(int argc, char *argv[])
{
	// Variables used in this scope
	Eigen::Matrix<double,6,6>  Gleft, Gright;
	Eigen::Matrix<double,6,12> G;
	Eigen::Matrix<double,12,6> C;
	Eigen::Matrix<double,6,1>  fc;
	Eigen::Matrix<double,6,1>  minLeft, maxLeft, minRight, maxRight;
	Eigen::Matrix<double,12,1> f;
	
	Eigen::Vector3d rleft;  rleft  << 0.00,
	                                  0.05,
	                                  0.01;
	Eigen::Vector3d rright; rright << 0.01,
	                                 -0.05,
	                                  0.01;
	
	// Set up grasp matrices
	Gleft.block(0,0,3,3).setIdentity();
	Gleft.block(0,3,3,3).setZero();
	Gleft.block(3,0,3,3) <<        0, -rleft(2),  rleft(1),
	                        rleft(2),         0, -rleft(0),
	                       -rleft(1),  rleft(0),         0;
	Gleft.block(3,3,3,3).setIdentity();
	
	std::cout << "\nHere is the left grasp:\n" << std::endl;
	std::cout << Gleft << std::endl;
	
	Gright.block(0,0,3,3).setIdentity();
	Gright.block(0,3,3,3).setZero();
	Gright.block(3,0,3,3) <<        0, -rright(2), rright(1),
	                        rright(2),         0, -rright(0),
	                       -rright(1),  rright(0),         0;
	Gright.block(3,3,3,3).setIdentity();
	
	std::cout << "\nHere is the right grasp:\n" << std::endl;
	std::cout << Gright << std::endl;
	
	G.block(0,0,6,6) = Gleft;
	G.block(0,6,6,6) = Gright;
	
	// Construct the constraint matrix
	C.block(0,0,6,6) = Gleft.inverse();
	C.block(6,0,6,6) =-Gright.inverse();
	
	std::cout << "\nHere is G*C:\n" << std::endl;
	std::cout << G*C << std::endl;
	
	// Set up the constraints
	
	minLeft << -1e6, // x
	            -10, // y
	           -1e6, // z
	           -1e6, // roll
	           -1e6, // pitch
	           -1e6; // yaw
	
	maxLeft << 1e6, // x
		    -5, // y
		   1e6, // z
		   1e6, // roll
		   1e6, // pitch
		   1e6; // yaw
		   
	minRight << -1e6, // x
	               5, // y
	            -1e6, // z
	            -1e6, // roll
	            -1e6, // pitch
	            -1e6; // yaw
	
	maxRight << 1e6, // x
		     10, // y
		    1e6, // z
		    1e6, // roll
		    1e6, // pitch
		    1e6; // yaw	
	
	Eigen::Matrix<double,24,6> B;
	B.block( 0,0,12,6) = -C;
	B.block(12,0,12,6) =  C;
	
	Eigen::Matrix<double,24,1> c;
	c.block( 0,0,6,1) = -maxLeft;
	c.block( 6,0,6,1) = -maxRight;
	c.block(12,0,6,1) =  minLeft;
	c.block(18,0,6,1) =  minRight;
	
	Eigen::Matrix<double,6,1> f0; f0 << 0, -6, 0, 0, 0, 0;
	
//	std::cout << "\nHere is B*f0 - c:\n" << std::endl;
//	std::cout << B*f0 - c << std::endl;


	QPSolver solver;
	timer = clock();
	fc = solver.solve(C.transpose()*C,Eigen::VectorXd::Zero(6),B,c,f0);
	timer = clock() - timer;
	
	std::cout << "\nHere is fc:\n" << std::endl;
	std::cout << fc << std::endl;
	
	std::cout << "\nHere are the left hand and right hand forces:\n" << std::endl;
	Eigen::Matrix<double,6,2> comp;
	comp.col(0) = C.block(0,0,6,6)*fc;
	comp.col(1) = C.block(6,0,6,6)*fc;
	std::cout << comp << std::endl;
	
	T = (double)timer/CLOCKS_PER_SEC;
	std::cout << "\nIt took " << T*1000 << " ms to solve (" << 1/T << " Hz)." << std::endl;
		   

	return 0;                                                                                   // No problems with main
}
