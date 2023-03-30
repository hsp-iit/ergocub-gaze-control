#include <Payload.h>

void Payload::update_state(const Eigen::Isometry3d &globalToLocal,
                           const Eigen::Matrix<double,6,1> &twist)
{
	this->_globalPose = globalToLocal*this->_localPose;
}

/*
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    CONSTRUCTOR                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
Payload::Payload(const Eigen::Isometry3d &_localPose,
		 const double &_mass,
	         const double &Ixx,
	         const double &Ixy,
	         const double &Ixz,
	         const double &Iyy,
	         const double &Iyz,
	         const double &Izz) :
	         localPose(_localPose),
	         mass(_mass)
{
	this->localInertia << Ixx, Ixy, Ixz,
	                      Ixy, Iyy, Iyz,
	                      Ixz, Iyz, Izz;
	                      
	check_inertia();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Ensure that the inertia properties are sound on construction                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Payload::check_inertia()
{
	// Ensure that the mass is positive
	if( this->mass < 0 )
	{
		std::cout << "[ERROR] [PAYLOAD] Constructor(): "
		          << "Mass is " << this->mass << " but cannot be negative!" << std::endl;
		
		this->mass *= -1;                                                                   // Ensure positive
	}
	
	// Ensure inertia matrix is positive definite
	if( (this->localInertia - this->localInertia.transpose()).norm() > 1e-03 )
	{
		std::cerr << "[ERROR] [PAYLOAD] Constructor(): "
		          << "The inertia matrix is not symmetric!" << std::endl;
		          
		
		this->localInertia = ( this->localInertia + this->localInertia.transpose() ) / 2;   // Force the inertia to be positive definite and symmetric
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Set new state, update dynamics                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Payload::update_state(const Eigen::Isometry3d &contactPointTransform,
                           const Eigen::Matrix<double,6,1> &contactPointVelocity)
{
	this->globalPose = contactPointTransform * this->localPose;
	
	// Velocity argument is for the external contact point,
	// so we need to transform it to the center of the payload
	
	Eigen::Vector3d r = this->globalPose.translation() - contactPointTransform.translation();   // Translation to center from contact point
	Eigen::Vector3d w = contactPointVelocity.tail(3);                                           // Angular velocity component
	
	this->twist.head(3) = contactPointVelocity.head(3) + w.cross(r);
	this->twist.tail(3) = w;
	
	// Rotate the inertia matrix to the new coordinates
	
	Eigen::Matrix<double,3,3> R = this->globalPose.rotation();
	this->globalInertia = R*this->localInertia*R.transpose();
	
	// Update the time derivative of the inertia matrix
	// Idot = skew(w)*I, where w is the angular velocity.
	// We can skip the zeros along the diagonal by solving manually.

	this->inertiaDerivative << this->twist(4)*this->globalInertia(2,0) - this->twist(5)*this->globalInertia(1,0),
	                           this->twist(4)*this->globalInertia(2,1) - this->twist(5)*this->globalInertia(1,1),
	                           this->twist(4)*this->globalInertia(2,2) - this->twist(5)*this->globalInertia(1,2),
	                           
	                           this->twist(5)*this->globalInertia(0,0) - this->twist(3)*this->globalInertia(2,0),
	                           this->twist(5)*this->globalInertia(0,1) - this->twist(3)*this->globalInertia(2,1),
	                           this->twist(5)*this->globalInertia(0,2) - this->twist(3)*this->globalInertia(2,2),
	                           
	                           this->twist(3)*this->globalInertia(1,0) - this->twist(4)*this->globalInertia(0,0),
	                           this->twist(3)*this->globalInertia(1,1) - this->twist(4)*this->globalInertia(0,1),
	                           this->twist(3)*this->globalInertia(1,2) - this->twist(4)*this->globalInertia(0,2);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Compute dynamic forces at the center of the object                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> Payload::get_centroid_forces(const Eigen::Matrix<double,6,1> &acc)
{
	Eigen::Matrix<double,6,1> f;                                                                // Value to be returned
	
	f.head(3) = this->mass*acc.head(3);                                                         // m x a
	f.tail(3) = this->globalInertia*acc.tail(3) + this->inertiaDerivative*this->twist.tail(3);  // I x a + Idot x v
	
	return f;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute dynamic forces at the contact point                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> Payload::get_contact_point_forces(const Eigen::Matrix<double,6,1> &acc,
                                                            const Eigen::Vector3d &point)
{
	Eigen::Matrix<double,6,1> f;                                                                // Value to be returned
	
	Eigen::Matrix<double,6,1> w = get_centroid_forces(acc);
	f.head(3) = w.head(3);
	
	Eigen::Vector3d r = this->globalPose.translation() - point;
	f.tail(3) << r(1)*w(2) - r(2)*w(1) + w(3),
	             r(2)*w(0) - r(0)*w(2) + w(4),
	             r(0)*w(1) - r(1)*w(0) + w(5);
	             
	return f;
}*/
