/*!
* @file 	main.cpp
* @author 	Michael Blösch
* @date		10.10.2012
* @brief	
 */

#include <Eigen/Dense>
#include "LSE/Manager.hpp"

// Kinematics of robot
Eigen::Vector3d legKin(Eigen::Matrix<double,LSE_DOF_LEG,1> a,int i){
	// Todo: include the parameters into the function call -> later used for robot calibration
	double bx = 0.2525;
	double by = 0.185;
	double lH = -0.0685;
	double lT = -0.2;
	double lS = -0.235;

	Eigen::Vector3d s;
	s(0) = ((i<2)*2-1)*bx+lT*sin(a(1))+lS*sin(a(1)+a(2));
	s(1) = -((i%2)*2-1)*by-sin(a(0))*(lH+lT*cos(a(1))+lS*cos(a(1)+a(2)));
	s(2) = cos(a(0))*(lH+lT*cos(a(1))+lS*cos(a(1)+a(2)));
	return s;
}

// Kinematic Jacobian of robot
Eigen::Matrix<double,3,LSE_DOF_LEG> legKinJac(Eigen::Matrix<double,LSE_DOF_LEG,1> a,int i){
	double lH = -0.0685;
	double lT = -0.2;
	double lS = -0.235;

	Eigen::Matrix<double,3,LSE_DOF_LEG> J;
	J.setZero();
	J(0,1) = lS*cos(a(1)+a(2))+lT*cos(a(1));
	J(0,2) = lS*cos(a(1)+a(2));
	J(1,0) = -cos(a(0))*(lH+lT*cos(a(1))+lS*cos(a(1)+a(2)));
	J(1,1) = sin(a(0))*(lT*sin(a(1))+lS*sin(a(1)+a(2)));
	J(1,2) = lS*sin(a(0))*sin(a(1)+a(2));
	J(2,0) = -sin(a(0))*(lH+lT*cos(a(1))+lS*cos(a(1)+a(2)));
	J(2,1) = -cos(a(0))*(lT*sin(a(1))+lS*sin(a(1)+a(2)));
	J(2,2) = -lS*cos(a(0))*sin(a(1)+a(2));
	return J;
}

// Init State Estimate
LSE::Manager* pManager_;
string fileName = "Parameters.xml";
pManager_ = new LSE::Manager(fileName.c_str(),&legKin,&legKinJac);
double time;
pManager_->resetEstimate(time);

// Send Measurements and update
LSE::ImuMeas imuMeas;
double time;
pManager_->addImuMeas(time,imuMeas);
LSE::EncMeas encMeas;
double time;
pManager_->addEncMeas(time,encMeas);
pManager_->update();

// Reset Estimate
double time;
pManager_->resetEstimate(time);

// Get State Estimate
LSE::State x;
x = pManager_->getEst();




// Finish
delete pManager_;
