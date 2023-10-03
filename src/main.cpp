/*!
* @file 	main.cpp
* @author 	Michael Blösch
* @date		10.10.2012
* @brief	Simple ros-node including the LSE library
 */

#include <Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "Manager.hpp"

using namespace std;

// Global Variables
bool gotFirstMeas_;
LSE::Manager* pManager_;
ros::Publisher poseEst;
ros::Publisher velEst;
ros::Publisher rrEst;

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

// Publishes the filtered data into ros topics
void plotResults(){
	LSE::State x;
	x = pManager_->getEst();
	geometry_msgs::PoseStamped msgPose;
	geometry_msgs::Vector3Stamped msgVec;

	// Build and publish pose message
	msgPose.header.seq = 0;
	msgPose.header.stamp = ros::Time(x.t_);
	msgPose.header.frame_id = "/W";
	msgPose.pose.position.x = x.r_(0);
	msgPose.pose.position.y = x.r_(1);
	msgPose.pose.position.z = x.r_(2);
	msgPose.pose.orientation.x = -x.q_(0);
	msgPose.pose.orientation.y = -x.q_(1);
	msgPose.pose.orientation.z = -x.q_(2);
	msgPose.pose.orientation.w = x.q_(3);
	poseEst.publish(msgPose);

	// Build and publish velocity message
	msgVec.header.seq = 0;
	msgVec.header.stamp = ros::Time(x.t_);
	msgVec.header.frame_id = "/W";
	msgVec.vector.x = x.v_(0);
	msgVec.vector.y = x.v_(1);
	msgVec.vector.z = x.v_(2);
	velEst.publish(msgVec);

	// Build and publish velocity message
	msgVec.header.seq = 0;
	msgVec.header.stamp = ros::Time(x.t_);
	msgVec.header.frame_id = "/B";
	msgVec.vector.x = x.w_(0);
	msgVec.vector.y = x.w_(1);
	msgVec.vector.z = x.w_(2);
	rrEst.publish(msgVec);
}

// Callback functions
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(!gotFirstMeas_){
		pManager_->resetEstimate(msg->header.stamp.toSec());
		gotFirstMeas_ = true;
	}

	LSE::ImuMeas imuMeas;
	imuMeas.f_(0) = msg->linear_acceleration.x;
	imuMeas.f_(1) = msg->linear_acceleration.y;
	imuMeas.f_(2) = msg->linear_acceleration.z;
	imuMeas.w_(0) = msg->angular_velocity.x;
	imuMeas.w_(1) = msg->angular_velocity.y;
	imuMeas.w_(2) = msg->angular_velocity.z;
	pManager_->addImuMeas(msg->header.stamp.toSec(),imuMeas);
}
void encCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(!gotFirstMeas_){
		pManager_->resetEstimate(msg->header.stamp.toSec());
		gotFirstMeas_ = true;
	}

	LSE::EncMeas encMeas;
	for(int i=0;i<LSE_N_LEG;i++){
		for(int j=0;j<LSE_DOF_LEG;j++){
			encMeas.e_(j,i) = msg->position[i*LSE_DOF_LEG+j];
		}
//		for(int j=0;j<LSE_DOF_LEG;j++){
//			encMeas.v_(j,i) = msg->velocity[i*LSE_DOF_LEG+j];
//		}
		encMeas.CF_[i] = (int)msg->effort[i*LSE_DOF_LEG];
	}
	pManager_->addEncMeas(msg->header.stamp.toSec(),encMeas);
	pManager_->update();
	plotResults();
}
void posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!gotFirstMeas_){
		pManager_->resetEstimate(msg->header.stamp.toSec());
		gotFirstMeas_ = true;
	}

	LSE::PosMeas posMeas;
	posMeas.r_(0) = msg->pose.position.x;
	posMeas.r_(1) = msg->pose.position.y;
	posMeas.r_(2) = msg->pose.position.z;
	posMeas.q_(0) = msg->pose.orientation.x;
	posMeas.q_(1) = msg->pose.orientation.y;
	posMeas.q_(2) = msg->pose.orientation.z;
	posMeas.q_(3) = msg->pose.orientation.w;
	pManager_->addPosMeas(msg->header.stamp.toSec(),posMeas);
}

int main (int argc, char **argv)
{
	// ROS init stuff
	ros::init(argc, argv, "LSE_test");
	ros::NodeHandle nh;

	string fileName = "Parameters.xml";

	for(int i=0;i<argc;i++){
		if (strcmp(argv[i],"-f")==0) {
			fileName = argv[i+1];
		}
	}

	// LSE
	pManager_ = new LSE::Manager(fileName.c_str(),&legKin,&legKinJac);
	pManager_->resetEstimate(0);
	// pManager_->setSamplingTime(0.0025);
	gotFirstMeas_ = false;

	// Publishers
	poseEst = nh.advertise<geometry_msgs::PoseStamped>("/LSE/filter/pose", 1000);
	velEst = nh.advertise<geometry_msgs::Vector3Stamped>("/LSE/filter/velocity", 1000);
	rrEst = nh.advertise<geometry_msgs::Vector3Stamped>("/LSE/filter/rotrate", 1000);

	// Subscribers
	ros::Subscriber imuSub = nh.subscribe("/MeasLoader/imuMeas", 1000, imuCallback);
	ros::Subscriber encSub = nh.subscribe("/MeasLoader/encMeas", 1000, encCallback);
	ros::Subscriber posSub = nh.subscribe("/MeasLoader/posMeas", 1000, posCallback);

	// ROS main loop
	ros::spin();

	delete pManager_;
	cout << "Finished" << endl;
	return 0;
}
