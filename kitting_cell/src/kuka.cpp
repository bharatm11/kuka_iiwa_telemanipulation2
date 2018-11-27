/**
* @file kuka.cpp
* @author Bharat Mathur
* @date 27 Nov 2018
* @copyright 2018 Bharat Mathur
* @brief This file implements the methods for class "kuka"
* This class cpp file defines data members and methods applicable for class
* kuka to to control a KUKA IIWA manipulator using on IIWA_STACK using
* OROCOS KDL for forward and inverse kinematics
*/

#include "kuka.hpp"

void kuka::make_chain() {
  //Define LWR Chain
  KDL::Chain chain;
  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(0,0,0.33989,0)));
  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));
  //joint 2
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,M_PI_2,0.40011,0)));
  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));
  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, -M_PI_2,0.40003,0)));
  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));
  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, M_PI_2,0,0)));
  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,0,0.12597,0)));
  this->kinematicChain = chain;
}

void kuka::getJointNums() {
  this->numJoints = this->kinematicChain.getNrOfJoints();
}

void kuka::initializeTrajectoryPoint() {
  this->joint_cmd.joint_names.push_back("iiwa_joint_1");
  this->joint_cmd.joint_names.push_back("iiwa_joint_2");
  this->joint_cmd.joint_names.push_back("iiwa_joint_3")
  this->joint_cmd.joint_names.push_back("iiwa_joint_4");
  this->joint_cmd.joint_names.push_back("iiwa_joint_5");
  this->joint_cmd.joint_names.push_back("iiwa_joint_6");
  this->joint_cmd.joint_names.push_back("iiwa_joint_7");
  this->joint_cmd.header.seq = 0;
  this->joint_cmd.header.stamp=ros::Time::now();
  this->joint_cmd.header.frame_id = "";
}
void kuka::initializePoints() {
  for (int i = 0; i < this->numJoints; ++i)
  {
    if(i==0)
    this->joint_cmd.positions.push_back(0);//1.3);
    if(i==1)
    this->joint_cmd.positions.push_back(1.0);//0.0);
    if(i==2)
    this->joint_cmd.positions.push_back(1.0);//0.0);
    if(i==3)
    this->joint_cmd.positions.push_back(-1.57);//-1.57);
    if(i==4)
    this->joint_cmd.positions.push_back(0.0);//0.0);
    if(i==5)
    this->joint_cmd.positions.push_back(1.0);//1.57);
    if(i==6)
    this->joint_cmd.positions.push_back(0);
  }
}

kuka::kuka() {
  this->fksolver = KDL::ChainFkSolverPos_recursive(this->kinematicChain);
  this->iksolverv = KDL::ChainIkSolverVel_pinv(this->kinematicChain);
  KDL::ChainIkSolverPos_NR IKsolver(this->kinematicChain,this->fksolver,
                                                    this->iksolverv,100,1e-4);
                                //Maximum 100 iterations, stop at accuracy 1e-6
  this->iksolver = IKsolver;
}
