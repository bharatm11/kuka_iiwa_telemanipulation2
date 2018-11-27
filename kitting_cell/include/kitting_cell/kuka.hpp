/**
* @file kuka.hpp
* @author Bharat Mathur
* @date 27 Nov 2018
* @copyright 2018 Bharat Mathur
* @brief This file defines the methods for class "kuka" to control a KUKA IIWA
* manipulator using on IIWA_STACK using OROCOS KDL
*/
#ifndef KUKA_H_
#define KUKA_H_

#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/**
* @brief Class to calculate forward and inverse kinematics for KUKA IIWA
*/

class kuka {
private:
  sensor_msgs::JointState joints_state; ///< sensor_msgs::JointState tye
                                        ///< variable to read current joint states
  KDL::Chain kinematicChain;                     ///< KDL::Chain type vaiable to define
                                        ///< kinematic chain
  KDL::ChainFkSolverPos_recursive fksolver  ///< forward kinematic solver object
  KDL::ChainIkSolverVel_pinv iksolverv  ///< inverse kinematics solver velocity
                                        /// <object
  KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6
  unsigned int numJoints;  ///< unsigned int variable to hold number of
                          ///< kinematic joints
  KDL::JntArray jointPosKdl;  ///< KDL joint array current for FK
  KDL::JntArray newJointPosKdl;  ///< KDL joint array new from IK
  KDL::Frame currCartpos;  ///< current cartesian pose in from FK
  trajectory_msgs::JointTrajectory jointCommands; ///< final motion commads sent
                                                  ///< to the robot
  bool kinematics_status;  ///< verify solver status

public:
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/

  kuka();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
void initializeTrajectoryPoint()
  void initializePoints()
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void initializeJointsSub();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void initializeJointsKDL();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void defineFKSolver();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void defineIKSolver();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void defineIKVSolver();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  void make_chain();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  trajectory_msgs::JointTrajectoryPoint normalizePoints(KDL::JntArray);
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  bool checkKinematicStatus();
  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
  getJoints(const sensor_msgs::JointState::ConstPtr& joints)

  /**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
KDL::JntArray evalKinematicsIK(KDL::Frame, KDL::JntArray)

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
KDL::Frame evalKinematicsFK(KDL::JntArray)

/**
* @brief <brief>
* @param [in] <name> <parameter_description>
* @return <return_description>
* @details <details>
*/
void getJointNums();
};







#endif /* KUKA_H_ */
