#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<iostream>
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kuka.hpp"

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

sensor_msgs::JointState joints_phantom;
geometry_msgs::PoseStamped pos;
geometry_msgs::Twist ref;
double cam_roll;
double cam_pitch;
double cam_yaw;
double init_roll,init_pitch,init_yaw;
geometry_msgs::Twist phantom_joints_msg;
bool initialized_phantom;
bool start_loc_available=false;
trajectory_msgs::JointTrajectory cmd;
KDL::Rotation rpy;
// =============================================================================

// =============================================================================
// Subscriber Callbacks
// =============================================================================

//  Get phantom joints
void get_joints_phantom(const sensor_msgs::JointState & data) {
	double swap;
	for (int i = 0; i < 6;++i) {
    std::cout<<i;
		joints_phantom.position[i] = data.position[i];
	}
	joints_phantom.position[1]=joints_phantom.position[1]-0.82;
	joints_phantom.position[5]=-3.185-joints_phantom.position[5];
  joints_phantom.position[3]=3.115-joints_phantom.position[3];
	joints_phantom.position[4]=-2.248-joints_phantom.position[4];
	swap=joints_phantom.position[3];
	joints_phantom.position[3]=joints_phantom.position[5];
	joints_phantom.position[5]=swap;//3.1205;//swap;*/
	cam_roll = (joints_phantom.position[3]);
	cam_pitch = (joints_phantom.position[4]);
	cam_yaw = (joints_phantom.position[5]);
	initialized_phantom = true;
}

//Get Pose of PHANTOM
void get_pos(const geometry_msgs::PoseStamped & _data) {
	pos = _data;
}
// =============================================================================


int main(int argc, char **argv) {
  // ===========================================================================
  // ROS Publisher, Subrscribers, Node handle
  // ===========================================================================
  ros::init(argc, argv, "kuka");
  ros::Time::init();
  int loop_freq = 100;
  float dt = static_cast<float>(1/loop_freq);
  ros::Rate loop_rate(loop_freq);
  ROS_INFO_STREAM("Hi");
  kuka ku;
  ros::NodeHandle n("~");
  auto joints_sub = n.subscribe("/iiwa/joint_states", 10,
                                                      &kuka::getJoints, &ku);
  std::string command_topic =
                  "/iiwa/PositionJointInterface_trajectory_controller/command";
  ros::Publisher cmd_pub =
                n.advertise<trajectory_msgs::JointTrajectory>(command_topic, 1);
                //Define Subscriber to Read Phantom Joints
  ros::Subscriber joints_phantom_sub = n.subscribe("/phantom/joint_states",10, get_joints_phantom);
                //Define Subscriber to Read Phantom Pose
  ros::Subscriber pos_sub = n.subscribe("/phantom/pose",10, get_pos);
  // ===========================================================================
  KDL::Frame init_cartpos;
  KDL::Frame fin_cartpos;

  KDL::JntArray inv;
  ros::Duration(1).sleep();
  ros::spinOnce();
	ros::Duration(1).sleep();
  KDL::JntArray jointpositions_new;
  trajectory_msgs::JointTrajectoryPoint pt;
  trajectory_msgs::JointTrajectory cmd;
  cmd = ku.driveRobot(ku.initializeHomePos());
  cmd_pub.publish(cmd);
  ros::Duration(3).sleep();

  while (ros::ok()) {
    if (initialized_phantom) {
      if(!start_loc_available) {
        init_cartpos = ku.evalKinematicsFK();
        init_cartpos.M.GetRPY(init_roll,init_pitch, init_yaw);
        start_loc_available = true;
      } else {
        ref.linear.x = init_cartpos.p[0]+pos.pose.position.x;
        ref.linear.y = init_cartpos.p[1]+pos.pose.position.y;
        ref.linear.z = init_cartpos.p[2]+pos.pose.position.z;
        ref.angular.x = init_roll+cam_yaw;
        ref.angular.y = init_pitch+cam_pitch;
        ref.angular.z = init_yaw;
        // update the reference cartesian positions
        fin_cartpos.p[0]=ref.linear.x;
        fin_cartpos.p[1]=ref.linear.y;
        fin_cartpos.p[2]=ref.linear.z;
        rpy = KDL::Rotation::RPY(ref.angular.x,ref.angular.y,ref.angular.z);
        fin_cartpos.M = rpy;
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(fin_cartpos)));
        cmd_pub.publish(cmd);
      }
    }
		ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
