#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<ros/ros.h>
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/WrenchStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
	_jointpositions(i) = _init;
}

void initialize_joints(sensor_msgs::JointState & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
	_jointpositions.position.push_back(_init);
}

// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init)
{
	for (int i = 0; i < _nj; ++i)
	{
		if(i==0)
		_pt.positions.push_back(0);//1.3);
		if(i==1)
		_pt.positions.push_back(1.0);//0.0);
		if(i==2)
		_pt.positions.push_back(1.0);//0.0);
		if(i==3)
		_pt.positions.push_back(-1.57);//-1.57);
		if(i==4)
		_pt.positions.push_back(0.0);//0.0);
		if(i==5)
		_pt.positions.push_back(1.0);//1.57);
		if(i==6)
		_pt.positions.push_back(0);
	}
}

//Read joint values LWR
bool initialized = false;
sensor_msgs::JointState joints;
void get_joints(const sensor_msgs::JointState & data)
{
	for (int i = 0; i < data.position.size();++i)
	{
		// if this is not the first time the callback function is read, obtain the joint positions
		if(initialized){
			joints.position[i] = data.position[i];
			// otherwise initilize them with 0.0 values
		}
	}
	initialized = true;
}

void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj){
	// joints can move between -+: 172,120,172,120,172,120,170
	//double joint_bounds[] = {3.002, 2.0944,3.002, 2.0944,3.002, 2.0944, 3.002};
	for (int i = 0; i < _nj; ++i){
		while(_jointpositions(i) > M_PI)
		_jointpositions(i) -= 2*M_PI;
		while(_jointpositions(i) < -M_PI)
		_jointpositions(i) += 2*M_PI;
		_point.positions[i] = _jointpositions(i);
	}
}
bool kinematics_status;
int main(int argc, char ** argv)
{
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
	//chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,0,0.12597,0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,0,0.37897,0)));

	//Define FK Solver
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	//Define IK Solver for LWR
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6

	//Define number of joints
	unsigned int nj = chain.getNrOfJoints();

	//Define Joint Arrays for Joint Positions
	KDL::JntArray jointpositions = KDL::JntArray(nj);
	//Define Joint Arrays for New Joint Positions for LWR
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	//Initialize Joint Arrays for Joint Positions
	initialize_joints(jointpositions, nj, 0.2);

	////Initialize Joints to be Read
	initialize_joints(joints, nj, 0.0);

	ros::init(argc, argv, "tool_tip_broadcaster");
	ros::NodeHandle nh_broadcaster;
	int loop_freq = 100;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);
	ros::Subscriber joints_sub = nh_broadcaster.subscribe("/iiwa/joint_states",10, get_joints);
	//Initialize pt, pt2
	trajectory_msgs::JointTrajectoryPoint pt,pt2;
	initialize_points(pt,nj,0.2);
	initialize_points(pt2,nj,0.0);
	KDL::Frame cartpos;
	double roll,pitch,yaw;
	//Define Roation Matrix for Roll,Pitch Yaw
	KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw);


	// Camera_broadcasater
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3( -0.0649, 0.0656, -0.0751));
	tf::Quaternion q;
	q.setRPY(-0.0631, -0.0843 , -2.3893);
	transform.setRotation(q);

	tf::TransformBroadcaster br2;
	tf::Transform transform2;



	while(ros::ok())
	{
		ros::spinOnce();
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "iiwa_link_ee", "camera_link_calibrated"));
		//Read Joint Positions of IIWA and put them in KDL format
		for (int k = 0; k<7; ++k)
		{
			jointpositions(k) = joints.position[k];

		}
		kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
		if(kinematics_status>=0)
		{

			tf::transformKDLToTF(cartpos, transform2);
		}
		br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "US_probe_tip222"));
		loop_rate.sleep();
		ros::spinOnce();
	}



	return 0;
}
