#include<ros/ros.h>
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>



#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>


double x_d = 0.0;
double y_d = 0.0;
double z_d = 0.0;

double kp_x = 100.0;
double kd_x = 300.0;
double kp_y = 100.0;
double kd_y = 300.0;
double kp_z = 150.0;
double kd_z = 600.0;

//reading the joint positions for BOTH
sensor_msgs::JointState joints;
sensor_msgs::JointState joints_phantom;

// Take robot to initial position
/*void initialize_robot_start(trajectory_msgs::JointTrajectoryPoint & _pt)
{
	_pt.positions.push_back(111.3);
	_pt.positions.push_back(0);
	_pt.positions.push_back(20);
	_pt.positions.push_back(-100.57);
	_pt.positions.push_back(0);
	_pt.positions.push_back(1.57);
	_pt.positions.push_back(100);

}*/

// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init)
{
	for (int i = 0; i < _nj; ++i)
	{
		if(i==0)
			_pt.positions.push_back(1.3);//1.3);
		if(i==1)
			_pt.positions.push_back(0.0);//0.0);
		if(i==2)
			_pt.positions.push_back(0.0);//0.0);
		if(i==3)
			_pt.positions.push_back(-1.57);//-1.57);
		if(i==4)
			_pt.positions.push_back(0.0);//0.0);
		if(i==5)
			_pt.positions.push_back(1.57);//1.57);
		if(i==6)
			_pt.positions.push_back(100.0);
	}

}
// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions(i) = _init;
}

void initialize_joints(sensor_msgs::JointState & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions.position.push_back(_init);
}


//Read joint values LWR
bool initialized = false;
bool initialized_phantom = false;
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
double cam_roll,cam_pitch,cam_yaw;
void get_joints_phantom(const sensor_msgs::JointState & data)
{
	double swap;
	for (int i = 0; i < 6;++i)
	{
		joints_phantom.position[i] = data.position[i];
	}
	joints_phantom.position[1]=joints_phantom.position[1]-0.82;
	joints_phantom.position[3]=3.115-joints_phantom.position[3];
	joints_phantom.position[5]=-3.185-joints_phantom.position[5];
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
geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped & _data)
{
	pos = _data;

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



//////////   MAIN  //////////////
int main(int argc, char * argv[])
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
chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0,0,0.12597,0)));



//Define PHANTOM Chain
KDL::Chain chain_phantom;
//base
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH(0,0,0,0)));
//joint 1
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI_2,0,0)));
//joint 2
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.1321, 0.0,0.0,0)));
//joint 3
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI_2,0.0,0)));
//joint 4
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, -M_PI_2,0.1321,0)));
//joint 5 not considering the roll angle in th stylus (last DoF)
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,M_PI_2,0,0)));
chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,0.0,-0.05,0)));



//Define FK Solver for BOTH
KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
KDL::ChainFkSolverPos_recursive fksolver_phantom = KDL::ChainFkSolverPos_recursive(chain_phantom);


//Define IK Solver for LWR
KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6


//Define number of joints for BOTH
unsigned int nj = chain.getNrOfJoints();
unsigned int nj_phantom = chain_phantom.getNrOfJoints();


//Define Joint Arrays for Joint Positions for BOTH
KDL::JntArray jointpositions = KDL::JntArray(nj);
KDL::JntArray jointpositions_phantom = KDL::JntArray(nj_phantom);

//Define Joint Arrays for New Joint Positions for LWR
KDL::JntArray jointpositions_new = KDL::JntArray(nj);

//Initialize Joint Arrays for Joint Positions
initialize_joints(jointpositions, nj, 0.2);
initialize_joints(jointpositions_phantom, nj_phantom, 0.2);

////Initialize Joints to be Read
initialize_joints(joints, nj, 0.0);
initialize_joints(joints_phantom, nj_phantom, 0.0);

//Initialize ROS node
ros::init(argc, argv, "tele_op");
ros::NodeHandle nh_;

//Set loop rate
int loop_freq = 100;
float dt = (float) 1/loop_freq;
ros::Rate loop_rate(loop_freq);

//Define Joint Position Command Publisher
std::string command_topic = "/iiwa/PositionJointInterface_trajectory_controller/command";
ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);
ros::Publisher pub1 = nh_.advertise<std_msgs::Float64>("debug",10);
ros::Publisher rob_pose = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose",10);
geometry_msgs::PoseStamped fk_pose;
std_msgs::Float64 msg1;
msg1.data=1;
pub1.publish(msg1);

//Define Subscriber to Read LWR Joints
ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states",10, get_joints);

//Define Subscriber to Read Phantom Joints
ros::Subscriber joints_phantom_sub = nh_.subscribe("/phantom/joint_states",10, get_joints_phantom);

//Define Subscriber to Read Phantom Pose
ros::Subscriber pos_sub = nh_.subscribe("/phantom/pose",10, get_pos);

//Define joint_cmd: final joint command
//Define pt: current trajectory points
//Define pt2: new trajectory points
trajectory_msgs::JointTrajectory joint_cmd;
trajectory_msgs::JointTrajectoryPoint pt,pt2;

//Initialize pt, pt2
initialize_points(pt,nj,10.0);
initialize_points(pt2,nj,0.0);
//Define Current Cartesian Position of LWR
KDL::Frame current_cartpos;

//Define positions and orientations for Phantom input
double roll, pitch, yaw, x, y, z;
double roll_phantom, pitch_phantom, yaw_phantom;

//Define joint names

joint_cmd.joint_names.push_back("iiwa_joint_1");
joint_cmd.joint_names.push_back("iiwa_joint_2");
joint_cmd.joint_names.push_back("iiwa_joint_3");
joint_cmd.joint_names.push_back("iiwa_joint_4");
joint_cmd.joint_names.push_back("iiwa_joint_5");
joint_cmd.joint_names.push_back("iiwa_joint_6");
joint_cmd.joint_names.push_back("iiwa_joint_7");

//Define Cartesian Position KDL Frames for BOTH
KDL::Frame cartpos;
KDL::Frame cartpos_phantom;
//Define Roation Matrix for Roll,Pitch Yaw
KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw);
ros::Duration(5).sleep();

pt.time_from_start = ros::Duration(1.0);
joint_cmd.points.push_back(pt);
geometry_msgs::Twist ref;
geometry_msgs::Twist xyz;
geometry_msgs::Twist xyz_phantom;
geometry_msgs::Twist xyz_command;
msg1.data=2;
pub1.publish(msg1);
//initialize robot to start pose
//initialize_robot_start(pt);
joint_cmd.points[0] = pt;
/*	for(int j = 0; j<1500000; ++j)
	{
		cmd_pub.publish(joint_cmd);
	}
*/
bool kinematics_status;
bool start_loc_available = false;
bool all_zero = true;

	while (ros::ok())
	{
		if (initialized)
		{
			if(initialized_phantom)
			{
				//Read Joint Positions of Phantom and put them in KDL format
				for (int k = 0; k<nj_phantom; ++k)
				{
					jointpositions_phantom(k) = joints_phantom.position[k];
				}
				//jointpositions_phantom(6)=3.1205;

				//Perform Forward Kinematics To get Cartesian Pose of PHANTOM
				kinematics_status = fksolver_phantom.JntToCart(jointpositions_phantom,cartpos_phantom);
				if(kinematics_status>=0)
				{
					xyz_phantom.linear.x = cartpos_phantom.p[0];
					xyz_phantom.linear.y = cartpos_phantom.p[1];
					xyz_phantom.linear.z = cartpos_phantom.p[2];
					//Get roll, pitch, yaw values
					cartpos_phantom.M.GetRPY(roll_phantom,pitch_phantom, yaw_phantom);
					xyz_phantom.angular.x = roll_phantom;
					xyz_phantom.angular.y = pitch_phantom;
					xyz_phantom.angular.z = yaw_phantom;
				}

				// Get LWR joint positions in KDL form
				for (int k = 0; k<7; ++k)
				{
					jointpositions(k) = joints.position[k];

				}
				msg1.data=3;
				pub1.publish(msg1);

				//Perform Forward Kinematics to get Cartesian Pose of LWR
				msg1.data=4;
				pub1.publish(msg1);
				//find where the robot is
				if(!start_loc_available) //if initial position is not known
				{

					msg1.data=5;
					pub1.publish(msg1);
					kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
					if(kinematics_status>=0)
					{

						xyz.linear.x = cartpos.p[0];
						xyz.linear.y = cartpos.p[1];
						xyz.linear.z = cartpos.p[2];
						cartpos.M.GetRPY(roll,pitch, yaw);
						xyz.angular.x = roll;
						xyz.angular.y = pitch;
						xyz.angular.z = yaw;
						}

					start_loc_available = true;
					}
				else
				{
					msg1.data=6;
					pub1.publish(msg1);
					double z_gain = 1;
					xyz_command.linear.x = xyz_phantom.linear.x;
					xyz_command.linear.y = xyz_phantom.linear.y;
					xyz_command.linear.z = xyz_phantom.linear.z*z_gain;
					ref.linear.x = xyz.linear.x + xyz_command.linear.x;
					ref.linear.y = xyz.linear.y + xyz_command.linear.y;
					ref.linear.z = xyz.linear.z + xyz_command.linear.z;
					//keep the same orientation
					ref.angular.x = xyz.angular.x+cam_roll;
					ref.angular.y = xyz.angular.y+cam_pitch;//+xyz_phantom.angular.y;//0.002;
					ref.angular.z = xyz.angular.z;//+cam_yaw;//+xyz_phantom.angular.z;//1.8415'
					// update the reference cartesian positions
					cartpos.p[0]=ref.linear.x;
					cartpos.p[1]=ref.linear.y;
					cartpos.p[2]=ref.linear.z;
					rpy = KDL::Rotation::RPY(ref.angular.x,ref.angular.y,ref.angular.z);
					cartpos.M = rpy;
					int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
					eval_points(pt, jointpositions_new, nj);
					pt.time_from_start = ros::Duration(dt);
					joint_cmd.points[0] = pt;
					joint_cmd.header.stamp = ros::Time::now();
					joint_cmd.header.seq = 0;
					joint_cmd.header.frame_id = "";
					msg1.data=5;
					pub1.publish(msg1);
					cmd_pub.publish(joint_cmd);
				}
			}
		}
		else
		{
			cmd_pub.publish(joint_cmd);
			msg1.data=9;
			pub1.publish(msg1);

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
