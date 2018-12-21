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


#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
sensor_msgs::JointState joints_phantom;
sensor_msgs::JointState joints;

double cam_yaw=0.0;
double cam_pitch=0.0;
double cam_roll=0.0;
geometry_msgs::Twist phantom_joints_msg;
//Get Joint values of PHANTOM
void get_joints_phantom(const sensor_msgs::JointState & data) {
	double swap;
	for (int i = 0; i < 6;++i) {
		joints_phantom.position[i] = data.position[i];
	}
	joints_phantom.position[1]=joints_phantom.position[1]-0.82;
	joints_phantom.position[3]=3.115-joints_phantom.position[3];
	joints_phantom.position[5]=-3.185-joints_phantom.position[5];
	joints_phantom.position[4]=-2.248-joints_phantom.position[4];
	swap=joints_phantom.position[3];
	joints_phantom.position[3]=joints_phantom.position[5];
	joints_phantom.position[5]=swap;//3.1205;//swap;*/
	cam_roll = (joints_phantom.position[3])*0.01;
	cam_pitch = (joints_phantom.position[4])*0.05;
	cam_yaw = (joints_phantom.position[5])*0.005;
	phantom_joints_msg.angular.x=joints_phantom.position[3];
	phantom_joints_msg.angular.y=joints_phantom.position[4];
	phantom_joints_msg.angular.z=joints_phantom.position[5];
	initialized_phantom = true;
}
//Get Pose of PHANTOM
geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped & _data) {
	pos = _data;
}

//////////   MAIN  //////////////
int main(int argc, char * argv[]) {
	KDL::Chain chain_phantom;
	//base
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH(0,0,0,0)));
	//joint 1
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI_2,0,0)));
	//joint 2
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.157508, 0.0,0.0,0)));//127508
	//joint 3
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI_2,0,0)));//0.149352
	//joint 4
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, -M_PI_2,0.139352,0)));
	//joint 5 not considering the roll angle in th stylus (last DoF)
	chain_phantom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0,M_PI_2,0,0)));

	KDL::ChainFkSolverPos_recursive fksolver_phantom = KDL::ChainFkSolverPos_recursive(chain_phantom);

	//Define number of joints for BOTH
	unsigned int nj_phantom = chain_phantom.getNrOfJoints();

	//Initialize ROS node
	ros::init(argc, argv, "tele_op");
	ros::NodeHandle nh_;

	//Set loop rate
	int loop_freq = 100;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);
	ROS_INFO_STREAM("Start tele_op");
	//Define Joint Position Command Publisher
	std::string command_topic = "/iiwa/PositionJointInterface_trajectory_controller/command";
	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,dt);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::Transform inv_transform;
	tf::TransformListener listener2;
	tf::StampedTransform transform2;
	tf::Transform inv_transform2;

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
	//Define Current Cartesian Position of LWR
	KDL::Frame current_cartpos;
	KDL::Frame cartpos_new;
	//Define positions and orientations for Phantom input
	double roll, pitch, yaw, x, y, z;
	double roll_phantom, pitch_phantom, yaw_phantom;
	double roll_phantom_init, pitch_phantom_init, yaw_phantom_init;
	double roll_iiwa_init, pitch_iiwa_init, yaw_iiwa_init;
	std_msgs::Int16 ret_solver;

	//Define Cartesian Position KDL Frames for BOTH
	KDL::Frame cartpos;
	KDL::Frame cartpos_iiwa_init;
	KDL::Frame cartpos_phantom;
	//Define Roation Matrix for Roll,Pitch Yaw
	KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw);

	geometry_msgs::Twist ref;
	geometry_msgs::Twist xyz;
	geometry_msgs::Twist xyz_new;
	geometry_msgs::Twist xyz_phantom;
	geometry_msgs::Twist xyz_phantom_init;
	geometry_msgs::Twist xyz_iiwa_init;
	geometry_msgs::Twist xyz_command;
	double roll_new, pitch_new, yaw_new;

	//initialize robot to start pose
	//initialize_robot_start(pt);
	ros::spinOnce();


	bool start_loc_available = false;
	bool all_zero = true;
	// Read Initial Transform For Initial IIWA Cartesian
	try{
		listener.lookupTransform("/US_probe_tip222", "/world", ros::Time(0), transform);
		inv_transform = transform.inverse();
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	tf::Vector3 iiwa_pos_in_camera, iiwa_pos_in_world;
	tf::Vector3 iiwa_prev_in_camera, iiwa_prev_in_world;
	tf::Vector3 iiwa_init_pos_in_camera, iiwa_init_pos_in_world;
	tf::Quaternion iiwa_rot_in_cam;
	tf::Quaternion phantom_rot_in_cam;
	tf::Quaternion phantom_rot_in_world;
	tf::Quaternion iiwa_init_rot_in_cam;
	tf::Quaternion iiwa_init_rot_in_world;
	double phantom_roll_in_cam,phantom_pitch_in_cam,phantom_yaw_in_cam;
	double iiwa_init_roll_in_cam = 0.0;
	double iiwa_init_pitch_in_cam = 0.0;
	double iiwa_init_yaw_in_cam = 0.0;

	double iiwa_roll_in_cam = 0.0;
	double iiwa_pitch_in_cam = 0.0;
	double iiwa_yaw_in_cam = 0.0;

	if(kinematics_status>=0)
	{
		xyz_phantom_init.linear.x = pos.pose.position.x;//cartpos_phantom.p[0];
		xyz_phantom_init.linear.y = pos.pose.position.y;//cartpos_phantom.p[1];
		xyz_phantom_init.linear.z = pos.pose.position.z;//cartpos_phantom.p[2];
		//Get roll, pitch, yaw values
		cartpos_phantom.M.GetRPY(roll_phantom_init, pitch_phantom_init, yaw_phantom_init);
		xyz_phantom_init.angular.x = roll_phantom_init;
		xyz_phantom_init.angular.y = pitch_phantom_init;
		xyz_phantom_init.angular.z = yaw_phantom_init;

	}
	ros::spinOnce();
	//Read Joint Positions of IIWA and put them in KDL format
	for (int k = 0; k<7; ++k)
	{
		jointpositions(k) = joints.position[k];

	}
	//Perform Forward Kinematics to get Cartesian Pose of LWR
	kinematics_status = fksolver.JntToCart(jointpositions,cartpos_iiwa_init);
	if(kinematics_status>=0)
	{

		xyz_iiwa_init.linear.x = cartpos_iiwa_init.p[0];
		xyz_iiwa_init.linear.y = cartpos_iiwa_init.p[1];
		xyz_iiwa_init.linear.z = cartpos_iiwa_init.p[2];
		cartpos_iiwa_init.M.GetRPY(roll_iiwa_init, pitch_iiwa_init, yaw_iiwa_init);
		xyz_iiwa_init.angular.x = roll_iiwa_init;
		xyz_iiwa_init.angular.y = pitch_iiwa_init;
		xyz_iiwa_init.angular.z = yaw_iiwa_init;

		ROS_INFO_STREAM("x"<<xyz_iiwa_init.linear.x);
		ROS_INFO_STREAM("y"<<xyz_iiwa_init.linear.y);
		ROS_INFO_STREAM("z"<<xyz_iiwa_init.linear.z);
		ROS_INFO_STREAM("ax"<<roll_iiwa_init);
		ROS_INFO_STREAM("ay"<<pitch_iiwa_init);
		ROS_INFO_STREAM("az"<<yaw_iiwa_init);

	}
	iiwa_init_rot_in_world.setRPY(xyz_iiwa_init.angular.x,xyz_iiwa_init.angular.y,xyz_iiwa_init.angular.z);
	iiwa_init_pos_in_camera = transform(tf::Vector3(xyz_iiwa_init.linear.x,xyz_iiwa_init.linear.y,xyz_iiwa_init.linear.z));
	iiwa_init_rot_in_cam=transform*(iiwa_init_rot_in_world);
	tf::Matrix3x3(iiwa_init_rot_in_cam).getRPY(iiwa_init_roll_in_cam,iiwa_init_pitch_in_cam,iiwa_init_yaw_in_cam);

	while (ros::ok())
	{
		if (initialized)
		{
			if(initialized_phantom)
			{
				try{
					listener.lookupTransform("/US_probe_tip222", "/world", ros::Time(0), transform);
					inv_transform = transform.inverse();
				}
				catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					continue;
				}
				//Read Joint Positions of Phantom and put them in KDL format
				for (int k = 0; k<nj_phantom; ++k)
				{
					jointpositions_phantom(k) = joints_phantom.position[k];
				}
				//jointpositions_phantom(5)=3;

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
					phantom_rpy.publish(xyz_phantom);
					phantom_rot_in_world.setRPY(roll_phantom, pitch_phantom, yaw_phantom);
					phantom_rot_in_cam=transform*phantom_rot_in_world;
					tf::Matrix3x3(phantom_rot_in_cam).getRPY(phantom_roll_in_cam,phantom_pitch_in_cam,phantom_yaw_in_cam);
				}

				// Get LWR joint positions in KDL form
				for (int k = 0; k<7; ++k)
				{
					jointpositions(k) = joints.position[k];

				}
				//Perform Forward Kinematics to get Cartesian Pose of LWR

				//find where the robot is
				if(!start_loc_available) //if initial position is not known
				{
					kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
					if(kinematics_status>=0)
					{
						xyz.linear.x=iiwa_pos.pose.position.x;
						xyz.linear.y=iiwa_pos.pose.position.y;
						xyz.linear.z=iiwa_pos.pose.position.z;
						cartpos.M.GetRPY(roll,pitch, yaw);
						xyz.angular.x = roll;
						xyz.angular.y = pitch;
						xyz.angular.z = yaw;
					}
					start_loc_available = true;
				}
				else
				{
					ros::spinOnce();
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

					double z_gain = 1;
					ros::spinOnce();
					try{
						listener.lookupTransform("/US_probe_tip222", "/world", ros::Time(0), transform);
						inv_transform = transform.inverse();
					}
					catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}
					xyz_command.linear.y = -pos.pose.position.x*1.5;//xyz_phantom.linear.x;
					xyz_command.linear.z =(-pos.pose.position.y)*1.5; //xyz_phantom.linear.z*z_gain;
					xyz_command.linear.x = pos.pose.position.z*1.5;//xyz_phantom.linear.y;

					xyz_command.angular.x=xyz_phantom.angular.x-xyz_phantom_init.angular.x;
					xyz_command.angular.y=xyz_phantom.angular.y-xyz_phantom_init.angular.y;
					xyz_command.angular.z=xyz_phantom.angular.z-xyz_phantom_init.angular.z;

					iiwa_init_pos_in_camera = transform(tf::Vector3(xyz_iiwa_init.linear.x,xyz_iiwa_init.linear.y,xyz_iiwa_init.linear.z));
					ref.linear.x=iiwa_init_pos_in_camera.x()-xyz_command.linear.x;
					ref.linear.y=iiwa_init_pos_in_camera.y()-xyz_command.linear.y;
					ref.linear.z=iiwa_init_pos_in_camera.z()-xyz_command.linear.z;

					iiwa_pos_in_world = inv_transform(tf::Vector3(ref.linear.x,ref.linear.y,ref.linear.z));
					ref.linear.y = iiwa_pos_in_world.y();
					ref.linear.x = iiwa_pos_in_world.x();
					ref.linear.z = iiwa_pos_in_world.z();

					ros::spinOnce();
					try{
						listener2.lookupTransform("/US_probe_tip222", "/world", ros::Time(0), transform2);
						inv_transform2 = transform2.inverse();
					}
					catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}


					prev_x_iiwa_position=ref.linear.x;
					prev_y_iiwa_position=ref.linear.y;
					prev_z_iiwa_position=ref.linear.z;
					ROS_INFO_STREAM("NOT blocking");
					ROS_INFO_STREAM(ref.linear.z);

					blocking=0;
					block_msg.data=0;
					blocker.publish(block_msg);

					ref.angular.x = xyz.angular.x+cam_roll;
					ref.angular.y = xyz.angular.y+cam_pitch;//+cam_pitch;
					ref.angular.z = xyz.angular.z;//+cam_yaw;

					cartpos.p[0]=ref.linear.x;//xyz_iiwa_init.linear.x;//
					cartpos.p[1]=ref.linear.y;//xyz_iiwa_init.linear.y;//ref.linear.y;
					cartpos.p[2]=ref.linear.z;//xyz_iiwa_init.linear.z;//ref.linear.z;
					rpy = KDL::Rotation::RPY(ref.angular.x,ref.angular.y,ref.angular.z);

					cartpos.M =rpy;// cartpos_phantom.M;//rpy;
					int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
					jointpositions_new(6)=jointpositions_new(6)+cam_yaw;
					solver_debug.publish(ret_solver);
					eval_points(pt, jointpositions_new, nj);
					pt.time_from_start = ros::Duration(dt);
					joint_cmd.points[0] = pt;
					joint_cmd.header.stamp = ros::Time::now();
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
