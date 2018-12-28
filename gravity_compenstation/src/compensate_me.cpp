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

//Get Pose of IIWA
geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped & _data)
{
	pos = _data;

}
geometry_msgs::WrenchStamped ft;
bool are_we_fucked=0;
int start_fucking=0;
void get_forces(const geometry_msgs::WrenchStamped & _fdata)
{
	ft = _fdata;
	are_we_fucked=1;


}
double weight=5.8;
//tf::Vector3 f_o={{0},{0.0},{weight}};
tf::Vector3 in_world;
double e_ft_rot[3][3]={{1,0,0},{0,1,0},{0,0,1}};
double q0,q1,q2,q3;
double rob_q0,rob_q1,rob_q2,rob_q3;
tf::Quaternion rob_quat;
double r_e_rot[3][3];
double determinant=0;

double curr_force_z=0;
double prev_force_z=0;
double curr_force_x=0;
double prev_force_x=0;
double curr_force_y=0;
double prev_force_y=0;
double dt_damp=0;
double error_z=0;
double error_y=0;
double error_x=0;

geometry_msgs::WrenchStamped compensated_force;
omni_msgs::OmniFeedback centering_force;
//////////   MAIN  //////////////
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "haptic_feedback");
	ros::NodeHandle nh_compensation;
	int loop_freq = 4;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	ros::Publisher force_pub =nh_compensation.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback",10);
	ros::Publisher compensated_forces_pub = nh_compensation.advertise<geometry_msgs::WrenchStamped>("/compensated_forces_ft_frame",10);
	ros::Subscriber forces_sub = nh_compensation.subscribe("/ft18215",10, get_forces);
	ros::Subscriber pos_sub = nh_compensation.subscribe("/iiwa/state/CartesianPose",10, get_pos);
	ros::Publisher pub5 = nh_compensation.advertise<trajectory_msgs::JointTrajectory> ("/iiwa/PositionJointInterface_trajectory_controller/command", 1000);// ("/lwr/joint_trajectory_controller/command", 1000);

	/*	// Move Robot To Start Position
	trajectory_msgs::JointTrajectory msg5;
	msg5.header.seq = 0;
	msg5.header.stamp.sec = 0;
	msg5.header.stamp.nsec = 0;
	msg5.header.frame_id = "";
	msg5.joint_names.push_back("iiwa_joint_1");//("lwr_a1_joint");
	msg5.joint_names.push_back("iiwa_joint_2");//("lwr_a2_joint");
	msg5.joint_names.push_back("iiwa_joint_3");//("lwr_a3_joint");
	msg5.joint_names.push_back("iiwa_joint_4");//("lwr_a4_joint");
	msg5.joint_names.push_back("iiwa_joint_5");//("lwr_a5_joint");
	msg5.joint_names.push_back("iiwa_joint_6");//("lwr_a6_joint");
	msg5.joint_names.push_back("iiwa_joint_7");//("lwr_e1_joint");
	msg5.points.resize(1);
	int ind = 0;
	msg5.points[ind].positions.resize(7);
	msg5.points[ind].positions[0] = 0;
	msg5.points[ind].positions[1] = 0;
	msg5.points[ind].positions[2] = 0;
	msg5.points[ind].positions[3] = -1.57;
	msg5.points[ind].positions[4] = 0;
	msg5.points[ind].positions[5] = 1.57;
	msg5.points[ind].positions[6] = 0.7784579992294312;//100;
	msg5.points[ind].velocities.resize(7);
	msg5.points[ind].effort.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
	msg5.points[ind].velocities[j]=0.0;
	msg5.points[ind].effort[j] = 0.0;
}
msg5.points[ind].time_from_start = ros::Duration(1.0);
pub5.publish(msg5);*/
ros::spinOnce();
loop_rate.sleep();

tf::TransformListener listener;
tf::StampedTransform transform;
tf::TransformListener listener2;
tf::StampedTransform transform2;
tf::Matrix3x3 rot_mat;
double curr_secs=ros::Time::now().toSec();
double prev_secs=ros::Time::now().toSec();
while(ros::ok())
{
	are_we_fucked=0;
	try
	{
		listener.lookupTransform("/ft_sensor","/world",ros::Time(0), transform);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
	}
	ros::spinOnce();
	q0=transform.getRotation().x();
	q1=transform.getRotation().y();
	q2=transform.getRotation().z();
	q3=transform.getRotation().w();

	in_world=transform(tf::Vector3(0,0,weight));

	compensated_force.wrench.force.x=(ft.wrench.force.x+in_world.x());
	compensated_force.wrench.force.y=(ft.wrench.force.y+in_world.y());
	compensated_force.wrench.force.z=(ft.wrench.force.z+in_world.z());

	centering_force.force.x=0.4*compensated_force.wrench.force.x;
	centering_force.force.y=-0.3*compensated_force.wrench.force.z;
	centering_force.force.z=0.4*compensated_force.wrench.force.y;


	curr_force_z=centering_force.force.z;
	curr_force_x=centering_force.force.x;
	curr_force_y=centering_force.force.y;
	if(curr_secs-prev_secs>0.1)
	{
		error_z=(curr_force_z-prev_force_z);
		error_x=(curr_force_x-prev_force_x);
		error_y=(curr_force_y-prev_force_y);
		dt_damp=curr_secs-prev_secs;
		centering_force.force.z=0.4*error_z+25*error_z/dt_damp;
		centering_force.force.x=0.4*error_x+25*error_x/dt_damp;
		centering_force.force.y=0.4*error_y+25*error_y/dt_damp;
		prev_secs=curr_secs;
		prev_force_z=curr_force_z;
		prev_force_x=curr_force_x;
		prev_force_y=curr_force_y;


	}
	if(centering_force.force.y>2.0)
	{
		centering_force.force.y=2;
	}





	//centering_force.force.z=0;
	ROS_INFO_STREAM("Forces:"<<"\n");
	ROS_INFO_STREAM("x:"<<in_world.x()<<"\n");
	ROS_INFO_STREAM("y:"<<in_world.y()<<"\n");
	ROS_INFO_STREAM("z:"<<in_world.z()<<"\n");
	centering_force.force.x=0;
	//centering_force.force.y=0;
	centering_force.force.z=0;
	force_pub.publish(centering_force);

	if(start_fucking<5)
	{
		start_fucking++;
	}
	else
	{
		if(are_we_fucked==0)
		{
			compensated_force.wrench.force.x=5;
			compensated_force.wrench.force.y=5;
			compensated_force.wrench.force.z=5;
		}
	}




	compensated_forces_pub.publish(compensated_force);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;

}
