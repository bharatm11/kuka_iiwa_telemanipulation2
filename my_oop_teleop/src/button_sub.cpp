#include "ros/ros.h"
#include "std_msgs/String.h"
#include<omni_msgs/OmniFeedback.h>
#include<omni_msgs/OmniButtonEvent.h>
omni_msgs::OmniButtonEvent button;
omni_msgs::OmniButtonEvent prev_button;
void get_button(const omni_msgs::OmniButtonEvent & _data){
  button = _data;
  // check if the control mode has changed via the grey button
  if(button.grey_button == 1 && prev_button.grey_button == 0){
    ROS_INFO_STREAM("Grey on");
  } else if(button.grey_button == 0 && prev_button.grey_button == 1) {
    ROS_INFO_STREAM("Grey off");
  }
  

  int main(int argc, char **argv)
  {

    ros::init(argc, argv, "listener");


    ros::NodeHandle n;


    ros::Subscriber button_sub = n.subscribe("/phantom/button",10, get_button);


    ros::spin();

    return 0;
  }
