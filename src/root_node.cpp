#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <ctime>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"


using namespace std;
int main(int argc, char** argv) {

  ros::init(argc, argv, "root_node");
  ros::NodeHandle nh;
  bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("waiting for set_model_state service");
      ros::Duration(1).sleep();
    }
  ROS_INFO("set_model_state service exists");
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  //ros::Publisher pub = nh.advertise<std_msgs::String>("can_xy",1);
  gazebo_msgs::SetModelState canstate;
  std::string can_name = "can_clone_";
  float x_rand,y_rand,x,y;
  float x_base , y_base;
  //float can_xy[16][2];
  string can_xy="";
  string comma=",";
  string colon = ";";
  string xcan = "/x/can";
  string ycan = "/y/can";
  x_base = -12.5;
  y_base = -4.5;
  int sign;
  srand((unsigned int)time(NULL));
  float a,b;
  a=0.6;
  b=0.3;
  for (int i=0; i<16; i++ )
  {
    if(i%4==0 && i!=0)
    {
      x_base=-7.5;
      y_base+=3.0;
    }
    else{x_base+=5;}
    can_name+= to_string(i);

    x_rand = (float(rand())/float((RAND_MAX))*a);
    y_rand = (float(rand())/float((RAND_MAX))*b);
    //cout<<x_rand<<endl<<i<<endl<<y_rand<<endl;
    sign = rand();
    if(sign%2 == 0){x_rand*=-1;}
    sign = rand();
    if(sign%2 == 0){y_rand*=-1;}

    x=x_base + x_rand;
    y=y_base + y_rand;

    canstate.request.model_state.model_name = can_name;
    canstate.request.model_state.pose.position.x = x;
    canstate.request.model_state.pose.position.y = y;
    canstate.request.model_state.pose.position.z = 1.012652;
    canstate.request.model_state.reference_frame = "world";

    client.call(canstate);
        //make sure service call was successful
    bool result = canstate.response.success;
    if (!result)
        ROS_WARN("LUL!");
    else
    ROS_INFO("Done");

    can_name = "can_clone_";
    // can_xy.append(to_string(x));
    // can_xy.append(comma);
    // can_xy.append(to_string(y));
    // can_xy.append(colon);
    // cout<<can_xy<<endl;
    //can_xy[i][0]=x;
    //can_xy[i][1]=y;
    //cout<<i<<") X: "<<x<<" Y: "<<y<<" can_x:"<<can_xy[i][0]<<" can_y:"<<can_xy[i][1]<<endl;

    xcan.append(to_string(i));
    ycan.append(to_string(i));
    ros::param::set(xcan,x);
    ros::param::set(ycan,y);
    xcan = "/x/can";
    ycan = "/y/can";
  }
  // std_msgs::String msg;
  // msg.data = can_xy;
  // if(ros::ok()==true)
  // {
  //   cout<<"ROS IS OK"<<endl;
  //   pub.publish(msg);
  // }
  ros::spinOnce();
  ros::shutdown();

  return 0;
}
