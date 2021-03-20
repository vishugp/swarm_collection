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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "interzone_rootnode");
  ros::NodeHandle nh;
  bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("Waiting for set_model_state service");
      ros::Duration(1).sleep();
    }
  ROS_INFO("set_model_state service exists");
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState canstate;

  int rem=36;
  int table_rand;
  int can_no=0;
  int total_can_no = 0;

  std::string can_name = "can_clone_";
  // std::string extra_can_name = "extra_can_clone_";

  float x_rand,y_rand,x,y;
  float x_base , y_base;

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



    // x_rand = (float(rand())/float((RAND_MAX))*a);
    // y_rand = (float(rand())/float((RAND_MAX))*b);
    // sign = rand();
    // if(sign%2 == 0){x_rand*=-1;}
    // sign = rand();
    // if(sign%2 == 0){y_rand*=-1;}
    //
    // x=x_base + x_rand;
    // y=y_base + y_rand;
    // can_name+= to_string(i);
    //
    // canstate.request.model_state.model_name = can_name;
    // canstate.request.model_state.pose.position.x = x;
    // canstate.request.model_state.pose.position.y = y;
    // canstate.request.model_state.pose.position.z = 1.012652;
    // canstate.request.model_state.reference_frame = "world";
    //
    // client.call(canstate);
    //     //make sure service call was successful
    // bool result = canstate.response.success;
    // if (!result)
    //     ROS_WARN("LUL!");
    // else
    // ROS_INFO("Done");
    //
    //
    //
    // xcan.append(to_string(total_can_no));
    // ycan.append(to_string(total_can_no));
    // ros::param::set(xcan,x);
    // ros::param::set(ycan,y);
    //
    // xcan = "/x/can";
    // ycan = "/y/can";
    // can_name = "can_clone_";
    // total_can_no++;
    //
    if(rem>0)
    {
      if(rem<4){table_rand = rand() % rem;}
      else {table_rand = rand() % 5;}
      ////cout<<endl<<table_rand<<endl;
      //table_rand=1;


      for(int j=0;j<table_rand;j++)
      {



        x_rand = (float(rand())/float((RAND_MAX))*a);
        y_rand = (float(rand())/float((RAND_MAX))*b);
        sign = rand();
        if(sign%2 == 0){x_rand*=-1;}
        sign = rand();
        if(sign%2 == 0){y_rand*=-1;}

        x=x_base + x_rand;
        y=y_base + y_rand;
        can_name+= to_string(can_no);

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

        xcan.append(to_string(total_can_no));
        ycan.append(to_string(total_can_no));
        ros::param::set(xcan,x);
        ros::param::set(ycan,y);

        xcan = "/x/can";
        ycan = "/y/can";
        rem--;
        can_no++;
        total_can_no++;

      }
    }
  }
  if(rem!=0)
  {
    while(rem--)
    {
      x=0.00;
      y=0.00;
      xcan.append(to_string(total_can_no));
      ycan.append(to_string(total_can_no));
      ros::param::set(xcan,x);
      ros::param::set(ycan,y);
      //cout<<endl<<endl<<total_can_no<<endl;
      xcan = "/x/can";
      ycan = "/y/can";
      total_can_no++;
    }
  }
  //cout<<" Total cans: "<<total_can_no<<endl;



  ros::spinOnce();
  ros::shutdown();

  return 0;
}
