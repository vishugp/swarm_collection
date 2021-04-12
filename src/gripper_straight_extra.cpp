#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "extra_straight");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);


  ROS_INFO("Started extra_straight for %s",nh.getNamespace().c_str());

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: extra_straight <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();


  float start_x = std::stof(args.at(1));
  float start_y = std::stof(args.at(2));
  float start_z = std::stof(args.at(3));

  float x2,y2,z2;


  float xcanparam,ycanparam;
  string xcanname = "/x/can";
  string ycanname = "/y/can";

  //assining random can coordinates in a matrix
  float can_xy[32][2];
  ros::Duration(delay/2).sleep();
  for(int i=0;i<32;i++)
  { xcanname.append(to_string(i));
    ycanname.append(to_string(i));
    ros::param::get(xcanname,xcanparam);
    ros::param::get(ycanname,ycanparam);
    can_xy[i][0]=xcanparam;
    can_xy[i][1]=ycanparam;
    xcanname = "/x/can";
    ycanname = "/y/can";
  }
  //identifying drone with drone_no
  string drone_name=nh.getNamespace().c_str();
  int drone_len = drone_name.length();
  drone_len--;
  drone_name = drone_name[drone_len];
  int drone_no = std::stoi(drone_name);

  float local[12][2];
//initializing all to 0
  for(int i=0;i<12;i++)
  {
    local[i][0]=0.00;
    local[i][1]=0.00;
  }

  //putting can_xy coordinates into local columnwise coordinates matrix

  int j=0;
  int t=0;
  while(j<32)
  {
    if(can_xy[j][0] > (start_x+0.6) || can_xy[j][0] < (start_x-0.6))

    {  j++;
    }

    else
    {
        local[t][0]=can_xy[j][0];
        local[t][1]=can_xy[j][1];
        t++;
        j++;
    }
  }



  //Takeoff drones
  Eigen::Vector3d desired_position(start_x,start_y,start_z);
  double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }


  ROS_INFO("Publishing extra_straight on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  //ROS_INFO("Bots taking off to %s: [%f, %f, %f].",nh.getNamespace().c_str(),start_x,start_y,start_z);
  trajectory_pub.publish(trajectory_msg);

  // GRIPPER ROS PARAM NAME //
  string name = nh.getNamespace().c_str();
  string gripper_mode = "/gripper_mode";
  name.append(gripper_mode);



  for(int i=0;i<t;i++)
  {  // goin to collect

      x2=local[i][0];
      y2=local[i][1];
      z2=2;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
          desired_yaw, &trajectory_msg);
    //  ROS_INFO("Robot Drone %s going to : [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
      ros::Duration(3).sleep();
      trajectory_pub.publish(trajectory_msg);

      //lowering to can height
      x2=local[i][0];
      y2=local[i][1];
      z2=1.45;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
          desired_yaw, &trajectory_msg);
      ros::Duration(3).sleep();
      trajectory_pub.publish(trajectory_msg);

      //picking the can using param
      ros::Duration(0.4).sleep();
      ros::param::set(name,1);

      //going to z=2
      x2=local[i][0];
      y2=local[i][1];
      z2=2;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
          desired_yaw, &trajectory_msg);
      //ROS_INFO("Robot Drone %s going to : [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
      ros::Duration(3).sleep();
      trajectory_pub.publish(trajectory_msg);


      //going to bin
      x2=start_x;
      y2=0;
      z2=2;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);
      ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
      ros::Duration(3).sleep();
      trajectory_pub.publish(trajectory_msg);


      // Can Drop
      ros::Duration(1.5).sleep();
      ros::param::set(name,0);

  }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
