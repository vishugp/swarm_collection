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
  ros::init(argc, argv, "straight");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);


  ROS_INFO("Started straight for %s",nh.getNamespace().c_str());

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: square <x> <y> <z> <yaw_deg> [<delay>]\n");
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
  float can_xy[16][2];
  ros::Duration(delay/2).sleep();
  for(int i=0;i<16;i++)
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


  int i=drone_no-1;
  float local[4][2];

  //putting can_xy coordinates into local columnwise coordinates matrix
  for(int j=0;j<4;j++)
  {
    local[j][0]=can_xy[i+(j*4)][0];
    local[j][1]=can_xy[i+(j*4)][1];
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


  ROS_INFO("Publishing straight on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  ROS_INFO("Bots taking off to %s: [%f, %f, %f].",nh.getNamespace().c_str(),start_x,start_y,start_z);
  trajectory_pub.publish(trajectory_msg);

  int row_no=1;
  for(int i=0;i<4;i++)
  {  // goin to collect
      int t;
      int flag=0;
      if(i==0)
      {t=3;}
      if(i==1)
      {t=4;}
      if(i==2)
      {t=6;}
      if(i==3)
      {
        //b3 to some comfortable height
        x2=start_x;
        y2=(start_y)+1;
        z2=4;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s going to comfortable height at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
        ros::Duration(3).sleep();
        trajectory_pub.publish(trajectory_msg);

        //b3 to r4
        x2=local[i][0];
        y2=local[i][1];
        z2=4;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),row_no,x2,y2,z2);
        ros::Duration(4).sleep();
        trajectory_pub.publish(trajectory_msg);

        //r4 top to low
        x2=local[i][0];
        y2=local[i][1];
        z2=1.5;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s lowering to: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
        ros::Duration(2).sleep();
        trajectory_pub.publish(trajectory_msg);

        //r4 to comfy height
        x2=local[i][0];
        y2=local[i][1];
        z2=4;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s going to comfy height at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
        ros::Duration(3).sleep();
        trajectory_pub.publish(trajectory_msg);

        //r4 to b4
        x2=start_x;
        y2=(start_y)+1;
        z2=4;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
        ros::Duration(5).sleep();
        trajectory_pub.publish(trajectory_msg);

        //lowering height
        x2=start_x;
        y2=(start_y)+1;
        z2=1.5;
        desired_position<<x2,y2,z2;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);
        ROS_INFO("Robot Drones %s lowering to: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
        ros::Duration(2).sleep();
        trajectory_pub.publish(trajectory_msg);
        flag=1;
      }


      if(flag==1)
      {break;}

      x2=local[i][0];
      y2=local[i][1];
      z2=1.5;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
          desired_yaw, &trajectory_msg);
      ROS_INFO("Robot Drone %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),row_no,x2,y2,z2);
      ros::Duration(t).sleep();
      trajectory_pub.publish(trajectory_msg);

    //going to bin
      x2=start_x;
      y2=(start_y)+1;
      z2=1.5;
      desired_position<<x2,y2,z2;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);
      ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
      ros::Duration(t).sleep();
      trajectory_pub.publish(trajectory_msg);
      row_no++;
  }


  ros::spinOnce();
  ros::shutdown();

  return 0;
}
