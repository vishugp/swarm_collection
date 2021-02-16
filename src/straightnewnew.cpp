#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "straightnew");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  int can_no=1;
  ROS_INFO("Started straightnew for %s",nh.getNamespace().c_str());

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: straightnew <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();


  float start_x = std::stof(args.at(1));
  float start_y = std::stof(args.at(2));
  float start_z = std::stof(args.at(3));

  float x2,y2,z2;

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

  // goin to collect  takeoff to r1
  x2=start_x;
  y2=(start_y)+2;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;

// r1 to b1
  x2=start_x;
  y2=(start_y)+1;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);

// b1 to r2
  x2=start_x;
  y2=start_y+5;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(4).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;

// r2 to b2
  x2=start_x;
  y2=(start_y)+1;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(4).sleep();
  trajectory_pub.publish(trajectory_msg);

//b2 to r3
  x2=start_x;
  y2=start_y+8;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(6.5).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;

//r3 to b3
  x2=start_x;
  y2=(start_y)+1;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(6).sleep();
  trajectory_pub.publish(trajectory_msg);

  //b3 to some comfortable height
  x2=start_x;
  y2=(start_y)+1;
  z2=4;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);

  //b3 to r4
  x2=start_x;
  y2=start_y+11;
  z2=4;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(9).sleep();
  trajectory_pub.publish(trajectory_msg);

  //r4 top to low
  x2=start_x;
  y2=(start_y)+11;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(2).sleep();
  trajectory_pub.publish(trajectory_msg);


  //r4 to comfy height
  x2=start_x;
  y2=start_y+11;
  z2=4;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to Row No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(9).sleep();
  trajectory_pub.publish(trajectory_msg);



  //r4 to b4
  x2=start_x;
  y2=(start_y)+1;
  z2=4;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(9).sleep();
  trajectory_pub.publish(trajectory_msg);

  x2=start_x;
  y2=(start_y)+1;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drones %s going to bin at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  ros::Duration(2).sleep();
  trajectory_pub.publish(trajectory_msg);




  ros::spinOnce();
  ros::shutdown();

  return 0;
}

