#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "square");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  int can_no=1;
  ROS_INFO("Started square for %s",nh.getNamespace().c_str());

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


  float centre_x = std::stof(args.at(1));
  float centre_y = std::stof(args.at(2));
  float centre_z = std::stof(args.at(3));

  float x2,y2,z2;




  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),std::stof(args.at(3)));
  double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Publishing square on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  ROS_INFO("Coordinates of namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),centre_x,centre_y,centre_z);
  trajectory_pub.publish(trajectory_msg);



  x2=centre_x-2.5;
  y2=centre_y+1.5;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;


  x2=centre_x+2.5;
  y2=centre_y+1.5;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;


  x2=centre_x+2.5;
  y2=centre_y-1.5;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);
  can_no++;


  x2=centre_x-2.5;
  y2=centre_y-1.5;
  z2=1.5;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);




  x2=centre_x;
  y2=centre_y;
  z2=2;
  desired_position<<x2,y2,z2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ROS_INFO("Robot Drone %s going back to home at: [%f, %f, %f] after picking up the %d cans! ",nh.getNamespace().c_str(),x2,y2,z2,can_no);
  ros::Duration(3).sleep();
  trajectory_pub.publish(trajectory_msg);



  ros::spinOnce();
  ros::shutdown();

  return 0;
}
