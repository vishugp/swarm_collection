#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_hover");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 2.0);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);
  ros::Duration(3.0).sleep();

//drone1
  ros::Publisher gripper_pub11 = nh.advertise<std_msgs::Float64>("/firefly1/firefly/arm1_joint_position_controller/command", 10);
  ros::Publisher gripper_pub12 = nh.advertise<std_msgs::Float64>("/firefly1/firefly/arm2_joint_position_controller/command", 10);
  ros::Publisher gripper_pub13 = nh.advertise<std_msgs::Float64>("/firefly1/firefly/arm3_joint_position_controller/command", 10);
//drone2
  ros::Publisher gripper_pub21 = nh.advertise<std_msgs::Float64>("/firefly2/firefly/arm3_joint_position_controller/command", 10);
  ros::Publisher gripper_pub22 = nh.advertise<std_msgs::Float64>("/firefly2/firefly/arm3_joint_position_controller/command", 10);
  ros::Publisher gripper_pub23 = nh.advertise<std_msgs::Float64>("/firefly2/firefly/arm3_joint_position_controller/command", 10);

  ros::Rate loop_rate(2);


  while (ros::ok())
  {

      std_msgs::Float64 angle;
      angle.data = 0.261799;        // 15 degrees

      gripper_pub11.publish(angle);
      gripper_pub12.publish(angle);
      gripper_pub13.publish(angle);

      gripper_pub21.publish(angle);
      gripper_pub22.publish(angle);
      gripper_pub23.publish(angle);

      ros::spinOnce();
      loop_rate.sleep();

  }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
