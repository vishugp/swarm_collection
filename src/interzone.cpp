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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "interzone");
  ros::NodeHandle nh;

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started interzone for %s",nh.getNamespace().c_str());

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    ROS_ERROR("Usage: interzone <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();


  float centre_x = std::stof(args.at(1));
  float centre_y = std::stof(args.at(2));
  float centre_z = std::stof(args.at(3));

  float x2,y2,z2;



























  // Taking ROS Param Can Coordinates and Saving in 2D array can_xy
  float xcanparam,ycanparam;
  string xcanname = "/x/can";
  string ycanname = "/y/can";
  // ros::param::get("/x/can2",xcanparam);
  // ros::param::get("/y/can2",ycanparam);
  // //cout<<xcanparam<<endl<<endl<<ycanparam<<endl<<endl;
  float can_xy[36][2];
  ros::Duration(delay/2).sleep();
  for(int i=0; i<36; i++)
  {
    xcanname.append(to_string(i));
    ycanname.append(to_string(i));
    ros::param::get(xcanname,xcanparam);
    ros::param::get(ycanname,ycanparam);
    can_xy[i][0]=xcanparam;
    can_xy[i][1]=ycanparam;
    //cout<<endl<<nh.getNamespace().c_str()<<" Can "<<i<<" : "<<xcanparam<<" , "<<ycanparam<<endl;
    xcanname = "/x/can";
    ycanname = "/y/can";

  }

  string drone_name = nh.getNamespace().c_str();
  int drone_len = drone_name.length();
  drone_len--;
  drone_name = drone_name[drone_len];
  int drone_no = std::stoi(drone_name);


///////////////////////////////////////////////////////////////////////////////

  // Segregating Can Coordinates of Drone according its Zone//
  float local[16][2];
  for(int i=0;i<16;i++)
  {
    local[i][0]=0.00;
    local[i][1]=0.00;
  }
  int j=0;

  //Method 1:-
  //1) Checking if belongs to 1&2 column (Drone 1/3) or 3&4column (Drone 2/4) by DroneNo%2 != or == 0
  //2) Checking if it belongs to Drone( 2/ 3) by DroneNo/2 ==1 or not (Drone 1/4 )


  // if(drone_no%2!=0)
  // {
  //   if(drone_no/2==1)
  //   {
  //     for(int i=0;i<16;i++)
  //     {
  //       if(can_xy[i][0]<0 && can_xy[i][1]<0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for(int i=0;i<16;i++)
  //     {
  //       if(can_xy[i][0]<0 && can_xy[i][1]>0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  // }
  // else
  // {
  //   if(drone_no/2==1)
  //   {
  //     for(int i=0;i<16;i++)
  //     {
  //       if(can_xy[i][0]>0 && can_xy[i][1]>0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for(int i=0;i<16;i++)
  //     {
  //       if(can_xy[i][0]>0 && can_xy[i][1]<0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  // }


  // Method 2:-
  // 1) Checking if its drone 3/4 by checking if DroneNo>2 or not (DroneNo 1/2)
  // 2) Checking if even numbered drone 2/4 by DroneNo%2==0 0r not (Drone 1/3)
  // if(drone_no>2)
  // {
  //   if(drone_no%2==0)
  //   {
  //     for(int i=0;i<8;i++)
  //     {
  //       if(can_xy[i][0]>0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for(int i=0;i<8;i++)
  //     {
  //       if(can_xy[i][0]<0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  // }
  // else
  // {
  //   if(drone_no%2==0)
  //   {
  //     for(int i=8;i<16;i++)
  //     {
  //       if(can_xy[i][0]>0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for(int i=8;i<16;i++)
  //     {
  //       if(can_xy[i][0]<0)
  //       {
  //         local[j][0]=can_xy[i][0];
  //         local[j][1]=can_xy[i][1];
  //         j++;
  //       }
  //     }
  //   }
  // }

  //Method 3:-
  //Direct XD
  if(drone_no==1)
  {
    for(int i = 0;i<36;i++)
    {
      if(can_xy[i][0]>0 && can_xy[i][1]>0)
      {
        local[j][0]=can_xy[i][0];
        local[j][1]=can_xy[i][1];
        j++;
      }
    }
  }
  else if(drone_no==2)
  {
    for(int i = 0;i<36;i++)
    {
      if(can_xy[i][0]<0 && can_xy[i][1]>0)
      {
        local[j][0]=can_xy[i][0];
        local[j][1]=can_xy[i][1];
        j++;
      }
    }
  }
  else if(drone_no==3)
  {
    for(int i = 0;i<36;i++)
    {
      if(can_xy[i][0]<0 && can_xy[i][1]<0)
      {
        local[j][0]=can_xy[i][0];
        local[j][1]=can_xy[i][1];
        j++;
      }
    }
  }
  else if(drone_no==4)
  {
    for(int i = 0;i<36;i++)
    {
      if(can_xy[i][0]>0 && can_xy[i][1]<0)
      {
        local[j][0]=can_xy[i][0];
        local[j][1]=can_xy[i][1];
        j++;
      }
    }
  }

  for(int i=0;i<16;i++)
  {
    //cout<<"Drone "<<nh.getNamespace().c_str()<<" Can: "<<i<<" X: "<<local[i][0]<<" Y: "<<local[i][1]<<endl;
  }



  ////cout<<endl<<drone_name<<"'s' no. is : "<<drone_no<<" and length of name: "<<drone_len<<endl;
////////////////////////////////////////////////////////////////////////////////

  //Drone hovers at starting point

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
  ROS_INFO("Publishing interzone on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
  //ROS_INFO("Coordinates of namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),centre_x,centre_y,centre_z);
  trajectory_pub.publish(trajectory_msg);


  //List of order of collecting Cans of Drones Zone
  // int list[]={0,3,2,1};
  float x_z = 0.0;
  float y_z = 0.0;
  int list[j];
  for(int i=0; i<j; i++)
  {
    list[i]=i;
  }

  int index;
  int can_no=1;
  for(int i=0;i<j;i++)
  {

    for(int k=0; k<32;k++)
    {
      if(local[index][0]==can_xy[k][0] && local[index][1]==can_xy[k][1])
      {
        xcanname.append(to_string(k));
        ycanname.append(to_string(k));
        ros::param::set(xcanname,x_z);
        ros::param::set(ycanname,y_z);
        xcanname = "/x/can";
        ycanname = "/y/can";
      }


    }

    //Drone goes to Can for picking up
    index = list[i];
    x2=local[index][0];
    y2=local[index][1];
    z2=1.5;

    desired_position<<x2,y2,z2;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);
    ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
    ros::Duration(3).sleep();
    trajectory_pub.publish(trajectory_msg);

    //Drone goes to bin at centre starting of drone
    x2=centre_x;
    y2=centre_y;
    desired_position<<x2,y2,z2;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);
    //ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
    ros::Duration(3).sleep();
    trajectory_pub.publish(trajectory_msg);

    can_no++;
  }
  //cout<<"Drone "<<drone_name<<" done with its zone!"<<endl<<"The Can Params now are :"<<endl<<endl;



  // cout<<"LEL";
  // for(int i=0; i<32;i++)
  // {
  //   xcanname.append(to_string(i));
  //   ycanname.append(to_string(i));
  //   ros::param::get(xcanname,xcanparam);
  //   ros::param::get(ycanname,ycanparam);
  //
  //   if(xcanparam!=0.0)
  //   {
  //     x2=xcanparam;
  //     y2=ycanparam;
  //     z2=1.5;
  //
  //     desired_position<<x2,y2,z2;
  //     mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
  //         desired_yaw, &trajectory_msg);
  //     ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),can_no,x2,y2,z2);
  //     ros::Duration(3).sleep();
  //     trajectory_pub.publish(trajectory_msg);
  //
  //     //Drone goes to bin at centre starting of drone
  //     x2=centre_x;
  //     y2=centre_y;
  //     desired_position<<x2,y2,z2;
  //     mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
  //         desired_yaw, &trajectory_msg);
  //     //ROS_INFO("Robot Drone %s going to Can No. %d at: [%f, %f, %f].",nh.getNamespace().c_str(),x2,y2,z2);
  //     ros::Duration(3).sleep();
  //     trajectory_pub.publish(trajectory_msg);
  //   }
  //   // can_xy[i][0]=xcanparam;
  //   // can_xy[i][1]=ycanparam;
  //   //cout<<endl<<nh.getNamespace().c_str()<<" Can "<<i<<" : "<<xcanparam<<" , "<<ycanparam<<endl;
  //   xcanname = "/x/can";
  //   ycanname = "/y/can";
  // }

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
