#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def arm_angler():
    ns = rospy.get_namespace()
    pi=3.142
    arm1 = 'firefly/arm1_joint_position_controller/command'
    arm2 = 'firefly/arm2_joint_position_controller/command'
    arm3 = 'firefly/arm3_joint_position_controller/command'

    print(ns)
    pubname1 = ns + arm1
    pubname2 = ns + arm2
    pubname3 = ns + arm3
    print(pubname2)
    pub1 = rospy.Publisher(pubname1, Float64, queue_size=10)
    pub2 = rospy.Publisher(pubname2, Float64, queue_size=10)
    pub3 = rospy.Publisher(pubname3, Float64, queue_size=10)


    gripper_mode = ns + 'gripper_mode'
    print("Gripper Mode= ", gripper_mode)
    rospy.set_param(gripper_mode,0)

    # pub2 = rospy.Publisher('/firefly1/firefly/arm2_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('gripper_control', anonymous=False)
    rate = rospy.Rate(1000)
    t1 = 0.00
    t2 = 0.00


    while not rospy.is_shutdown():
        opt = rospy.get_param(gripper_mode)
        if opt==0:
            t2 = 0.00
        if opt==1:
            t2 = 7.77

        if t1>t2:
            print("Gripper is Opening")
        if t1<t2:
            print(t1,t2)
            print("Gripper is Closing")
        t2 = (t2*pi)/180
        diff = t2-t1
        #print("Initial Angle: ",t1)
        for i in range(0,100):
            t1+=(diff/100)
            #print("Iteration: ",i," Angle= ",t1*180/3.142)
            pub1.publish(t1)
            pub2.publish(t1)
            pub3.publish(t1)
            rospy.sleep(0.01)
        # print("Angle of arms wrt axes: ",t2)
        t1=t2


if __name__ == '__main__':
   try:
       arm_angler()
   except rospy.ROSInterruptException:
       pass
