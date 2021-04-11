#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def rosparamsetter():
    print("\n*** DEFAULT GRIPPER OPEN/CLOSE MODE ***")
    ns = rospy.get_namespace()
    rate = rospy.Rate(1000)

    gripper_mode_test = ns + 'firefly1/gripper_mode'
    while not rospy.is_shutdown():
        print(gripper_mode_test)
        opt = int(input("\nEnter 0 for open OR 1 for close : "))
        if opt==0:
            t2 = 0.00
        if opt==1:
            t2 = 8.00

        t2 = (t2*pi)/180

        if t1>t2:
            print("Gripper of",ns,"is Opening")
            print("From",t1,"to",t2)
        if t1<t2:
            print("Gripper of ",ns," is Closing")
            print("From",t1,"to",t2)


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


def arm_angler():
    print("\n*** USER INPUT GRIPPER ANGLE MODE ***")
    ns = rospy.get_namespace()
    ns = '/firefly1/'
    pi=3.142
    arm1 = '/arm1_joint_position_controller/command'
    arm2 = '/arm2_joint_position_controller/command'
    arm3 = '/arm3_joint_position_controller/command'

    print(ns)
    pubname1 = ns + ns[1:-2] + arm1
    pubname2 = ns + ns[1:-2] + arm2
    pubname3 = ns + ns[1:-2] + arm3
    print(pubname2)
    pub1 = rospy.Publisher(pubname1, Float64, queue_size=10)
    pub2 = rospy.Publisher(pubname2, Float64, queue_size=10)
    pub3 = rospy.Publisher(pubname3, Float64, queue_size=10)


    gripper_mode = ns + 'gripper_mode'
    print("Gripper Mode= ", gripper_mode)
    # pub2 = rospy.Publisher('/firefly1/firefly/arm2_joint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(1000)
    t1 = 0.00
    while not rospy.is_shutdown():
        t2 = float(input("Enter Angle in Degrees: "))
        t2 = (t2*pi)/180
        diff = t2-t1
        print("Initial Angle: ",t1)
        if t1>t2:
            print("Gripper of",ns,"is Opening")
            print("From",t1,"to",t2)
        if t1<t2:
            print("Gripper of ",ns," is Closing")
            print("From",t1,"to",t2)
        for i in range(0,100):
            t1+=(diff/100)
            print("Iteration: ",i," Angle= ",t1*180/3.142)
            pub1.publish(t1)
            pub2.publish(t1)
            pub3.publish(t1)
            rospy.sleep(0.01)
        t1=t2



if __name__ == '__main__':
   try:
       rospy.init_node('gripper_control_test', anonymous=False)
       option = int(input("For Default Open Close Mode\tPress 0: \nFor User Input Angle Mode\tPress 1: \n\n??? := "))
       if option==0:
           rosparamsetter()
       if option==1:
           arm_angler()
   except rospy.ROSInterruptException:
       pass
