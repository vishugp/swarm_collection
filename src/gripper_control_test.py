#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def rosparamsetter():
    ns = rospy.get_namespace()
    rospy.init_node('gripper_control_test', anonymous=False)
    rate = rospy.Rate(1000)

    gripper_mode_test = ns + 'firefly1/gripper_mode'
    while not rospy.is_shutdown():
        print(gripper_mode_test)
        opt = int(input("\nEnter 0 for open OR 1 for close : "))
        if opt==0:
            rospy.set_param(gripper_mode_test,0)
            print("Opening Gripper")
        else:
            rospy.set_param(gripper_mode_test,1)
            print("Closing Gripper")

if __name__ == '__main__':
   try:
       rosparamsetter()
   except rospy.ROSInterruptException:
       pass
