#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def arm_angler():
    pi=3.142
    pub1 = rospy.Publisher('/firefly1/firefly/arm1_joint_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/firefly1/firefly/arm2_joint_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/firefly1/firefly/arm3_joint_position_controller/command', Float64, queue_size=10)

    rospy.init_node('gripper_control', anonymous=False)
    rate = rospy.Rate(1000)
    t1 = 0.00
    while not rospy.is_shutdown():
        t2 = float(input("Enter Angle: "))
        t2 = (t2*pi)/180
        diff = t2-t1
        print("Initial Angle: ",t1)
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
       arm_angler()
   except rospy.ROSInterruptException:
       pass
