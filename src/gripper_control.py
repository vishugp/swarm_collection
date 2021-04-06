#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pi=3.142
    pub1 = rospy.Publisher('/firefly/firefly/arm1_joint_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/firefly/firefly/arm2_joint_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/firefly/firefly/arm3_joint_position_controller/command', Float64, queue_size=10)

    rospy.init_node('gripper_control', anonymous=False)
    rate = rospy.Rate(1000)
    rospy.set_param('angle',0)
    while not rospy.is_shutdown():
       instr = rospy.get_param('angle')

       if instr==0:
           ## Goes from 0 to pi/8
           theta = 0.00
           print("Initial Angle: ",theta)
           for i in range(0,100):
               theta = theta+pi/800
               print("Iteration: ",i," Angle= ",theta*180/3.142)
               pub1.publish(theta)
               pub2.publish(theta)
               pub3.publish(theta)
               rospy.sleep(0.01)
        			   #100 iter.. totally takes 1 sec to go from 0 to pi/8
           rospy.set_param('angle',1)
           rate.sleep()
       elif instr==1:
           ## Goes from pi/8 to 0.
           theta = pi/8;
           print("Initial Angle: ",theta)
           for i in range(0,100):
               theta = theta-pi/800
               print("Iteration: ",i," Angle= ",theta*180/3.142)
               pub1.publish(theta)
               pub2.publish(theta)
               pub3.publish(theta)
               rospy.sleep(0.01)			#100 iter.. totally takes 1 sec to go from pi/8 to 0.
           rospy.set_param('angle',0)
           rate.sleep()


if __name__ == '__main__':
   try:
       talker()
   except rospy.ROSInterruptException:
       pass
