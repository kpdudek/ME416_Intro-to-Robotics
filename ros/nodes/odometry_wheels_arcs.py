#!/usr/bin/env python
"""Node that publishes estimated pose of ROSBot using the closed form solution"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from me416_lab.msg import MotorSpeedsStamped
import motor_command_model as mcm
import me416_utilities as mu
import numpy as np

class ROSBot_Odometry(object):
    def __init__(self):
        self.mtr_spd_sub = rospy.Subscriber('/motor_speeds',MotorSpeedsStamped,callback=self.mtr_spd_callback,queue_size=10)
        self.odom_pub = rospy.Publisher('/pose_arcs',Pose2D,queue_size=10)
        self.odom_data = Pose2D
        self.odom_data.x = 0
        self.odom_data.y = 0
        self.odom_data.z = 0
        
        self.mtr_spd_stamp = MotorSpeedsStamped
        self.msg_reg = mcm.StampedMsgRegister()

    def mtr_spd_callback(self,msg):
        self.mtr_spd_stamp.left = msg.left
        self.mtr_spd_stamp.right = msg.right
        self.mtr_spd_stamp.header = msg.header

    def update_pose(self):
        z = np.array([[self.odom_data.x],[self.odom_data.y],[self.odom_data.z]])
        u = np.array([[self.mtr_spd_stamp.left],[self.mtr_spd_stamp.right]])
        T = self.msg_reg.replace_and_compute_delay(self.mtr_spd_stamp)
        update_pose = mcm.closed_form_step(z,u,T)

        self.odom_data.x = update_pose[0]
        self.odom_data.y = update_pose[1]
        self.odom_data.z = update_pose[2]
    
    def send(self):
        self.odom_pub.publish(self.odom_data)

def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('Odometry', anonymous='True')
    
    odom = ROSBot_Odometry()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        odom.send()
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
