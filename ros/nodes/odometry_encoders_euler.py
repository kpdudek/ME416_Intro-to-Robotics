#!/usr/bin/env python
"""Node that publishes estimated pose of ROSBot using the encoder readings and euler integration"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from me416_lab.msg import MotorSpeedsStamped
import motor_command_model as mcm
import me416_utilities as mu
import numpy as np

class ROSBot_Odometry_Euler(object):
    def __init__(self):
        self.mtr_spd_sub = rospy.Subscriber('/motor_speeds_encoders',MotorSpeedsStamped,callback=self.mtr_spd_callback,queue_size=10)
        self.odom_pub = rospy.Publisher('/pose_euler',Pose2D,queue_size=10)
        self.odom_data = Pose2D()
        self.odom_data.x = 0.0
        self.odom_data.y = 0.0
        self.odom_data.theta = 0.0

        self.mtr_spd_stamp_prev = MotorSpeedsStamped()
        self.msg_reg = mcm.StampedMsgRegister()

    def mtr_spd_callback(self,msg):
        z = np.array([[self.odom_data.x],[self.odom_data.y],[self.odom_data.theta]])
        u = np.array([[self.mtr_spd_stamp_prev.left],[self.mtr_spd_stamp_prev.right]])
        
        update_pose = mcm.euler_step(z,u,.01)

        self.odom_data.x = float(update_pose[0])
        self.odom_data.y = float(update_pose[1])
        self.odom_data.theta = float(update_pose[2])

        
        self.mtr_spd_stamp_prev.left = msg.left
        self.mtr_spd_stamp_prev.right = msg.right
    
    def send(self):
        self.odom_pub.publish(self.odom_data)

def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('Odometry', anonymous='True')
    
    odom = ROSBot_Odometry_Euler()
    rate = rospy.Rate(10)

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
