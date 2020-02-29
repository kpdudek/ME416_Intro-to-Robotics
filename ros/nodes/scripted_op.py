#!/usr/bin/env python
"""Simple script that waits for the key 'q' and then terminates"""

import rospy
import me416_utilities as mu
import motor_command_model as mcm
from geometry_msgs.msg import Twist
import time

def main():
    """Function to setup node and loop"""
    #Setup node and object for rate throttling
    rospy.init_node('key_op')
    rate = rospy.Rate(1)

    pub = rospy.Publisher('robot_twist', Twist, queue_size=10)
    cmd_vel = Twist()

    list_idx = 0
    filename = 'test_csv.csv'
    twist_data = mu.read_two_columns_csv(filename)

    time.sleep(.5)
    #Main ROS loop
    while not rospy.is_shutdown():
        if list_idx < len(twist_data):
            data = twist_data[list_idx]
            cmd_vel.linear.x = data[0]
            cmd_vel.angular.z = data[1]

            pub.publish(cmd_vel)
            list_idx += 1
        
        rate.sleep()


if __name__ == '__main__':
    main()
