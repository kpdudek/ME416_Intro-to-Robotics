#!/usr/bin/env python
"""Script that reads keyboard input and updates motor speeds accordingly"""

import rospy
import me416_utilities as mu
import motor_command_model as mcm
from geometry_msgs.msg import Twist

def print_usage():
    print('aA/dD: Decrease/Increase angular speed')
    print('sS/wW: Decrease/Increase linear speed')
    print('zZ: Zero linear speed')
    print('cC: Zero angular speed')
    print('xX: Zero linear and angular speed')
    print('')


def main():
    """Function to setup node and loop"""
    #Setup node and object for rate throttling
    rospy.init_node('key_op')
    rate = rospy.Rate(50)

    pub = rospy.Publisher('robot_twist', Twist, queue_size=10)

    #Boilerplate to get function to read keyboard
    getch = mu._Getch()

    key_to_speed = mcm.KeysToVelocities()
    cmd_vel = Twist()
    print_usage()
    print('Linear Speed: %f | Angular Speed: %f'%(key_to_speed.speed_linear,key_to_speed.speed_angular))
    print('')
    
    #Main ROS loop
    while not rospy.is_shutdown():
        key = getch()

        # Check if exit is commanded
        if key == 'q' or key == 'Q':
            rospy.loginfo("Shutdown initiated")
            rospy.signal_shutdown(
                'Shutting down initiated by %s' % rospy.get_name())
        else:
            key_to_speed.update_speeds(key)
            cmd_vel.linear.x = key_to_speed.speed_linear
            cmd_vel.angular.z = key_to_speed.speed_angular
            print(key_to_speed.action)
            print('Linear Speed: %f | Angular Speed: %f'%(key_to_speed.speed_linear,key_to_speed.speed_angular))
            print('')

            pub.publish(cmd_vel)

        rate.sleep()


if __name__ == '__main__':
    main()
