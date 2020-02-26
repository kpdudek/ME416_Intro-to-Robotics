#!/usr/bin/env python
"""Node that listens to the twist message for the robot and publishes the individual motor speeds"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from me416_lab.msg import MotorSpeedsStamped
import motor_command_model as mcm
import me416_utilities as mu

class MotorCommand(object):
    def __init__(self):
        # Set up publishers and subscribers
        self.pub = rospy.Publisher('motor_speeds', MotorSpeedsStamped, queue_size=10)
        self.sub = rospy.Subscriber('robot_twist', Twist, callback=self.twist_callback,queue_size=10)

        # Set up the motor feedback object
        self.motor_cmd = MotorSpeedsStamped()

        # Set up the motor objects with an offset for the right motor
        self.speed_offset = 0.86
        self.L_motor = mu.MotorSpeedLeft()
        self.R_motor = mu.MotorSpeedRight(self.speed_offset)

    def twist_callback(self,data):
        # Get differential drive speeds
        left,right = mcm.twist_to_speeds(data.linear.x,data.angular.z)
        
        # Set motor speed feedback data
        self.motor_cmd.left = left
        self.motor_cmd.right = right

        # Send PWM pulses to motors according to desired motor speed
        self.L_motor.set_speed(left)
        self.R_motor.set_speed(right)

        # Stamp time
        self.motor_cmd.header.stamp = rospy.Time.now()
        
        # Publish feedback
        self.pub.publish(self.motor_cmd)

def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('Motor_Command', anonymous='True')

    # Set up the motor command object
    motor_command = MotorCommand()
        
    # Keep ros alive
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
