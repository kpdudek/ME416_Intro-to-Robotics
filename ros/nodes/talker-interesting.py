#!/usr/bin/env python
"""Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic"""

import rospy
from std_msgs.msg import String
import time
class Talker(object):
    def __init__(self,words):
        self.pub = rospy.Publisher('chatter', String, queue_size=100)
        self.msg = String()
        self.words = words
        self.idx = 0
        self.width = 5

    def set_msg(self):
        if (self.idx+self.width) < len(self.words):
            self.msg.data = self.words[self.idx:self.idx+self.width]
        else:
            self.msg.data = self.words[self.idx:]
        
        self.idx+=self.width

    def send(self):
        if len(self.msg.data)>0:
            self.pub.publish(self.msg)

def set_words():
    words = 'In ROSBot, you can control the speed of the two motors to change the overall linear and angular speed of the entire robot. See Figure 1 for an illustration of the axes and robot velocities. In this question you will make'
    return words

def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker', anonymous='True')

    #Set rate to use (in Hz)
    rate = rospy.Rate(10)

    words = set_words()
    talker = Talker(words)
    time.sleep(.5) # ROS was dropping the first message so this delay lets initialization finish
    while not rospy.is_shutdown():
        talker.set_msg()
        talker.send()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
