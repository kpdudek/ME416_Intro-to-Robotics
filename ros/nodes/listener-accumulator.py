#!/usr/bin/env python
"""Node that listens to chatter and accumulates messages to be published at 1/3 Hz"""

import rospy
from std_msgs.msg import String


class Accumulator(object):
    def __init__(self):
        self.sub = rospy.Subscriber('chatter', String, self.callback,queue_size=10000)
        self.pub = rospy.Publisher('chatter_repeated', String, queue_size=10)

        self.msg = String()
        self.str = ''

    def callback(self,data):
        new_str = data.data
        self.str = '%s%s'%(self.str,new_str)

    def send(self):
        if len(self.str) > 0:
            self.msg.data = self.str
            self.pub.publish(self.msg)
            print(self.str)
            self.str = ''

def main():
    """Node setup and main ROS loop"""

    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker', anonymous='True')

    rate = rospy.Rate(1.0/3.0)

    accum = Accumulator()
    while not rospy.is_shutdown():
        accum.send()
        rate.sleep()

    


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
