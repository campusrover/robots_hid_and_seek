#!/usr/bin/env python3

# rosrun hider timer.py

import rospy
from std_msgs.msg import Int32

RATE = 30

class Timer:
    def __init__(self):
        self.my_timer_pub = rospy.Publisher("my_timer", Int32, queue_size=1)

    #publish an increasing integer
    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        rate = rospy.Rate(RATE)
        count = 0

        while not rospy.is_shutdown():
            count += 1
            msg = Int32()
            msg.data = count
            print(count)
            self.my_timer_pub.publish(msg)
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('timer')
    Timer().publish_data()