#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None
        self.dist = 0.0
        self.yaw = 0.0
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        if self.old_pose == None:
            self.old_pose = msg.pose.pose
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        delta_x = cur_pose.position.x - self.old_pose.position.x
        delta_y = cur_pose.position.y - self.old_pose.position.y

        #a^2 + b^2 = c^2
        self.dist = (delta_x **2 + delta_y **2)**0.5

        #set the old pose to the current pose
        self.old_pose = cur_pose

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        position = (
            cur_orientation.x,
            cur_orientation.y,
            cur_orientation.z,
            cur_orientation.w
        )
        euler = euler_from_quaternion(position)

        #tuple is (roll, pitch, yaw)
        self.yaw = euler[2]

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        point = Point()
        point.x = self.dist
        point.y = self.yaw
        self.my_odom_pub.publish(point)
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
