#!/usr/bin/env python3
import math
import tf2_ros
import rospy
from geometry_msgs.msg import Point, Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import time

class RandSearch:

    def __init__(self):
        #FIDUCIALS
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #CMD AND ODOM
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        self.cur_point = Point()
        self.cur_point.x = 0
        self.cur_point.y = 0
        self.cur_point.z = 0

        #LIDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

    def scan_cb(self, msg):

        R = len(msg.ranges)
        
        Front = list(range(int(R - (R*30/360)), R)) + list(range(0, int(R*30/360))) #330 - 0 - 30

        Front_ranges = [msg.ranges[i] for i in Front]

        max_range = msg.range_max
        min_range = msg.range_min

        Front_valid = [r for r in Front_ranges if min_range < r < max_range]

        if Front_valid:
            self.Front_avg = sum(Front_valid) / len(Front_valid)
            self.Front_min = min(Front_valid)
        else:
            self.Front_avg = math.inf
            self.Front_min = math.inf

    def get_pin_pose(self, pin_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                f'pin_{pin_id}', 
                rospy.Time(0)
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f'Failed to get pin transform: {e}')
            return None

    def odom_cb(self, msg):
        self.cur_point.x = msg.pose.pose.position.x
        self.cur_point.y = msg.pose.pose.position.y
        self.cur_point.z = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        yaw = self.cur_point.z[2] * 180 / math.pi

        if yaw < 0 :
            yaw = 360 + yaw
        
        self.cur_point.z = yaw

        print("==========")
        print(self.cur_point)
        print("==========")

    def turn_to_random_deg(self):

        prev = time.time()

        rand = math.random(1, 10)

        twist = Twist()

        while time.time() - prev < rand :

            twist.angular.z = 0.01

            self.cmd_vel_pub.publish(twist)
        
        twist.angular.z = 0

        self.cmd_vel_pub.publish(twist)

    def move_forward_random(self):

        prev = time.time()

        rand = math.random(1, 10)

        twist = Twist()

        while time.time() - prev < rand :
            
            #FORWARD
            twist.linear.x = 0.2

            self.cmd_vel_pub.publish(twist)

            #IF WALL STOP
            if self.Front_avg < 0.2 :
                
                break

        twist.linear.x = 0

        self.cmd_vel_pub.publish(twist)

    def do_360(self):

        prev = time.time()

        twist = Twist()

        while time.time() - prev < 3 :

            twist.angular.z = 0.01

            self.cmd_vel_pub.publish(twist)

        twist.angular.z = 0

        self.cmd_vel_pub.publish(twist)

    def search(self):
        
        found = {
            100 : False,
            101 : False
        }

        found_all = False

        while not found_all:

            #SCAN FOR FIDUCIALS
            found_all = True

            for i in found.keys():

                if self.get_pin_pose(i) != None and not found[i]:

                    found[i] = True

                else:

                    found_all = False

            #RANDOM MOVEMENT
            self.turn_to_random_deg()
            self.move_forward_random()

if __name__ == '__main__':

    rospy.init_node('randsearch')
    
    search = RandSearch()

    search.search()
