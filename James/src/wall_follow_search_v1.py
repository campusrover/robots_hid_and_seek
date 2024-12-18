#!/usr/bin/env python3
import math
import tf2_ros
import rospy
from geometry_msgs.msg import Point, Pose, Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import time

class WallFollowSearch :

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

        self.Degs = None

        self.twist = Twist()

    def odom_cb(self, msg):
        self.cur_point.x = msg.pose.pose.position.x
        self.cur_point.y = msg.pose.pose.position.y
        self.cur_point.z = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        yaw = self.cur_point.z[2] * 180 / math.pi

        if yaw < 0 :
            yaw = 360 + yaw
        
        self.cur_point.z = yaw

    def scan_cb(self, msg):

        R = len(msg.ranges)
        
        #Deg 0 

        Deg0 = list(range(int(R - (R * 20 / 360)), R)) + list(range(0, int(R * 20 / 360))) #-20 to 20
        Deg45 = list(range(int(R * 25 / 360), int(R * 65 / 360)))
        Deg90 = list(range(int(R * 70 / 360), int(R * 110 / 360)))
        Deg135 = list(range(int(R * 115 / 360), int(R * 155 / 360)))
        Deg180 = list(range(int(R * 160 / 360), int(R * 200 / 360)))
        Deg225 = list(range(int(R * 205 / 360), int(R * 245 / 360)))
        Deg270 = list(range(int(R * 250 / 360), int(R * 290 / 360)))
        Deg315 = list(range(int(R * 295 / 360), int(R * 335 / 360)))

        Degs = [Deg0, Deg45, Deg90, Deg135, Deg180, Deg225, Deg270, Deg315]
        
        for i in range(len(Degs)) :

            tmp = [msg.ranges[j] for j in Degs[i]]
            
            Degs[i] = [k for k in tmp if msg.range_min < k < msg.range_max]

        DegsMap = {
            "Deg0" : [None, None],
            "Deg45" : [None, None],
            "Deg90" : [None, None],
            "Deg135" : [None, None],
            "Deg180" : [None, None],
            "Deg225" : [None, None],
            "Deg270" : [None, None],
            "Deg315" : [None, None]
        }

        j = 0

        for i in DegsMap :

            if Degs[j]:
                #AVG
                DegsMap[i][0] = sum(Degs[j]) / len(Degs[j])
                #MIN
                DegsMap[i][1] = min(Degs[j])
            else:
                DegsMap[i][0] = False
                DegsMap[i][1] = False

            j += 1

        self.Degs = DegsMap

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

    def wall_follow(self):
        Kp = -1
        Ki = 0
        Kd = 0.07
        dt = 0.1
        MaxAngularVelocity = 0.5
        LinearVelocity = 0.1
        DistanceFromWall = 0.35

        print("-----------------")

        proportional = DistanceFromWall - self.Degs['Deg90'][0]

        print(proportional)

        try:
            self.integral = self.integral + proportional * dt
        except:
            self.integral = 0

        print(self.integral)

        try:
            derivative = (proportional - self.prev_error) / dt
        except:
            derivative = 0

        print(derivative)

        output = Kp * proportional + Ki * self.integral + Kd * derivative

        self.prev_error = proportional
        
        if output < -MaxAngularVelocity:
            output = -MaxAngularVelocity
        elif output > MaxAngularVelocity:
            output = MaxAngularVelocity
        elif -MaxAngularVelocity < output < MaxAngularVelocity:
            output = output
        else:
            output = 0

        print(output)

        print("-----------------")

        self.twist.angular.z = output
        self.twist.linear.x = LinearVelocity

        self.cmd_vel_pub.publish(self.twist)

        sleep(dt)

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        sleep(0.25)  #stop all

    def facing_wall(self):
        
        if self.Degs["Deg0"][1] < 0.3:
            
            print("FACING WALL")

            self.stop()

            while self.Degs["Deg0"][1] < 0.3:
                self.twist.angular.z = -0.5
                self.cmd_vel_pub.publish(self.twist)
                # sleep(1.5) #3

            self.stop()

            return True
        
        else:

            return False

    def search(self):

        sleep(1)

        found = {
            108: False
        }

        found_all = False

        while True:

            found_all = True

            self.wall_follow()

            self.facing_wall()

            for i in found.keys():

                if self.get_pin_pose(i) != None and not found[i]:

                    found[i] = True

                else:

                    found_all = False

            if found_all == True:
                
                print("FOUND ALL")

                return

if __name__ == '__main__':
    
    rospy.init_node('wall_follow_search')

    WallFollowSearch().search()