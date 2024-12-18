#!/usr/bin/env python3

import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist 
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf


RATE = 5
PI = math.pi
POINT_RADIUS = 2


class ScanSim:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_scan_pub = rospy.Publisher("my_scan", Float32MultiArray, queue_size=1)

        self.tf_listener = tf.TransformListener()

        self.range_max = -999
        self.range_min = -999
        self.ranges = []

    #Get the robot's current position using tf
    def get_robot_position(self):
        try:
            #wait for the transform between 'map' and 'base_link'
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            x, y, z = trans  #position in 3D space
            #convert quaternion to Euler angles to get yaw
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            
            return x, y, yaw  #return position and yaw angle
        except (tf.Exception, tf.ConnectivityException, tf.LookupException) as e:
            print(f"TF Error: {e}")
            return None, None, None

    #Callback function for `self.scan_sub`
    def scan_cb(self, msg):
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = self.clean(msg.ranges)
    
    #Cleans data from LiDAR
    def clean(self, ranges):
        clean = []
        for i in ranges:
            if i > self.range_max or i < self.range_min:
                clean.append(float('nan'))
            else:
                clean.append(i)
        return clean
    
    #Calculates a point in the middle of the provided three points
    def centroid(self, points):
        (x1, y1), (x2, y2), (x3, y3) = points
        
        # Calculate centroid coordinates
        centroid_x = (x1 + x2 + x3) / 3
        centroid_y = (y1 + y2 + y3) / 3

        return centroid_x, centroid_y
    
    #Helper function to calculate coordinates based on angle and distance
    def get_coord(self, distance, radians, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        new_x = x + distance * math.cos(radians)
        new_y = y + distance * math.sin(radians)
        return new_x, new_y
    
    #Finds the middle value and index of the provided array
    def set_mid(self, array, averaged):
        if len(array) != 0:
            if len(array) % 2 == 0:
                localmax = array[len(array)//2]
                localmax_index = averaged.index(localmax)
                return localmax_index, localmax
            else:
                localmax = array[(len(array) - 1)//2]
                localmax_index = averaged.index(localmax)
                return localmax_index, localmax

    #convert lidar to be the same as yaw and combine them
    def get_angle(self, yaw, lidar):
        lidar = lidar
        if lidar > 180:
            remainder = lidar - 180
            lidar = (180 - remainder) * -1

        result = math.radians(lidar) + yaw
        return result

    #search lidar data for pockets
    def find(self, ranges):
        print("--------------------------------------")
        sensitivity = 0.2

        #get averages
        averaged = []
        min_amt = 0
        max_amt = 5
        while max_amt < len(ranges) + 1:
            value = round(np.nanmean(ranges[min_amt:max_amt]), 6)
            averaged.append(value)
            min_amt += 6
            max_amt += 6
            if max_amt > len(ranges) + 1:
                max_amt = len(ranges) + 1

        #Set up variables
        spots = []
        localmin_1 = -999
        localmin_1_index = -999
        localmax = -999
        localmax_index = -999
        localmin_2 = -999
        localmin_2_index = -999

        last_max = -999
        maxes = []

        increasing = []
        increase_sens = 0.1
        increase_lock = False
        plateau = []
        plateau_sens = 0.1
        plateau_lock = False
        decreasing = []
        decreasing_sens = 0.1

        go_back = 0

        #loop through all points in the averaged data
        for index in range(len(averaged) + go_back):
            i = averaged[index - go_back]
            print(i)

            #restart if there is a nan point
            if math.isnan(i):
                #if the arrays are all filled, add the point
                if len(increasing) != 0 and len(plateau) != 0 and len(decreasing) != 0:
                    spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
                    #go back a few points to not miss the start of a pocket
                    if index >= 2:
                        go_back += 2
                increasing = []
                plateau = []
                decreasing = []
                increase_lock = False
                plateau_lock = False
            #if increasing array is empty add the point
            elif len(increasing) == 0:
                increasing.append(i)
                localmin_1 = increasing[0]
                localmin_1_index = averaged.index(localmin_1)
            #if the distance between the last point added is large enough and the point is larger, add it to the array
            elif abs(i-increasing[-1]) >= increase_sens and i > increasing[-1]:
                increasing.append(i)
                localmin_1 = increasing[0]
                localmin_1_index = averaged.index(localmin_1)
            #if the above condition is not met and plateau is empty and the value is greater, add it to the array
            elif len(plateau) == 0 and i > increasing[-1]:
                plateau.append(i)
                increase_lock = True
                temp_index, temp_max = self.set_mid(plateau, averaged)
                localmax_index = temp_index
                localmax = temp_max
            #if above conditions are not met and the difference between the value and the last point is large enough, add the point to the array
            elif increase_lock and abs(i-plateau[-1]) >= plateau_sens:
                plateau.append(i)
                temp_index, temp_max = self.set_mid(plateau, averaged)
                localmax_index = temp_index
                localmax = temp_max
            #if the decreasing array is empty and point is less than the last plateau point, add it to the decrease array
            elif increase_lock and len(decreasing) == 0 and i < plateau[-1]:
                decreasing.append(i)
                plateau_lock = True
                localmin_2 = decreasing[-1]
                localmin_2_index = averaged.index(localmin_2)
            #if the difference between the point and the last decreasing point is large enough and the point is smaller than the last decrease point, add it to the array
            elif increase_lock and plateau_lock and abs(i-decreasing[-1]) >= decreasing_sens and i < decreasing[-1]:
                decreasing.append(i)
                localmin_2 = decreasing[-1]
                localmin_2_index = averaged.index(localmin_2)
            #else reset
            else:
                #if all the array are filled add the point
                if len(increasing) != 0 and len(plateau) != 0 and len(decreasing) != 0:
                    spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
                    #go back a few points to be sure to not miss the start of a connected pocket
                    if index >= 2:
                        go_back += 2
                if index >= 1:
                    go_back += 1
                increasing = []
                plateau = []
                decreasing = []
                increase_lock = False
                plateau_lock = False

        print(f"All: {averaged}")
        
        #calculate the coordinates of the center of each pocket
        new_spots = []
        x, y, yaw = self.get_robot_position()
        for pocket in spots:
            print(f"Spot: {pocket}")
            distance = pocket[1]
            radians = math.radians(pocket[3] * 6)

            min_x_1, min_y_1 = self.get_coord(pocket[0], self.get_angle(yaw, pocket[3] * 6), [x, y])
            max_x, max_y = self.get_coord(pocket[1], self.get_angle(yaw, pocket[4] * 6), [x, y])
            min_x_2, min_y_2 = self.get_coord(pocket[2], self.get_angle(yaw, pocket[5] * 6), [x, y])
            
            final_x, final_y = self.centroid([(min_x_1, min_y_1), (max_x, max_y), (min_x_2, min_y_2)])
            new_spots.append(final_x)
            new_spots.append(final_y)
            print(f"Coord: {final_x}, {final_y}")

        #return array with possible spots
        return new_spots

    #publish the coordinates
    def publish_data(self):
        rate = rospy.Rate(RATE)
        count = 20
        while count > 0:
            count -= 1

        while not rospy.is_shutdown():
            msg = Float32MultiArray()
            msg.data = self.find(self.ranges)
            self.my_scan_pub.publish(msg)
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('scan_sim')
    ScanSim().publish_data()