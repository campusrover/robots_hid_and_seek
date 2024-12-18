#!/usr/bin/env python3

import rospy
import actionlib
import math
import random
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import time

class PID:
    def __init__(self, Kp, Ki, Kd, target_distance=1):
        self.Kp = Kp  # proportional gain
        self.Ki = Ki  # integral gain
        self.Kd = Kd  # derivative gain

        self.distance = target_distance
        self.d_error = 0  # derivative error
        self.i_error = 0  # integral error

    def update(self, cur_distance):
        error = self.distance - cur_distance
        P = self.Kp * error 

        self.i_error += error 
        I = self.Ki * self.i_error 

        derivative = (error - self.d_error) 
        D = self.Kd * derivative

        self.d_error = error
        return P + I + D
    
class HiderRobot:
    def __init__(self):
        rospy.init_node('hider_robot')

        #move base client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        #intial state, some parameters
        self.twist = Twist()
        self.state = "move_to_goal"  # Initial state
        self.wall_distance = 0.5 
        self.corner_threshold = 0.2
        self.MOVE_SPEED = 0.2
        self.TURN_SPEED = 0.3
        self.TIMER = 60 * 5  # 5 minute timer 

        # state flags
        self.corner_detected = False
        self.goal_reached = False
        self.wall_detected = False
        self.map_data = None
        self.map_received = False

        self.PID = PID(Kp=1.0, Ki=0.0, Kd=0.1, target_distance=self.wall_distance)
        self.start_time = time()

        #timeouts used
        self.map_timeout = 10
        self.wall_follow_timeout = 20

    def wait_for_map(self): 
        while not self.map_received and rospy.Time.now() < self.map_timeout:
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)
        
        if not self.map_received:
            rospy.logerr("Couldn't receive map within timeout")
            return False
        return True

    def map_cb(self, msg): 
        rospy.loginfo("Received map data")
        self.map_data = msg
        self.map_received = True

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)

        left_front_ranges = msg.ranges[0:15]   
        right_front_ranges = msg.ranges[345:360]  
        left_ranges = msg.ranges[15:45]
        right_ranges = msg.ranges[315:345]
        #want to get different ranges + min distance from each range
        left_front_min = min(left_front_ranges, default=float('inf'))
        right_front_min = min(right_front_ranges, default=float('inf'))
        self.left_distance = min(left_ranges, default=float('inf'))
        self.right_distance = min(right_ranges, default=float('inf'))

        self.cur_wall_dist = min(min(left_ranges), min(right_ranges)) #use min to get current wall distance

        if self.left_distance < self.wall_distance or self.right_distance < self.wall_distance:
            self.wall_detected = True

        front_corner_dist = min(left_front_min, right_front_min)
        if (self.state == "follow_wall"):
            if (front_corner_dist < self.corner_threshold and  # close to front wall
                left_front_min < self.corner_threshold and     # Left front is close
                right_front_min < self.corner_threshold and    # Right front is close
                
                self.left_distance < 0.5 and                   # Left side wall there
                self.right_distance < 0.5):                    # Right side wall there
                
                    self.corner_detected = True
                    rospy.loginfo("Corner detected. Stopping.")
                    self.state = "stop"

    def generate_goal(self): 
        if not self.map_data:
            rospy.logwarn("No map data available.")
            return None

        # getting map metadata
        info = self.map_data.info
        width, height = info.width, info.height
        resolution = info.resolution
        origin_x, origin_y = info.origin.position.x, info.origin.position.y

        map_array = np.array(self.map_data.data).reshape(height, width)  # reshape map data into an array 
        
        free_cells = []
        for y in range(height):  #finds all free cells on map as potential spot for navigation
            for x in range(width):
                if map_array[y, x] == 0:
                    free_cells.append((x, y))

        if not free_cells:
            return None

        random_cell = random.choice(free_cells)  #picks random point, converts to world coords
        world_x = random_cell[0] * resolution + origin_x
        world_y = random_cell[1] * resolution + origin_y
        return (world_x, world_y)

    def move_to_goal(self):  
        goal_coords = self.generate_goal()
        if not goal_coords:
            rospy.logwarn("No valid goal found.")
            return

        goal = MoveBaseGoal()  #move_base goal message
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_coords[0]
        goal.target_pose.pose.position.y = goal_coords[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base.send_goal(goal)   # send message and wait for response
        self.move_base.wait_for_result() 
        self.goal_reached = True
    
    def follow_wall(self):  
        rate = rospy.Rate(10)
        start_follow_time = rospy.Time.now() #records start time when follow_wall called

        while not rospy.is_shutdown() and self.state == "follow_wall":
            self.twist.linear.x = self.MOVE_SPEED
            angular_correction = 0.0
            
            # uses PID to correct based on left and right distances from walls
            if self.left_distance < self.wall_distance:  
                angular_correction = self.PID.update(self.left_distance)
                self.twist.angular.z = -angular_correction
            
            elif self.right_distance < self.wall_distance: 
                angular_correction = self.PID.update(self.right_distance)
                self.twist.angular.z = angular_correction
            
            else:
                self.twist.angular.z = 0.0

            if abs(angular_correction) > self.TURN_SPEED: #stop from turning too fast
                self.twist.angular.z = self.TURN_SPEED 

            self.cmd_vel_pub.publish(self.twist)

            #checking if over timeout limit
            elasped_follow_time = rospy.Time.now() - start_follow_time
            if elapsed_follow_time > self.wall_follow_timeout:
                print("wall follow timeout, moving to goal")
                state = "move_to_goal"

    def log_time(self, event):
        self.elapsed_time = time() - self.start_time
        print(f"{self.elapsed_time:.2f} seconds")
        
        if self.elapsed_time > self.TIMER:
            timer.shutdown()

    def run(self):
        self.wait_for_map()
        timer = rospy.Timer(rospy.Duration(1), self.log_time)
        
        while not rospy.is_shutdown() and (time() - self.start_time) <= self.TIMER:
            if self.state == "move_to_goal":
                self.move_to_goal()
                rospy.loginfo("going to goal")

                if self.goal_reached:
                    if self.wall_detected:
                        rospy.loginfo("switching to wall following.")
                        self.state = "follow_wall"  
                    else:
                        rospy.loginfo("No wall detected. Generating new goal.")
                        self.goal_reached = False  # Reset for next goal

            elif self.state == "follow_wall":
                self.follow_wall()

            elif self.state == "stop":
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
            
        timer.shutdown()

if __name__ == '__main__':
    robot = HiderRobot()
    robot.run()
