#!/usr/bin/env python3

# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="/my_ros_data/catkin_ws/hider_sim_1.yaml"
# rosrun hider hider_sim.py
# rosrun hider scan_sim.py
# rosrun hider my_odom.py
# rosrun hider timer.py

# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="/my_ros_data/catkin_ws/real_1.yaml" 

import rospy
import actionlib
import math
import random
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan

RATE = 30
PI = math.pi
POINT_RADIUS = 8

POS_X = 3
NEG_X = -3
POS_Y = 3
NEG_Y = -3

HIDE_TIME = 9000 #5 mins
#HIDE_TIME = 1800 #1 min

START_COORD = [(0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_1 = [(2.0, 4.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_2 = [(4.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_3 = [(2.0, -4.5, 0.0),
      (0.0, 0.0, 0.0, 1.0)]

DISTANCE = 0.2 #0.3
DISTANCE_2 = 0.3
STOP_DIST = 0.3 #0.3
TURN_SENS = 1

# how many hz per second to wait before cancelling move_base goal
CANCEL_GOAL = 300

hide_spot_coord = []
hide_spot_avg = []


class Hider:
    def __init__(self):
        #topic subscriptions
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_scan_sub = rospy.Subscriber('my_scan', Float32MultiArray, self.my_scan_cb)
        self.my_scan_sub = rospy.Subscriber('my_timer', Int32, self.my_timer_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)

        #client for move_base
        self.client = None

        #scan data is stored here
        self.range_max = -999
        self.range_min = -999
        self.ranges = []
        self.clean_ranges = []

        #upcoming places to visit
        self.queue = []

        #places already visited
        self.visited = []
        
        #coordinates of places that failed
        self.failed = []

        #data from odometry
        self.old_pose = None
        self.dist = 0.0
        self.not_moving = -999
        self.last_yaw = 0.0
        self.cur_yaw = 0.0
        self.dist = 0.0
        self.total_dist = 0.0
        self.start_bearing = 0.0
        self.range_max = 0.0

        #running count from timer
        self.timer = -999
    
    #callback for timer
    def my_timer_cb(self, msg):
        self.timer = msg.data
    
    #callback for odom
    def odom_cb(self, msg):
        if self.old_pose == None:
            self.old_pose = msg.pose.pose
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        rounded = round(self.dist, 4)
        #print(rounded)

        #if robot has been still or almost still for a while, cancel goal
        if rounded == 0:
            if self.not_moving == -999:
                self.not_moving = self.timer
            else:
                if self.timer - self.not_moving >= CANCEL_GOAL/2:
                    self.not_moving = -999
                    self.client.cancel_goal()
                    print("Goal took too long: cancelling")
        elif rounded <= 0.002:
            if self.not_moving == -999:
                self.not_moving = self.timer
            else:
                if self.timer - self.not_moving >= CANCEL_GOAL:
                    self.not_moving = -999
                    self.client.cancel_goal()
                    print("Goal took too long: cancelling") 
        else:
            self.not_moving = -999


    #he;per to odom_cb. updates self.dist to the distance bewteen self.old_pose and cur_pose
    def update_dist(self, cur_pose):
        delta_x = cur_pose.position.x - self.old_pose.position.x
        delta_y = cur_pose.position.y - self.old_pose.position.y

        #a^2 + b^2 = c^2
        self.dist = (delta_x **2 + delta_y **2)**0.5

        #set the old pose to the current pose
        self.old_pose = cur_pose
    
    #callback for self.my_odom_sub
    def my_odom_cb(self, msg):
        self.last_yaw = self.cur_yaw
        self.cur_yaw = self.posify(msg.y)
        self.dist = msg.x
        self.total_dist += self.dist

    #callback for self.scan_sub
    def scan_cb(self, msg):
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = msg.ranges
        self.clean_ranges = self.clean(msg.ranges)
    
    #cleans data from LiDAR scan
    def clean(self, ranges):
        clean = []
        for i in ranges:
            if i > self.range_max or i < self.range_min:
                clean.append(float('nan'))
            else:
                clean.append(i)
        return clean

    #callback for self.my_scan_sub
    def my_scan_cb(self, msg):
        pockets = msg.data

        #if queue is empty
        while len(pockets) > 0:

            #separate the data into variables
            coord_x = round(pockets[0], 3)
            coord_y = round(pockets[1], 3)
            coord = [(coord_x, coord_y, 0.0),(0.0, 0.0, 0.0, 1.0)]
            
            #get checks set up
            not_queue = True
            not_visited = True
            not_failed = True
            
            #if the queue is not 'full'
            if len(self.queue) < 10:

                #check the new coordinate against each point in the queue
                for i in self.queue:
                    spot = i
                    spot_x = round(spot[0][0], 3)
                    spot_y = round(spot[0][1], 3)

                    #check that it is not identical
                    if coord_x == spot_x and coord_y == spot_y:
                        not_queue = False
                    
                    #check that it is not too close
                    if round(abs(coord_x - spot_x) <= 0.7, 2) and round(abs(coord_y - spot_y) <= 0.7, 2):
                        not_queue = False
                
                #do the same checks for any points that have already been visited
                for i in self.visited:
                    spot = i
                    spot_x = spot[0][0]
                    spot_y = spot[0][1]

                    if coord_x == spot_x and coord_y == spot_y:
                        not_visited = False
                    
                    if round(abs(coord_x - spot_x) <= 0.7, 2) and round(abs(coord_y - spot_y) <= 0.7, 2):
                        not_visited = False
                
                #check for points that have been failed. Only check if the points are identical
                for i in self.failed:
                    spot = i
                    spot_x = spot[0][0]
                    spot_y = spot[0][1]

                    if coord_x == spot_x and coord_y == spot_y:
                        not_failed = False
                    
                #only add the new coordinate to the queue if the point passes all the checks
                if not_queue and not_visited and not_failed:
                    self.queue.append(coord)
                pockets = pockets[2:]

    #helper function for move_base
    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    #send robot to coordinate using move_base
    def goto(self, pose):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client = self.client

        # wait for action server to be ready
        client.wait_for_server()
        goal = self.goal_pose(pose)
        print(f"Going for goal: {pose}{goal}")
        client.send_goal(goal)

        success = client.wait_for_result()

        # Check status of the goal
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            print("Goal reached successfully!")
            return 1
        elif state == GoalStatus.ABORTED:
            print("Goal aborted! Could not reach the goal.")
            return 2
        elif state == GoalStatus.PREEMPTED:
            print("Goal preempted! The goal was cancelled.")
            return 3
        else:
            print(f"Goal failed with state: {state}")
            return 4
    
    #main hiding logic
    def hide(self):
        hidden = False
        start = True

        #while ros is running and the robot is not hidden
        while not rospy.is_shutdown() and hidden == False:
            coord = 0
            
            #if this is the start of the program, go to 0,0
            if self.timer != -999:
                if start:
                    self.goto([(0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)])
                    start = False

                #if the timer has reached the set time, go to the best hiding spot 
                if self.timer >= HIDE_TIME:
                    print(f"              *~*~* HIDING *~*~* ")
                    print(f"coord: {hide_spot_coord}")
                    print(f"avg: {hide_spot_avg}")

                    #iterate through all hiding spots and determine which had the lowest average lidar reading
                    smallest = 999
                    index = 999
                    if len(hide_spot_avg) > 0:
                        for i in range(len(hide_spot_avg)):
                            value = hide_spot_avg[i]
                            if value < smallest:
                                smallest = value
                                index = i
                        result = self.goto(hide_spot_coord[index])
                        if result == 1:
                            hidden = True
                    else:
                        print("No acceptable spots were found :( Hopefully lady luck is on my side!")
                        hidden = True
                #if there are coordinates in the queue, visit those
                elif len(self.queue) > 0:
                    print(self.queue)
                    print("Going to a spot!")
                    current = self.queue[0]
                    print(current)
                    saved_coord = current
                    result = self.goto(current)
                    self.visited.append(current)
                    self.queue = self.queue[1:]

                    #based on how visiting the spot went, save the coordinate to different arrays
                    if result == 1:
                        ranges = self.clean_ranges
                        average = np.nanmean(ranges)
                        hide_spot_coord.append(saved_coord)
                        hide_spot_avg.append(average)
                    elif result == 3:
                        self.failed.append(saved_coord)
                    else:
                      self.visited.append(saved_coord)  
                #if there are no coordinates in the queue go to a random location
                else:
                    if coord == 0:
                        coord = START_COORD
                    start_x = coord[0][0]
                    start_y = coord[0][1]
                    start_z = coord[0][2]
                    x = round(random.uniform((start_x - POINT_RADIUS), (start_x + POINT_RADIUS)), 1)
                    while x > POS_X or x < NEG_Y:
                        x = round(random.uniform((start_x - POINT_RADIUS), (start_x + POINT_RADIUS)), 1)
                    
                    y = round(random.uniform((start_y - POINT_RADIUS), (start_y + POINT_RADIUS)), 1)
                    while y > POS_Y or y < NEG_Y:
                        y = round(random.uniform((start_y - POINT_RADIUS), (start_y + POINT_RADIUS)), 1)
                    
                    self.goto([(x, y, 0.0),
                (0.0, 0.0, 0.0, 1.0)])
                    self.visited.append(coord)
                    coord = [(x, y, 0.0),
                    (0.0, 0.0, 0.0, 1.0)]

if __name__ == '__main__':
    rospy.init_node('hider')
    Hider().hide()
