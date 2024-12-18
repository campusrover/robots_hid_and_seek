# Introduction

Hide and seek game consists of two separate teams; Hiders and Seekers. Hider's main objective is to find the best hiding spot that will prevent them from being found for the longest time. There is no limit to using sensory devices but there is a limit to actuators, specifically motor speed. There also is no limit to using any ROS topics. Same rules apply for Seekers. The Seeker's main objective is to find the Hiders. To better detect Hiders, Hiders will have fiducials attached on all four sides of the robot. This imposes one additional mandatory task for seekers which is having to seek for fiducials at all times.


# Summary
## James
Multiple Seeking algorithms that are still under improvement. One that is looking the most promising is the "wall follow search." as it is reliable and is almost guaranteed to find the robot. However, if the algorithm runs "luckily", the 'random search' algorithm in theory can find hider by, best case, traveling minimum distance and speed. This seeker algorithms are an amalgamation of wall follow, odom, base move, and fiducial.
## Daphne
The hiding algorithm I have created gets the robot to explore the space randomly for a period of time. When this time is over, the robot will go to a wall it detects, and follows it until it finds a corner where the robot will then hide. If the robot is following a wall for a while without finding a suitable corner, it will go back to exploring for a bit before finding another wall with the hope of finding a good spot before the time is up.
## Chloe
I designed a hiding algorithm that identifies areas that are walled in on three sides - pockets - using lidar, and navigates to them using move_base. My project runs using *hider_real.py*, *scan_sim.py*, *timer.py*, and *my_odom.py*. The identification of pockets and coordinate calculation is done in *scan_sim.py* and move_base is handled in *hider_real.py*. *my_odom.py* and *timer.py* are helper programs.

In order to better develop my pocket detection program, I graphed the lidar data in the spreadsheet below. The spreadsheet also includes notes on the debugging process and the success of the tests.

Data and progression: [https://docs.google.com/spreadsheets/d/1lpmWlnTqMWZSKndbC97cu0bUynrpZY1xOJgwtPlUaSQ/edit?usp=sharing]

## map_files

This folder contains the map files used to run move_base for the demo


# Video
# Lab Notebook project report
[https://github.com/campusrover/labnotebook2/blob/main/docs/reports/2024/HideAndSeek.md]
