"""ros2 node that 
- subs odomatry and LiDAR scan data
- pubs ackCal data to drive - this will actually move vehicle

to trajectoryManager :
    - give trajectoryManager csv file location and max accel, decel, speed and min speed
    - might need to give its current position from odom in order to get the close waypoint from vehicle
from trajectoryManager :
    - gets full list of waypoints when first called
    - gets the close waypoint from vehicle when given odom data

to generalPurePursuit:
    - give it the list of waypoints at first
    - when close waypoint from vehicle it will use it to calculate the target waypoint and curvature to reach that waypoint
from generalPurePursuit:
    - gets the target waypoint and curvature to reach that waypoint to ackCal to change it to actual ackerman steering

to lidarPerception:
    - give it the LiDAR scan data and the close waypoint from vehicle to check if there is any obstacle in the path to that waypoint
from lidarPerception:
    - gets the obstacle data to ackCal to change it to actual ackerman steering to avoid the obstacle and reach the target waypoint and curvature

to ackermanCalculator:
    - give val from GPP the curvature and target waypoint from generalPurePursuit and calculate the actual ackerman steering to reach that target waypoint and curvature
from ackermanCalculator:
    - gets the actual ackerman steering to control the vehicle to reach the target waypoint and curvature
"""
#!/usr/bin/env python3
import trajectoryManager
testLocationStr="./wp_optimal_OX.csv"
#"./wp_optimal_XO.csv"
#"./wp_optimal_XX.csv"
#'./wp_optimal_OO.csv'
#"./wp_amcl-2026-06-11-09-12-07-clean_copy.csv"
class Dummy:
    pass

vp = Dummy()
vp.max_accel = 2.0
vp.max_decel = -2.0

vp.max_speed = 5.0

vp.min_speed = 0.5

tm = trajectoryManager.trajectoryManager(vp)
tm.load_csv(testLocationStr)
tm._compute_geometry()
print(tm._compute_speed_profile())