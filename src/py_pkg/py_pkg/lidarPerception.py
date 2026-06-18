"""
to lidarPerception:
    - give it the LiDAR scan data and the close waypoint from vehicle to check if there is any obstacle in the path to that waypoint
from lidarPerception:
    - gets the obstacle data to ackCal to change it to actual ackerman steering to avoid the obstacle and reach the target waypoint and curvature
"""
#!/usr/bin/env python3