import os
import csv
import math
import numpy as np
from dataclasses import dataclass
from statistics import median

"""pls feed this class with csv file that have atlest x,y,theta and location string and it will do its job just fine"""

@dataclass
#waypoint class to store the x, y, theta, speed and heading of each waypoint
class waypoint:
    x: float
    y: float
    theta: float # this is also heading at same time
    speed: float = 0.0
    curvature: float = 0.0

class trajectoryManager:
    def __init__(self,vehicleParams):
        self.maxAccele = vehicleParams.max_accel
        self.maxDecel = vehicleParams.max_decel
        self.maxSpeed = vehicleParams.max_speed
        self.minSpeed = vehicleParams.min_speed
        self.trajectory = []
        self.current_index = 0
        
        self.waypoints: list[waypoint] = []
        self.is_closed: bool = False
        self.segment_lengths = []

    def load_csv(self, file_path: str) -> bool:
        # load csv file into waypoints with float x, y, theta
        try:
            with open(file_path, 'r') as path:
                reader = csv.reader(path)
                for row in reader:
                    #for each row, check if it has at least 3 values, if not skip it
                    if len(row) < 3:    continue
                    try:
                        val_x = float(row[0])
                        val_y = float(row[1])
                        val_theta = float(row[2])

                        wp = waypoint(x=val_x, y=val_y, theta=val_theta)
                        self.waypoints.append(wp)

                    except ValueError:  pass
    
                return True
            
        except FileNotFoundError:
            print(f"File not found: {file_path}")
            return False
    
    def _compute_geometry(self) -> None:
    
        N = len(self.waypoints)
        if N < 3:   return
        self.segment_lengths.clear()

        #compute and store the length of each segment between each waypoint into self.segment_lengths
        for i in range(N-1):
            waypointFirst = self.waypoints[i]
            waypointSecond = self.waypoints[i + 1]
            length = math.sqrt((waypointSecond.x - waypointFirst.x) ** 2 + (waypointSecond.y - waypointFirst.y) ** 2)
            self.segment_lengths.append(length)
        

        #check it is closed or not by checking the distance between the first and last waypoint, if it is less than 0.1, it is closed
        start_wp = self.waypoints[0]
        end_wp = self.waypoints[N-1]
        gap = math.sqrt((start_wp.x - end_wp.x) ** 2 + (end_wp.y - start_wp.y) ** 2)

        if gap < 3.0*median(self.segment_lengths):
            self.is_closed = True
            self.segment_lengths.append(gap)
        else:
            self.is_closed = False
        
        for i in range(N):
            curr_wp = self.waypoints[i]
            #heading using theta that we aready have

            if not self.is_closed and (i == 0 or i == N-1):
                curr_wp.curvature = 0.0
                continue

            if self.is_closed:
                prev_idx = ((i-1+N)%N)
                next_idx = ((i+1)%N)
            else:
                prev_idx = i-1
                next_idx = i+1

            prev_wp = self.waypoints[prev_idx]
            next_wp = self.waypoints[next_idx]
            #idk about this one lets see
            

            side_a=math.sqrt((curr_wp.x - prev_wp.x)**2 + (curr_wp.y - prev_wp.y)**2)
            side_b=math.sqrt((next_wp.x - curr_wp.x)**2 + (next_wp.y - curr_wp.y)**2)
            side_c=math.sqrt((next_wp.x - prev_wp.x)**2 + (next_wp.y - prev_wp.y)**2)

            area2=abs((curr_wp.x - prev_wp.x)*(next_wp.y - curr_wp.y) - (curr_wp.y - prev_wp.y)*(next_wp.x - prev_wp.x))
            area=area2/2.0


            curr_wp.curvature = np.clip((4*area)/(side_a*side_b*side_c), 0.0, 10.0) if (side_a*side_b*side_c)>1e-9 else 0.0

