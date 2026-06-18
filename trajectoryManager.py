"""
from coreNodeRos2.py :
    - get csv file location and max accel, decel, speed and min speed
    - might need to get its current position from odom in order to get the close waypoint from vehicle
to coreNodeRos2.py :
    - gives full list of waypoints when first called
    - gives the close waypoint from vehicle when given odom data
"""

#!/usr/bin/env python3
import os
import csv
import math
from dataclasses import dataclass
from statistics import median

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
        self.maxDecel = abs(vehicleParams.max_decel)
        self.maxSpeed = vehicleParams.max_speed
        self.minSpeed = vehicleParams.min_speed
        self.trajectory = []
        self.current_index = 0
        
        self.waypoints: list[waypoint] = []
        self.is_closed: bool = False
        self.segment_lengths = []
        
    """ 
    def __setattr__(self, name, value):
        if name == "maxSpeed":
            print("SETTING maxSpeed ->", value, type(value))
        super().__setattr__(name, value)"""

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

                        #print(wp)#<---debug

                    except ValueError:  pass
    
                return True
            
        except FileNotFoundError:
            print(f"File not found: {file_path}")
            return False
    
    def _compute_geometry(self) -> None:
        print("starting geometry computaion")
        N = len(self.waypoints)
        if N < 3:   return
        self.segment_lengths.clear()

        #compute and store the length of each segment between each waypoint into self.segment_lengths
        for i in range(N-1):
            waypointFirst = self.waypoints[i]
            waypointSecond = self.waypoints[i + 1]
            length = math.sqrt((waypointSecond.x - waypointFirst.x) ** 2 + (waypointSecond.y - waypointFirst.y) ** 2)
            self.segment_lengths.append(length)
            
            
            #print(length)#<--debug
        

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

            area2=abs((curr_wp.x - prev_wp.x)*(next_wp.y - curr_wp.y) - (curr_wp.y - prev_wp.y)*(next_wp.x - curr_wp.x))
            area=area2/2.0


            curr_wp.curvature = max(0.0 , min((4*area)/(side_a*side_b*side_c), 15.0)) if (side_a*side_b*side_c)>1e-9 else 0.0

    def _compute_speed_profile(self) -> None:
        print("starting speed computaion")

        N = len(self.waypoints)
        if N <= 3:   return

        #compute speed profile based on curvature and max speed, max accel and max decel
        for i in range(N):
            prev_idx=((i-1+N)%N if self.is_closed else max(i-1,0))
            next_idx=((i+1)%N if self.is_closed else min(N-1,i+1))

            smth_curv = ((self.waypoints[prev_idx].curvature
                          +self.waypoints[i].curvature
                          +self.waypoints[next_idx].curvature)/3.0
                        )
            
            curr_wp = self.waypoints[i]
            
            #compute speed based on curvature
            if smth_curv > 1e-5:
                curr_wp.speed = max(self.minSpeed,
                                    min(math.sqrt(self.maxAccele/smth_curv),self.maxSpeed))
            else:
                curr_wp.speed = self.maxSpeed

        total_passes = 3 if self.is_closed else 1

    #for p in range(total_passes):
        #forward pass to ensure acceleration limits are respected
        for p in range(total_passes):
            for i in range(N-1):
                curr_idx = i
                next_idx = (i+1)%N if self.is_closed else i+1

                if self.is_closed and next_idx == N:
                    break

                curr_wp = self.waypoints[curr_idx]
                next_wp = self.waypoints[next_idx]
                ds = self.segment_lengths[curr_idx]

                max_reachable_speed = math.sqrt((curr_wp.speed**2)+2*self.maxAccele*ds)
                next_wp.speed = min(next_wp.speed, max_reachable_speed)

        #backward pass to ensure deceleration limits are respected
        for p in range(total_passes):
            if self.is_closed:

                for i in range(N-1,-1,-1):

                    curr_idx=i
                    next_idx=(i+1)%N if self.is_closed else i+1
                    if not self.is_closed and next_idx ==N:
                        continue

                    curr_wp = self.waypoints[curr_idx]
                    next_wp = self.waypoints[next_idx]
                    ds = self.segment_lengths[curr_idx]

                    max_decal_speed = math.sqrt((next_wp.speed**2) + 2*self.maxDecel*ds)
                    curr_wp.speed = min(curr_wp.speed, max_decal_speed)

        #curvatures = [w.curvature for w in self.waypoints]
        #print(sorted(curvatures)[:20])
        #print(sorted(curvatures)[-20:])
        #print(
        #    min(w.curvature for w in self.waypoints),
        #    max(w.curvature for w in self.waypoints))

        maxPlanSpeed = max(wp.speed for wp in self.waypoints[:])

        for i in range(N):
                
            curr_wp = self.waypoints[i]
            normalizedRatio = curr_wp.speed/maxPlanSpeed
            curr_wp.speed = max(min(normalizedRatio,1.0),0.05)
            print(curr_wp.speed)
            #just same as np.clip((curr_wp.speed/max_temp_val)*weightsValRatio,0.05,1.0)
        return self.waypoints

    #def _close_node(self):
    #    #get tf/odom and find close index from maplist
    #    pass


    
# #simple test run when executed directly
#file_path = './wp_amcl-2026-06-11-09-12-07-clean_copy.csv'
#
#if __name__ == '__main__':
#    class _VP:
#        max_accel = 1.0
#        max_decel = 1.0
#        max_speed = 1.0
#        min_speed = 0.3
#
#    tm = trajectoryManager(_VP())
#    tm.load_csv(file_path)
#    tm._compute_geometry()
#    tm._compute_speed_profile()
#
#    for i in range(min(10, len(tm.waypoints))):
#        wp = tm.waypoints[i]
#        print(wp.x, wp.y, wp.theta, wp.speed, wp.curvature)
#
#    print(len(tm.waypoints))
#    print(tm.is_closed)
#    print(min(w.speed for w in tm.waypoints))
#    print(max(w.speed for w in tm.waypoints))
#    start = tm.waypoints[0]
#    end = tm.waypoints[-1]
#
#    print(
#        math.hypot(
#            start.x - end.x,
#            start.y - end.y
#        )
#    )