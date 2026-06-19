"""
to generalPurePursuit:
    - give it the list of waypoints at first
    - when close waypoint from vehicle it will use it to calculate the target waypoint and curvature to reach that waypoint
from generalPurePursuit:
    - gets the target waypoint and curvature to reach that waypoint to ackCal to change it to actual ackerman steering
"""
#!/usr/bin/env python3

#class generalPurePursuit():
#    def __init__(self,currentPose,targetWaypoint,wheelbase):
#        currentPose
#        targetWaypoint
#        wheelbase
#        pass
import math

class generalPurePursuit:

    def __init__(self, wheelbase: float):
        self.wheelbase = wheelbase

    def computeCurvature(
        self,
        poseX :float,
        poseY :float,
        poseYaw :float,
        targetX :float,
        targetY :float
    ):
        print("drive logic on")
        dx = targetX - poseX
        dy = targetY - poseY

        localX = (
            math.cos(poseYaw) * dx +
            math.sin(poseYaw) * dy
        )

        localY = (
            -math.sin(poseYaw) * dx +
            math.cos(poseYaw) * dy
        )

        lookahead = max(
            math.hypot(localX, localY),
            1e-6
        )

        curvature = (
            2.0 * localY /
            (lookahead * lookahead)
        )

        print(curvature)
        return curvature