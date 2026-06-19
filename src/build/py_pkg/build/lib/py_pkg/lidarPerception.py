"""
to lidarPerception:
    - give it the LiDAR scan data and the close waypoint from vehicle to check if there is any obstacle in the path to that waypoint
from lidarPerception:
    - gets the obstacle data to lidarPerception to change it to actual ackerman steering to avoid the obstacle and reach the target waypoint and curvature
"""
#!/usr/bin/env python3
import math
import numpy as np

class lidarPerceptiom:
    def __init__(
        self,
        dMin: float = 0.5,
        kGain: float = 1.0,
        maxLookahead: float = 4.0,
    ):
        self.dMin = dMin
        self.kGain = kGain
        self.maxLookahead = maxLookahead

        self.centerIdx = 540

        # 270 deg lidar, 1081 beams
        self.angleIncrement = math.radians(270.0 / 1080.0)

        # only consider ±60 deg forward
        self.maxViewAngle = math.radians(60.0)

    def getLookahead(
        self,
        scan: list[float],
        speed: float,
    ) -> float:

        ppLookahead = self.dMin + speed * self.kGain

        forwardDists = []

        for i, r in enumerate(scan):

            if not math.isfinite(r):
                continue

            angle = (i - self.centerIdx) * self.angleIncrement

            if abs(angle) > self.maxViewAngle:
                continue

            forwardDist = r * math.cos(angle)

            if forwardDist > 0.0:
                forwardDists.append(forwardDist)

        if len(forwardDists) == 0:
            return self.dMin

        dFree = np.percentile(forwardDists, 10)

        lookahead = min(
            ppLookahead,
            dFree,
            self.maxLookahead,
        )

        return max(self.dMin, lookahead)