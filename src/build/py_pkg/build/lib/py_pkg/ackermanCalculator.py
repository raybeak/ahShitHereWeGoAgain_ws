"""
to ackermanCalculator:
    - give val from GPP the curvature and target waypoint from generalPurePursuit and calculate the actual ackerman steering to reach that target waypoint and curvature
from ackermanCalculator:
    - gets the actual ackerman steering to control the vehicle to reach the target waypoint and curvature
"""
#!/usr/bin/env python3
import math

class AckermannCalculator:

    def __init__(
        self,
        wheelbase,
        maxSteeringDeg
    ):
        self.wheelbase = wheelbase

        self.maxSteering = math.radians(
            maxSteeringDeg
        )

    def curvatureToSteering(
        self,
        curvature
    ):

        steering = math.atan(
            self.wheelbase * curvature
        )

        steering = max(
            -self.maxSteering,
            min(
                self.maxSteering,
                steering
            )
        )

        return steering