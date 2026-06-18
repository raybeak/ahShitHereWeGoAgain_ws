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
import math
import trajectoryManager as TJM
import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

testLocationStr = ""
#csv location go here

class vehicleParams:
    pass
vp = vehicleParams()
vp.max_accel = 2.0
vp.max_decel = -2.0
vp.max_speed = 5.0
vp.min_speed = 0.5

class coreNode(Node):
    tm = TJM.trajectoryManager(vp)
    tm.load_csv(testLocationStr)
    tm._compute_geometry()
    map=list(tm._compute_speed_profile())

    def __init__(self):
        super().__init__("coreNode")

        self.odomMsg = None
        self.scanMsg = None

        self.odomSub = self.create_subscription(
            Odometry,
            "/odom",
            self.odomCallback,
            10,
        )

        self.scanSub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scanCallback,
            10,
        )

        self.drivePub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10,
        )

        self.timer = self.create_timer(
            0.1,  # 40 Hz
            self.controlLoop,
        )

        self.get_logger().info("DriveNode started")

    def odomCallback(self, msg: Odometry) -> None:
        self.odomMsg = msg

    def scanCallback(self, msg: LaserScan) -> None:
        self.scanMsg = msg

    def controlLoop(self) -> None:
        if self.odomMsg is None:
            return

        if self.scanMsg is None:
            return

        currentSpeed = math.sqrt(
            self.odomMsg.twist.twist.linear.x ** 2
            + self.odomMsg.twist.twist.linear.y ** 2
        )

        frontIndex = len(self.scanMsg.ranges) // 2
        frontDistance = self.scanMsg.ranges[frontIndex]

        driveMsg = AckermannDriveStamped()
        driveMsg.header.stamp = self.get_clock().now().to_msg()
        driveMsg.header.frame_id = "base_link"

        # Example logic
        if frontDistance < 1.0:
            driveMsg.drive.speed = 0.0
            driveMsg.drive.steering_angle = 0.0
        else:
            driveMsg.drive.speed = 1.0
            driveMsg.drive.steering_angle = 0.0

        self.drivePub.publish(driveMsg)

        self.get_logger().debug(
            f"speed={currentSpeed:.2f}, frontDist={frontDistance:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = coreNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
