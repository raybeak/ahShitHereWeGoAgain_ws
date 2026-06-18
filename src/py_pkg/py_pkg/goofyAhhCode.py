#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE_NAME = 'lab5'
WAYPOINTS_FILENAME = 'wp_amcl-2026-05-28-07-09-38-corrected.csv'

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class GeneralPurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_general_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('pose_topic', '/pf/pose/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_timeout', 0.5)
        self.declare_parameter('waypoints_file', '')

        self.declare_parameter('drive_speed_sign', -1.0)
        self.declare_parameter('steering_sign', 1.0)

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_deg', 28.0)
        self.declare_parameter('max_steering_rate_deg', 160.0)
        self.declare_parameter('steering_smoothing', 0.35)

        self.declare_parameter('use_speed_profile', True)
        self.declare_parameter('min_speed', 2.0)
        self.declare_parameter('max_speed', 4.5)
        self.declare_parameter('max_lateral_accel', 4.0)
        self.declare_parameter('profile_max_accel', 3.5)
        self.declare_parameter('profile_max_decel', 3.0)
        self.declare_parameter('curvature_smoothing', 3)
        self.declare_parameter('cruise_speed', 1.5)

        self.declare_parameter('lookahead_gain', 0.75)
        self.declare_parameter('lookahead_min', 0.85)
        self.declare_parameter('lookahead_max', 1.2)

        self.declare_parameter('steering_trim_deg', 0.0)
        self.declare_parameter('use_auto_trim', True)
        self.declare_parameter('auto_trim_rate', 0.02)
        self.declare_parameter('auto_trim_max_deg', 3.0)
        self.declare_parameter('speed_ramp_accel', 3.5)
        self.declare_parameter('speed_preview_time', 0.15)

        self.declare_parameter('use_lidar_safety', False)
        self.declare_parameter('corridor_half_width', 0.22)
        self.declare_parameter('corridor_max_range', 6.0)
        self.declare_parameter('safety_stop_distance', 0.45)
        self.declare_parameter('safety_brake_decel', 3.5)
        self.declare_parameter('scan_timeout', 0.5)

        # 경로 이탈 한계를 회피를 위해 조금 넉넉히 늘림
        self.declare_parameter('max_path_error', 1.5)
        self.declare_parameter('search_back_points', 5)
        self.declare_parameter('search_ahead_points', 60)

        # 측면 벽 밀어내기 파라미터
        self.declare_parameter('repulsion_strength', 0.25)
        self.declare_parameter('wall_threshold', 0.30)

        # [NEW] 전방 장애물 회피 (Dodge) 파라미터
        self.declare_parameter('dodge_threshold', 1.2)   # 장애물이 이 거리(m) 이내에 오면 회피 시작
        self.declare_parameter('dodge_strength', 0.6)    # 회피 조향 강도 (클수록 확 틀어버림)

        gp = lambda name: self.get_parameter(name).value
        self.map_frame = gp('map_frame')
        self.pose_timeout = float(gp('pose_timeout'))
        self.drive_speed_sign = float(gp('drive_speed_sign'))
        self.steering_sign = float(gp('steering_sign'))
        self.wheelbase = float(gp('wheelbase'))
        self.max_steering = math.radians(float(gp('max_steering_deg')))
        self.max_steering_rate = math.radians(float(gp('max_steering_rate_deg')))
        self.steering_smoothing = float(np.clip(gp('steering_smoothing'), 0.0, 0.95))
        self.use_speed_profile = bool(gp('use_speed_profile'))
        self.min_speed = float(gp('min_speed'))
        self.max_speed = float(gp('max_speed'))
        self.max_lat_accel = float(gp('max_lateral_accel'))
        self.max_accel = float(gp('profile_max_accel'))
        self.max_decel = float(gp('profile_max_decel'))
        self.curv_smooth = int(gp('curvature_smoothing'))
        self.cruise_speed = float(gp('cruise_speed'))
        self.la_gain = float(gp('lookahead_gain'))
        self.la_min = float(gp('lookahead_min'))
        self.la_max = float(gp('lookahead_max'))
        self.steering_trim = math.radians(float(gp('steering_trim_deg')))
        self.use_auto_trim = bool(gp('use_auto_trim'))
        self.auto_trim_rate = float(gp('auto_trim_rate'))
        self.auto_trim_max = math.radians(float(gp('auto_trim_max_deg')))
        self.speed_ramp_accel = float(gp('speed_ramp_accel'))
        self.speed_preview_time = float(gp('speed_preview_time'))
        self.use_lidar_safety = bool(gp('use_lidar_safety'))
        self.corridor_half_width = float(gp('corridor_half_width'))
        self.corridor_max_range = float(gp('corridor_max_range'))
        self.safety_stop_distance = float(gp('safety_stop_distance'))
        self.safety_brake_decel = float(gp('safety_brake_decel'))
        self.scan_timeout = float(gp('scan_timeout'))
        self.max_path_error = float(gp('max_path_error'))
        self.search_back = int(gp('search_back_points'))
        self.search_ahead = int(gp('search_ahead_points'))
        
        self.repulsion_strength = float(gp('repulsion_strength'))
        self.wall_threshold = float(gp('wall_threshold'))
        
        self.dodge_threshold = float(gp('dodge_threshold'))
        self.dodge_strength = float(gp('dodge_strength'))

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, gp('drive_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/markers', marker_qos)
        self.pose_sub = self.create_subscription(Odometry, gp('pose_topic'), self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, gp('scan_topic'), self.scan_callback, 5)

        self.load_and_preprocess(self.resolve_waypoint_filepath(gp('waypoints_file')))

        self.progress_index = None
        self.current_pose = None
        self.last_pose_time = None
        self.commanded_speed = self.min_speed
        self.last_steering = 0.0
        self.auto_trim = 0.0
        self.front_clearance = float('inf')
        self.left_clearance = 3.0
        self.right_clearance = 3.0
        self.dodge_direction = 1.0  # 1.0 = 좌측으로 회피, -1.0 = 우측으로 회피
        self.last_scan_time = None
        self.control_period = 0.02

        self.create_timer(self.control_period, self.control_callback)
        self.create_timer(1.0, lambda: self.publish_markers(None, None))

    def resolve_waypoint_filepath(self, param_path):
        if param_path:
            return str(param_path)
        cwd = Path.cwd()
        candidates = [
            Path.home() / WAYPOINTS_FILENAME,
            cwd / WAYPOINTS_FILENAME,
            cwd / PACKAGE_NAME / WAYPOINTS_FILENAME,
            Path(__file__).resolve().parent.parent / WAYPOINTS_FILENAME,
        ]
        try:
            candidates.append(Path(get_package_share_directory(PACKAGE_NAME)) / WAYPOINTS_FILENAME)
        except (PackageNotFoundError, Exception):
            pass
        for c in candidates:
            if c.exists():
                return str(c)
        return str(candidates[-1])

    def load_and_preprocess(self, filename):
        points = []
        with open(filename) as f:
            for row in csv.reader(f):
                if len(row) < 2: continue
                points.append([float(row[0]), float(row[1])])
        if len(points) < 3:
            raise RuntimeError(f'Requires at least 3 waypoints, got {len(points)}')

        self.xy = np.array(points, dtype=float)
        n = len(self.xy)

        seg = np.linalg.norm(np.diff(self.xy, axis=0), axis=1)
        gap = float(np.linalg.norm(self.xy[0] - self.xy[-1]))
        self.closed = gap < 3.0 * float(np.median(seg))
        self.seg_len = np.append(seg, gap) if self.closed else seg
        self.mean_spacing = float(np.mean(self.seg_len))

        nxt = np.roll(self.xy, -1, axis=0)
        prv = np.roll(self.xy, 1, axis=0)
        if not self.closed:
            nxt[-1] = self.xy[-1] + (self.xy[-1] - self.xy[-2])
            prv[0] = self.xy[0] - (self.xy[1] - self.xy[0])
        d = nxt - prv
        self.headings = np.arctan2(d[:, 1], d[:, 0])

        curv = np.zeros(n)
        for i in range(n):
            a = np.linalg.norm(self.xy[i] - prv[i])
            b = np.linalg.norm(nxt[i] - self.xy[i])
            c = np.linalg.norm(nxt[i] - prv[i])
            area2 = abs((self.xy[i, 0] - prv[i, 0]) * (nxt[i, 1] - prv[i, 1]) 
                      - (self.xy[i, 1] - prv[i, 1]) * (nxt[i, 0] - prv[i, 0]))
            curv[i] = 2.0 * area2 / (a * b * c) if a * b * c > 1e-9 else 0.0

        if self.curv_smooth > 0:
            w = 2 * self.curv_smooth + 1
            kernel = np.ones(w) / w
            if self.closed:
                padded = np.r_[curv[-self.curv_smooth:], curv, curv[:self.curv_smooth]]
                curv = np.convolve(padded, kernel, mode='valid')
            else:
                curv = np.convolve(curv, kernel, mode='same')
        self.curvature = curv

        if self.use_speed_profile:
            self.speeds = self.build_speed_profile()
        else:
            self.speeds = np.full(n, np.clip(self.cruise_speed, self.min_speed, self.max_speed))

        self.get_logger().info(f'Loaded {n} waypoints | closed={self.closed} | L={self.seg_len.sum():.1f}m')

    def build_speed_profile(self):
        n = len(self.xy)
        v = np.sqrt(self.max_lat_accel / np.maximum(self.curvature, 1e-6))
        v = np.clip(v, self.min_speed, self.max_speed)

        n_pass = 3 if self.closed else 1
        for _ in range(n_pass):
            for i in range(n if self.closed else n - 1):
                j = (i + 1) % n
                ds = self.seg_len[i % len(self.seg_len)]
                v[j] = min(v[j], math.sqrt(v[i]**2 + 2.0 * self.max_accel * ds))
            for i in range(n - 1 if not self.closed else n - 2, -1, -1):
                j = (i + 1) % n
                ds = self.seg_len[i % len(self.seg_len)]
                v[i] = min(v[i], math.sqrt(v[j]**2 + 2.0 * self.max_decel * ds))
        return np.clip(v, self.min_speed, self.max_speed)

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.current_pose = (p.x, p.y, yaw)
        self.last_pose_time = self.get_clock().now()

    def scan_callback(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
        
        valid_ranges = np.isfinite(ranges) & (ranges > msg.range_min)
        
        # 1. 정면 장애물 거리 확인 (복도 내)
        front_valid = valid_ranges & (ranges < self.corridor_max_range)
        front_valid &= np.abs(angles) < math.radians(90.0)
        
        if not np.any(front_valid):
            self.front_clearance = float('inf')
        else:
            r = ranges[front_valid]
            a = angles[front_valid]
            x = r * np.cos(a)
            y = r * np.sin(a)
            in_corridor = (x > 0.0) & (np.abs(y) < self.corridor_half_width)
            self.front_clearance = float(x[in_corridor].min()) if np.any(in_corridor) else float('inf')
            
        # 2. 좌우 측면 밀어내기 (Wall Repulsion) 공간 확인
        left_wall_mask = (angles > math.radians(30.0)) & (angles < math.radians(90.0)) & valid_ranges
        right_wall_mask = (angles < math.radians(-30.0)) & (angles > math.radians(-90.0)) & valid_ranges
        
        self.left_clearance = float(np.min(ranges[left_wall_mask])) if np.any(left_wall_mask) else 3.0
        self.right_clearance = float(np.min(ranges[right_wall_mask])) if np.any(right_wall_mask) else 3.0
        
        # 3. [NEW] 전방 회피 방향 결정 (어느 쪽이 더 뚫려있는가?)
        # 정면 기준 좌측 대각선(0~45도)과 우측 대각선(-45~0도)의 평균 거리를 비교
        left_dodge_mask = (angles > 0.0) & (angles < math.radians(45.0)) & valid_ranges
        right_dodge_mask = (angles < 0.0) & (angles > math.radians(-45.0)) & valid_ranges
        
        left_openness = np.mean(ranges[left_dodge_mask]) if np.any(left_dodge_mask) else 3.0
        right_openness = np.mean(ranges[right_dodge_mask]) if np.any(right_dodge_mask) else 3.0
        
        # 더 넓게 열린 공간 쪽으로 회피 방향 설정 (1.0 = 좌측, -1.0 = 우측)
        self.dodge_direction = 1.0 if left_openness > right_openness else -1.0
        
        self.last_scan_time = self.get_clock().now()

    def control_callback(self):
        if self.current_pose is None or self.last_pose_time is None:
            self.stop('Waiting for pose')
            return
            
        now = self.get_clock().now()
        age = (now - self.last_pose_time).nanoseconds * 1e-9
        if age > self.pose_timeout:
            self.stop(f'Pose is old ({age:.2f}s)')
            return

        pose = self.current_pose
        closest_index, path_error = self.update_progress_index(pose)
        if path_error > self.max_path_error:
            self.stop(f'Path error {path_error:.2f}m is too large')
            return

        lookahead = float(np.clip(self.la_gain * self.commanded_speed, self.la_min, self.la_max))
        target = self.interpolate_target(closest_index, lookahead)
        if target is None:
            self.stop('No forward target')
            return
        target_xy, target_index = target

        # Pure Pursuit 기본 조향 계산
        steering, local_x, local_y = self.compute_steering(pose, target_xy)
        if local_x <= 0.0:
            self.stop('Target is behind the car')
            return

        # 장애물 회피 중인지 확인
        is_dodging = self.front_clearance < self.dodge_threshold

        # Auto-Trim 적용 (단, 회피 조작 중에는 오차가 일시적으로 커지므로 Trim 정지)
        if self.use_auto_trim and abs(self.curvature[closest_index]) < 0.15 \
                and self.commanded_speed > 0.5 and not is_dodging:
            hx = math.cos(self.headings[closest_index])
            hy = math.sin(self.headings[closest_index])
            ex = pose[0] - self.xy[closest_index, 0]
            ey = pose[1] - self.xy[closest_index, 1]
            signed_err = hx * ey - hy * ex
            self.auto_trim += -self.auto_trim_rate * signed_err * self.control_period
            self.auto_trim = float(np.clip(self.auto_trim, -self.auto_trim_max, self.auto_trim_max))

        steering += self.steering_trim + self.auto_trim

        # 측면 LiDAR 벽 밀어내기 (Wall Repulsion)
        left_repulsion = 0.0
        right_repulsion = 0.0
        
        if self.right_clearance < self.wall_threshold:
            right_repulsion = (self.wall_threshold - self.right_clearance) * self.repulsion_strength
            
        if self.left_clearance < self.wall_threshold:
            left_repulsion = -(self.wall_threshold - self.left_clearance) * self.repulsion_strength

        # [NEW] 전방 장애물 회피 (Obstacle Dodge)
        dodge_repulsion = 0.0
        if is_dodging:
            # 장애물에 가까워질수록 밀어내는 힘(urgency)이 선형적으로 거세짐 (최대 1.0)
            urgency = (self.dodge_threshold - self.front_clearance) / self.dodge_threshold
            dodge_repulsion = self.dodge_direction * urgency * self.dodge_strength
            
            # 터미널 창에 회피 방향 표시
            direction_str = "Left" if self.dodge_direction > 0 else "Right"
            self.get_logger().info(f'[OBSTACLE] Dodging {direction_str}! (Clearance: {self.front_clearance:.2f}m)', throttle_duration_sec=0.5)

        # 최종 스티어링 = Pure Pursuit + 측면 벽 밀어내기 + 전방 장애물 회피
        steering += left_repulsion + right_repulsion + dodge_repulsion
        steering = float(np.clip(steering, -self.max_steering, self.max_steering))

        # 속도 및 안전 제어
        speed = self.compute_speed(closest_index, steering, path_error)
        speed, safety_active = self.approx_lidar_safety(speed)

        max_up = self.speed_ramp_accel * self.control_period
        if speed > self.commanded_speed + max_up:
            speed = self.commanded_speed + max_up
            
        if self.steering_smoothing > 0.0:
            steering = ((1.0 - self.steering_smoothing) * steering 
                        + self.steering_smoothing * self.last_steering)
                        
        steering = self.rate_limit_steering(steering)
        self.commanded_speed = speed
        
        self.publish_drive(speed, steering)
        self.publish_markers(target_index, target_xy)

    def update_progress_index(self, pose):
        x, y, yaw = pose
        n = len(self.xy)
        if self.progress_index is None:
            dist = np.linalg.norm(self.xy - np.array([x, y]), axis=1)
            herr = np.abs(np.array([wrap_angle(yaw - h) for h in self.headings]))
            self.progress_index = int(np.argmin(dist + 0.3 * herr))
            return self.progress_index, float(dist[self.progress_index])

        best_i, best_d = self.progress_index, float('inf')
        for off in range(-self.search_back, self.search_ahead + 1):
            raw = self.progress_index + off
            if not self.closed and (raw < 0 or raw >= n):
                continue
            i = raw % n
            d = math.hypot(self.xy[i, 0] - x, self.xy[i, 1] - y)
            if d < best_d:
                best_d, best_i = d, i
                
        self.progress_index = best_i
        return best_i, best_d

    def interpolate_target(self, start_index, lookahead):
        n = len(self.xy)
        accumulated = 0.0
        prev = self.xy[start_index]
        for off in range(1, n): 
            raw = start_index + off
            if not self.closed and raw >= n:
                return (self.xy[-1].copy(), n - 1)
            i = raw % n
            cur = self.xy[i]
            ds = float(np.linalg.norm(cur - prev))
            if ds < 1e-9:
                continue
            if accumulated + ds >= lookahead:
                t = (lookahead - accumulated) / ds
                return (prev + t * (cur - prev), i)
            accumulated += ds
            prev = cur
        return (prev.copy(), start_index)

    def compute_steering(self, pose, target_xy):
        x, y, yaw = pose
        dx, dy = target_xy[0] - x, target_xy[1] - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        L = max(math.hypot(local_x, local_y), 1e-6)
        curvature = 2.0 * local_y / (L * L)
        steering = math.atan(self.wheelbase * curvature)
        return float(np.clip(steering, -self.max_steering, self.max_steering)), local_x, local_y

    def compute_speed(self, closest_index, steering, path_error):
        n = len(self.speeds)
        effective_preview_time = max(self.speed_preview_time, 0.4) 
        ahead_distance = self.commanded_speed * effective_preview_time
        ahead_indices = int(ahead_distance / max(self.mean_spacing, 1e-3))
        
        idx = (closest_index + ahead_indices) % n if self.closed else min(closest_index + ahead_indices, n - 1)
        speed = float(self.speeds[idx])
        
        if path_error > 0.6:
            speed = min(speed, self.min_speed)
        elif path_error > 0.35:
            speed = min(speed, 0.6 * self.max_speed)
            
        if abs(steering) > 0.95 * self.max_steering:
            speed = min(speed, max(self.min_speed, 0.5 * self.max_speed))
            
        return float(np.clip(speed, self.min_speed, self.max_speed))

    def approx_lidar_safety(self, speed):
        if not self.use_lidar_safety:
            return speed, False
        if self.last_scan_time is None:
            return speed, False
            
        scan_age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if scan_age > self.scan_timeout:
            return speed, False

        d = self.front_clearance
        if d == float('inf'):
            return speed, False
            
        margin = d - self.safety_stop_distance
        if margin <= 0.0:
            return 0.0, True
            
        v_allowed = math.sqrt(2.0 * self.safety_brake_decel * margin)
        if v_allowed < speed:
            return max(0.0, v_allowed), True
            
        return speed, False

    def rate_limit_steering(self, steering):
        max_delta = self.max_steering_rate * self.control_period
        steering = float(np.clip(steering, self.last_steering - max_delta, self.last_steering + max_delta))
        self.last_steering = steering
        return steering

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.drive_speed_sign * speed
        msg.drive.steering_angle = self.steering_sign * steering
        self.drive_pub.publish(msg)

    def stop(self, reason):
        self.last_steering = 0.0
        self.commanded_speed = self.min_speed
        self.publish_drive(0.0, 0.0)

    def publish_markers(self, target_index, target_xy):
        markers = MarkerArray()
        
        path = Marker()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.ns, path.id, path.type, path.action = 'path', 0, Marker.POINTS, Marker.ADD
        path.scale.x = path.scale.y = 0.045
        path.color.a = 1.0
        
        vmin, vmax = float(self.speeds.min()), float(self.speeds.max())
        span = max(vmax - vmin, 1e-6)
        
        for p, v in zip(self.xy, self.speeds):
            path.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
            t = (float(v) - vmin) / span
            path.colors.append(ColorRGBA(r=t, g=0.2, b=1.0 - t, a=1.0))
        markers.markers.append(path)

        if target_xy is not None and self.current_pose is not None:
            tgt = Marker()
            tgt.header.frame_id = self.map_frame
            tgt.header.stamp = path.header.stamp
            tgt.ns, tgt.id, tgt.type, tgt.action = 'target', 1, Marker.SPHERE, Marker.ADD
            tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.28
            tgt.color.a, tgt.color.r = 1.0, 1.0
            tgt.pose.position.x, tgt.pose.position.y = float(target_xy[0]), float(target_xy[1])
            tgt.pose.orientation.w = 1.0
            markers.markers.append(tgt)

            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = path.header.stamp
            line.ns, line.id, line.type, line.action = 'lookahead', 2, Marker.LINE_STRIP, Marker.ADD
            line.scale.x = 0.035
            line.color.a, line.color.r, line.color.g = 0.95, 1.0, 0.4
            line.points = [
                Point(x=float(self.current_pose[0]), y=float(self.current_pose[1]), z=0.03),
                Point(x=float(target_xy[0]), y=float(target_xy[1]), z=0.03)
            ]
            markers.markers.append(line)
            
        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = GeneralPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()