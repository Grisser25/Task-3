#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class EmergencyLandingNode(Node):
    def __init__(self):
        super().__init__('emergency_landing_node')

        self.declare_parameter('battery_threshold', 20.0)
        self.declare_parameter('landing_clearance_height_var', 0.35)
        self.declare_parameter('descent_speed', 0.1)   # slow visible descent
        self.declare_parameter('reposition_speed', 0.30)  # visible sideways movement
        self.declare_parameter('sector_clearance_min', 0.30)
        self.declare_parameter('touchdown_height', 0.03)

        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.height_var_thresh = self.get_parameter('landing_clearance_height_var').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.reposition_speed = self.get_parameter('reposition_speed').value
        self.sector_clearance_min = self.get_parameter('sector_clearance_min').value
        self.touchdown_height = self.get_parameter('touchdown_height').value

        self.battery_level = 100.0
        self.signal_ok = True
        self.sensor_ok = True
        self.emergency_active = False
        self.state = "MONITOR"
        self.landing_complete = False

        # Demo mode: force visible reposition before descent
        self.demo_mode = True
        self.demo_reposition_count = 0
        self.demo_reposition_max = 2

        # For making states visible during demo
        self.state_start_time = self.get_clock().now()

        self.side_depth = {
            's1': None,
            's2': None,
            's3': None,
            's4': None,
            's5': None
        }
        self.bottom_depth = None

        self.create_subscription(Float32, '/battery_percent', self.battery_callback, 10)
        self.create_subscription(Bool, '/signal_ok', self.signal_callback, 10)
        self.create_subscription(Bool, '/sensor_ok', self.sensor_callback, 10)
        self.create_subscription(Bool, '/landing_complete', self.landing_complete_callback, 10)

        self.create_subscription(Image, '/sensor_side_1/depth/image_raw', lambda msg: self.side_depth_callback(msg, 's1'), 10)
        self.create_subscription(Image, '/sensor_side_2/depth/image_raw', lambda msg: self.side_depth_callback(msg, 's2'), 10)
        self.create_subscription(Image, '/sensor_side_3/depth/image_raw', lambda msg: self.side_depth_callback(msg, 's3'), 10)
        self.create_subscription(Image, '/sensor_side_4/depth/image_raw', lambda msg: self.side_depth_callback(msg, 's4'), 10)
        self.create_subscription(Image, '/sensor_side_5/depth/image_raw', lambda msg: self.side_depth_callback(msg, 's5'), 10)
        self.create_subscription(Image, '/sensor_bottom/depth/image_raw', self.bottom_depth_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/emergency_landing_status', 10)

        # Slower loop for visibility
        self.create_timer(0.5, self.control_loop)

        self.get_logger().info("Emergency Landing Node started.")

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def signal_callback(self, msg):
        self.signal_ok = msg.data

    def sensor_callback(self, msg):
        self.sensor_ok = msg.data

    def landing_complete_callback(self, msg):
        if msg.data and not self.landing_complete:
            self.get_logger().info('Landing complete received -> switching to LANDED')
        self.landing_complete = msg.data

    def image_to_depth_array(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        arr = np.where(np.isfinite(arr), arr, np.nan)
        return arr

    def side_depth_callback(self, msg, sensor_name):
        self.side_depth[sensor_name] = self.image_to_depth_array(msg)

    def bottom_depth_callback(self, msg):
        self.bottom_depth = self.image_to_depth_array(msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f"STATE -> {new_state}")

    def time_in_state(self):
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def trigger_emergency(self):
        if self.battery_level < self.battery_threshold:
            return True
        if not self.signal_ok:
            return True
        if not self.sensor_ok:
            return True
        return False

    def evaluate_bottom_landing_zone(self):
        if self.bottom_depth is None:
            return False, 0.0

        img = self.bottom_depth
        h, w = img.shape
        crop = img[h // 3: 2 * h // 3, w // 3: 2 * w // 3]

        finite = crop[np.isfinite(crop)]
        finite = finite[finite > 0.005]

        if finite.size < 10:
            return False, 0.0

        self.get_logger().info(
            f"BOTTOM EVAL: points={finite.size}, min={np.nanmin(finite):.6f}, max={np.nanmax(finite):.6f}",
            throttle_duration_sec=0.5
        )

        depth_var = float(np.nanstd(finite))
        depth_range = float(np.nanmax(finite) - np.nanmin(finite))

        flatness_score = max(0.0, 1.0 - depth_var / self.height_var_thresh)
        roughness_score = max(0.0, 1.0 - depth_range / (self.height_var_thresh * 2.0))

        score = 0.5 * flatness_score + 0.5 * roughness_score
        safe = score > 0.55

        return safe, score

    def ground_distance_below(self):
        if self.bottom_depth is None:
            return None

        img = self.bottom_depth
        h, w = img.shape
        crop = img[h // 3: 2 * h // 3, w // 3: 2 * w // 3]

        finite = crop[np.isfinite(crop)]
        finite = finite[finite > 0.005]

        if finite.size < 20:
            return None

        return float(np.median(finite))

    def evaluate_side_clearance(self):
        clearances = {}

        for name, img in self.side_depth.items():
            if img is None:
                clearances[name] = 0.0
                continue

            finite = img[np.isfinite(img)]
            finite = finite[finite > 0.12]

            if finite.size == 0:
                clearances[name] = 0.0
            else:
                clearances[name] = float(np.percentile(finite, 30))

        return clearances

    def choose_best_direction(self, clearances):
        best_sector = max(clearances, key=clearances.get)
        return best_sector, clearances[best_sector]

    def sector_to_velocity(self, sector_name):
        mapping = {
            's1': (1.0, 0.0),
            's2': (math.cos(math.radians(72)),  math.sin(math.radians(72))),
            's3': (math.cos(math.radians(144)), math.sin(math.radians(144))),
            's4': (math.cos(math.radians(216)), math.sin(math.radians(216))),
            's5': (math.cos(math.radians(288)), math.sin(math.radians(288))),
        }
        return mapping.get(sector_name, (0.0, 0.0))

    def send_cmd(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.cmd_pub.publish(msg)

    def control_loop(self):
        self.emergency_active = self.trigger_emergency()

        if self.landing_complete and self.state == "DESCEND":
            self.set_state("LANDED")

        if self.state == "LANDED":
            self.publish_status("LANDED")
            self.send_cmd(0.0, 0.0, 0.0, 0.0)
            return

        if not self.emergency_active:
            self.set_state("MONITOR")
            self.publish_status("MONITOR")
            self.send_cmd(0.0, 0.0, 0.0, 0.0)
            return

        safe_below, score = self.evaluate_bottom_landing_zone()
        side_clearances = self.evaluate_side_clearance()
        ground_dist = self.ground_distance_below()

        if self.state == "MONITOR":
            self.set_state("STABILIZE")

        if self.state == "STABILIZE":
            self.send_cmd(0.0, 0.0, 0.0, 0.0)
            self.publish_status("STABILIZE")

            # Keep STABILIZE visible for 2 seconds
            if self.time_in_state() >= 2.0:
                self.set_state("SCAN")
            return
            
        if self.state == "SCAN":

            self.publish_status("SCAN")

            self.get_logger().info(
                f"SCAN RESULT: safe_below={safe_below}, "
                f"score={score:.2f}, ground_dist={ground_dist}, "
                f"side_clearances={side_clearances}"
            )

            # wait 2 seconds so SCAN is visible
            if self.time_in_state() < 2.0:
                return

             # DEMO MODE: force reposition twice before landing
            if self.demo_mode and self.demo_reposition_count < self.demo_reposition_max:
                self.get_logger().info("SCAN -> REPOSITION (demo)")
                self.set_state("REPOSITION")
                return

            # normal landing logic
            if safe_below:
                self.get_logger().info("SCAN -> DESCEND")
                self.set_state("DESCEND")
            else:
                self.get_logger().info("SCAN -> REPOSITION")
                self.set_state("REPOSITION")

            return

        if self.state == "REPOSITION":
            best_sector, clearance = self.choose_best_direction(side_clearances)
            self.publish_status(f"REPOSITION:{best_sector}:{clearance:.2f}")

            dx, dy = self.sector_to_velocity(best_sector)

            if clearance > self.sector_clearance_min:
                self.send_cmd(
                    vx=dx * self.reposition_speed,
                    vy=dy * self.reposition_speed,
                    vz=0.0,
                    yaw_rate=0.0
                )
            else:
                self.send_cmd(
                    vx=dx * 0.10,
                    vy=dy * 0.10,
                    vz=0.0,
                    yaw_rate=0.0
                )

            # Keep REPOSITION visible for 2 seconds
            if self.time_in_state() >= 2.0:
                if self.demo_mode and self.demo_reposition_count < self.demo_reposition_max:
                    self.demo_reposition_count += 1
                self.set_state("SCAN")
            return

        if self.state == "DESCEND":
            self.publish_status(f"DESCEND:score={score:.2f}")

            self.get_logger().info(
                f"DESCEND CHECK: landing_complete={self.landing_complete}, "
                f"ground_dist={ground_dist}, touchdown_height={self.touchdown_height:.2f}, "
                f"safe_below={safe_below}, score={score:.2f}"
            )

            if self.landing_complete:
                self.get_logger().info("DESCEND -> LANDED (landing_complete=True)")
                self.send_cmd(0.0, 0.0, 0.0, 0.0)
                self.set_state("LANDED")
                return

            if ground_dist is not None and ground_dist <= self.touchdown_height:
                self.get_logger().info(
                    f"DESCEND -> LANDED (ground_dist={ground_dist:.3f} <= {self.touchdown_height:.3f})"
                )
                self.send_cmd(0.0, 0.0, 0.0, 0.0)
                self.set_state("LANDED")
                return

            if safe_below:
                self.get_logger().info("DESCEND -> continuing descent")
                self.send_cmd(0.0, 0.0, -self.descent_speed, 0.0)
            else:
                self.get_logger().warn("DESCEND -> REPOSITION (safe_below=False)")
                self.send_cmd(0.0, 0.0, 0.0, 0.0)
                self.set_state("REPOSITION")
            return

    def destroy_node(self):
        self.send_cmd(0.0, 0.0, 0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
