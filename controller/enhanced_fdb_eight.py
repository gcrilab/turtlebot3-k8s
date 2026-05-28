#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, JointState
from std_msgs.msg import Float32MultiArray
import os
import pandas as pd
import psutil


try:
    from turtlebot3_msgs.msg import SensorState
    TURTLEBOT3_MSGS_AVAILABLE = True
except ImportError:
    TURTLEBOT3_MSGS_AVAILABLE = False
    print("Warning: turtlebot3_msgs not available. Some features disabled.")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class EnhancedTurtlebot3Figure8(Node):
    def __init__(self):
        super().__init__('enhanced_turtlebot3_figure8')

        print('='*60)
        print('TurtleBot3 Figure-8 Controller')
        print('Tracking a figure-8 trajectory')
        print('='*60)

        # Shape, start, orientation options
        self.declare_parameter('a', 0.5)
        self.declare_parameter('b', 0.25)
        self.declare_parameter('sigma_ref0', math.pi/4)
        self.declare_parameter('mirror_up', True)
        self.declare_parameter('force_up', True)
        self.declare_parameter('reverse', False)

        # Gains
        self.declare_parameter('k_p1', 2.0)
        self.declare_parameter('k_p2', 2.0)
        self.declare_parameter('k_d1', 2.5)
        self.declare_parameter('k_d2', 2.5)

        # Speed law
        self.declare_parameter('v_max', 0.18)
        self.declare_parameter('v_min', 0.06)
        self.declare_parameter('v_cap', 0.22)
        self.declare_parameter('lambda', 2.0)

        # Run settings
        self.declare_parameter('rate_hz', 200.0)  
        self.declare_parameter('experiment_duration', 600.0)  

        # Load parameters
        self.a = float(self.get_parameter('a').value)
        self.b = float(self.get_parameter('b').value)
        self.sigma = float(self.get_parameter('sigma_ref0').value)
        self.mirror_up = bool(self.get_parameter('mirror_up').value)
        self.force_up = bool(self.get_parameter('force_up').value)
        self.reverse = bool(self.get_parameter('reverse').value)
        self.kp = np.diag([float(self.get_parameter('k_p1').value),
                           float(self.get_parameter('k_p2').value)])
        self.kd = np.diag([float(self.get_parameter('k_d1').value),
                           float(self.get_parameter('k_d2').value)])
        self.v_max = float(self.get_parameter('v_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_cap = float(self.get_parameter('v_cap').value)
        self.lmbd = float(self.get_parameter('lambda').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.experiment_duration = float(self.get_parameter('experiment_duration').value)

        # Odom state
        self.have_start = False
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Mapping curve->world
        self.R = np.eye(2)
        self.translation_offset = np.zeros(2)  # Changed from p0_curve/p0_world

        # Bookkeeping
        self.t = 0.0
        self.dt = 1.0 / self.rate_hz
        self.done = False
        self.last_time = None

        # Wheel velocities
        self.vR_actual = 0.0
        self.vL_actual = 0.0

        # IMU data
        self.imu_accel_x = 0.0
        self.imu_accel_y = 0.0
        self.imu_accel_z = 0.0
        self.imu_gyro_x = 0.0
        self.imu_gyro_y = 0.0
        self.imu_gyro_z = 0.0

        # Battery data
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_percentage = 0.0

        # Sensor state (TurtleBot3 specific)
        self.left_encoder = 0
        self.right_encoder = 0
        self.bumper = 0
        self.cliff = 0

        # Network statistics
        self.net_bytes_sent = 0
        self.net_bytes_recv = 0
        self.net_packets_sent = 0
        self.net_packets_recv = 0
        self.net_io_start = None

        # System metrics from Raspberry Pi
        self.pi_cpu_percent = 0.0
        self.pi_memory_percent = 0.0
        self.pi_temperature = 0.0

        # Message reception tracking
        self.odom_msg_count = 0
        self.joint_msg_count = 0
        self.imu_msg_count = 0
        self.msg_latencies = {'odom': [], 'joint': [], 'imu': []}

        # Data storage
        self.log_data = []

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.err_pub = self.create_publisher(Float32MultiArray, 'tracking_error', qos)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, qos)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', 
                                                   self.joint_states_callback, qos)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, qos)
        self.pi_metrics_sub = self.create_subscription(Float32MultiArray, 'pi_system_metrics',
                                                       self.pi_metrics_callback, qos)

        # Optional subscribers
        try:
            self.battery_sub = self.create_subscription(BatteryState, 'battery_state',
                                                        self.battery_callback, qos)
        except:
            self.get_logger().warn('Battery state topic not available')

        if TURTLEBOT3_MSGS_AVAILABLE:
            try:
                self.sensor_sub = self.create_subscription(SensorState, 'sensor_state',
                                                           self.sensor_callback, qos)
            except:
                self.get_logger().warn('Sensor state topic not available')

        # Control timer
        self.timer = self.create_timer(self.dt, self.control_cb)

        self.get_logger().info('Enhanced figure-8 controller started')
        self.get_logger().info(f'Control rate: {self.rate_hz} Hz')
        self.get_logger().info(f'Experiment duration: {self.experiment_duration}s')

    # ========== Sensor Callbacks ==========

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_accel_y = msg.linear_acceleration.y
        self.imu_accel_z = msg.linear_acceleration.z
        self.imu_gyro_x = msg.angular_velocity.x
        self.imu_gyro_y = msg.angular_velocity.y
        self.imu_gyro_z = msg.angular_velocity.z
        
        self.imu_msg_count += 1
        self.track_message_latency('imu', msg.header.stamp)

    def battery_callback(self, msg):
        """Process battery data."""
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_percentage = msg.percentage

    def pi_metrics_callback(self, msg):
        """Receive Pi system metrics: [cpu_percent, memory_percent, temperature]."""
        if len(msg.data) >= 3:
            self.pi_cpu_percent = msg.data[0]
            self.pi_memory_percent = msg.data[1]
            self.pi_temperature = msg.data[2]

    def sensor_callback(self, msg):
        """Process TurtleBot3 sensor state."""
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder
        self.bumper = msg.bumper
        self.cliff = msg.cliff

    def joint_states_callback(self, msg):
        """Extract actual wheel velocities."""
        if len(msg.name) >= 2 and len(msg.velocity) >= 2:
            try:
                left_idx = msg.name.index('wheel_left_joint')
                right_idx = msg.name.index('wheel_right_joint')
                self.vL_actual = msg.velocity[left_idx]
                self.vR_actual = msg.velocity[right_idx]
            except ValueError:
                self.vL_actual = msg.velocity[0]
                self.vR_actual = msg.velocity[1]
        
        self.joint_msg_count += 1
        if hasattr(msg, 'header'):
            self.track_message_latency('joint', msg.header.stamp)

    def track_message_latency(self, topic_name, msg_stamp):
        """Track message latency for network analysis."""
        current_time = self.get_clock().now()
        if msg_stamp.sec > 0:
            msg_time_ns = msg_stamp.sec * 1e9 + msg_stamp.nanosec
            current_time_ns = current_time.nanoseconds
            latency_ms = (current_time_ns - msg_time_ns) / 1e6
            
            if len(self.msg_latencies[topic_name]) > 100:
                self.msg_latencies[topic_name].pop(0)
            self.msg_latencies[topic_name].append(latency_ms)

    # ========== Figure-8 Trajectory Functions ==========

    def fig8_pos(self, s):
        xd = self.a * math.sin(s)
        yd = self.b * math.sin(2.0 * s)
        if self.mirror_up:
            yd = -yd
        return np.array([xd, yd])

    def fig8_dpos(self, s):
        c = math.cos(s)
        c2 = math.cos(2.0 * s)
        xd_s = self.a * c
        yd_s = 2.0 * self.b * c2
        if self.mirror_up:
            yd_s = -yd_s
        if self.reverse:
            xd_s = -xd_s
            yd_s = -yd_s
        return np.array([xd_s, yd_s])

    def fig8_ddpos(self, s):
        s1 = math.sin(s)
        s2 = math.sin(2.0 * s)
        xss = -self.a * s1
        yss = -4.0 * self.b * s2
        if self.mirror_up:
            yss = -yss
        if self.reverse:
            xss = -xss
            yss = -yss
        return np.array([xss, yss])

    def curvature_and_S(self, s):
        d = self.fig8_dpos(s)
        dd = self.fig8_ddpos(s)
        S = float(np.linalg.norm(d) + 1e-9)
        num = d[0] * dd[1] - d[1] * dd[0]
        kappa = num / max(S**3, 1e-9)
        return kappa, S

    # ========== Odometry Callback ==========

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = yaw_from_quat(msg.pose.pose.orientation)

        if not self.have_start:
            self.have_start = True
            self.t = 0.0

            # Robot's starting position in world coordinates
            p0_world = np.array([self.x, self.y])

            # Get the curve geometry at initial sigma
            p_curve = self.fig8_pos(self.sigma)
            dp_curve = self.fig8_dpos(self.sigma)

            # Calculate rotation to align curve tangent with robot heading
            th_curve = math.atan2(dp_curve[1], dp_curve[0])
            dth = self.th - th_curve
            c = math.cos(dth)
            s = math.sin(dth)
            self.R = np.array([[c, -s],
                               [s,  c]])

            # Force the figure-8 to go upward if desired
            dp0_world = self.R @ dp_curve
            if self.force_up and dp0_world[1] < 0.0:
                self.R = np.array([[1.0, 0.0],
                                   [0.0, -1.0]]) @ self.R

            # Calculate translation offset so the figure-8 center aligns with robot start
            # We want: p_world = R @ p_curve + translation_offset
            # At the center of the figure-8 (sigma=0 or pi), p_curve = [0, 0]
            # We want that center to be at p0_world
            # But we're starting at sigma=pi/4, so we need:
            # p0_world = R @ p_curve_at_sigma + translation_offset
            # translation_offset = p0_world - R @ p_curve_at_sigma
            self.translation_offset = p0_world - self.R @ p_curve
            
            self.get_logger().info('Alignment locked at first odom')
            self.get_logger().info(f'Starting at world pos: ({p0_world[0]:.3f}, {p0_world[1]:.3f})')
            self.get_logger().info(f'Curve position at sigma={self.sigma:.3f}: ({p_curve[0]:.3f}, {p_curve[1]:.3f})')
            self.get_logger().info(f'Translation offset: ({self.translation_offset[0]:.3f}, {self.translation_offset[1]:.3f})')
        
        self.odom_msg_count += 1
        self.track_message_latency('odom', msg.header.stamp)

    # ========== Control Loop ==========
    def control_cb(self):
        # Initialize network stats on first call
        if self.net_io_start is None:
            self.net_io_start = psutil.net_io_counters()

        # Update network statistics
        net_io = psutil.net_io_counters()
        self.net_bytes_sent = net_io.bytes_sent - self.net_io_start.bytes_sent
        self.net_bytes_recv = net_io.bytes_recv - self.net_io_start.bytes_recv
        self.net_packets_sent = net_io.packets_sent - self.net_io_start.packets_sent
        self.net_packets_recv = net_io.packets_recv - self.net_io_start.packets_recv

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            dt = self.dt
        else:
            dt = max(min(now - self.last_time, 0.2), 0.0)
        self.last_time = now

        if self.done or not self.have_start:
            return

        if self.t >= self.experiment_duration:
            self.cmd_pub.publish(Twist())
            self.save_log()
            self.done = True
            self.get_logger().info('Experiment complete, stopping robot.')
            return

        # Curve geometry
        p_curve = self.fig8_pos(self.sigma)
        dp_curve = self.fig8_dpos(self.sigma)
        ddp_curve = self.fig8_ddpos(self.sigma)

        # Curvature-aware speed
        kappa, S = self.curvature_and_S(self.sigma)
        u1_des = self.v_max / (1.0 + self.lmbd * abs(kappa))
        u1_des = min(max(u1_des, self.v_min), self.v_cap)

        # Phase dynamics
        sigma_dot = u1_des / max(S, 1e-9)
        if self.reverse:
            sigma_dot = -sigma_dot
        sigma_ddot = 0.0

        # World mapping (CORRECTED)
        p_world = self.R @ p_curve + self.translation_offset
        dp_world = self.R @ dp_curve * sigma_dot
        ddp_world = self.R @ (ddp_curve * (sigma_dot**2) + dp_curve * sigma_ddot)

        # Measured
        y = np.array([self.x, self.y])
        ydot_model = np.array([u1_des * math.cos(self.th),
                               u1_des * math.sin(self.th)])

        # Feedback linearization
        e = y - p_world
        e_dot = ydot_model - dp_world
        y_virtual = ddp_world - self.kd @ e_dot - self.kp @ e

        cth = math.cos(self.th)
        sth = math.sin(self.th)
        Minv = np.array([[cth, sth],
                         [-sth / max(u1_des, 1e-6), cth / max(u1_des, 1e-6)]])
        U = Minv @ y_virtual
        u2 = float(U[1])

        # Command limits
        u1_cmd = float(u1_des)
        u2_cmd = float(np.clip(u2, -1.82, 1.82))

        # Publish command
        cmd = Twist()
        cmd.linear.x = u1_cmd
        cmd.angular.z = u2_cmd
        self.cmd_pub.publish(cmd)

        # Calculate metrics
        norm_err = float(np.linalg.norm(e))

        # Average latencies
        avg_odom_latency = np.mean(self.msg_latencies['odom']) if self.msg_latencies['odom'] else 0.0
        avg_joint_latency = np.mean(self.msg_latencies['joint']) if self.msg_latencies['joint'] else 0.0
        avg_imu_latency = np.mean(self.msg_latencies['imu']) if self.msg_latencies['imu'] else 0.0

        # Log data (36 features matching circle controller)
        self.log_data.append([
            self.t,                          # Time
            self.x,                          # X position
            self.y,                          # Y position
            self.th,                         # Theta (heading)
            u1_cmd,                          # vCmd (commanded linear velocity)
            u2_cmd,                          # wCmd (commanded angular velocity)
            self.vR_actual,                  # v_R (right wheel velocity)
            self.vL_actual,                  # v_L (left wheel velocity)
            self.imu_accel_x,                # IMU linear acceleration X
            self.imu_accel_y,                # IMU linear acceleration Y
            self.imu_accel_z,                # IMU linear acceleration Z
            self.imu_gyro_x,                 # IMU angular velocity X
            self.imu_gyro_y,                 # IMU angular velocity Y
            self.imu_gyro_z,                 # IMU angular velocity Z
            self.battery_voltage,            # Battery voltage
            self.battery_current,            # Battery current
            self.battery_percentage,         # Battery percentage
            self.left_encoder,               # Left wheel encoder
            self.right_encoder,              # Right wheel encoder
            self.pi_cpu_percent,             # Pi CPU usage %
            self.pi_memory_percent,          # Pi Memory usage %
            self.pi_temperature,             # Raspberry Pi temperature (°C)
            self.net_bytes_sent,             # Network bytes sent
            self.net_bytes_recv,             # Network bytes received
            self.net_packets_sent,           # Network packets sent
            self.net_packets_recv,           # Network packets received
            norm_err,                        # Tracking error magnitude
            e[0],                            # Error X
            e[1],                            # Error Y
            self.odom_msg_count,             # Odometry message count
            self.joint_msg_count,            # Joint state message count
            self.imu_msg_count,              # IMU message count
            avg_odom_latency,                # Average odometry latency (ms)
            avg_joint_latency,               # Average joint state latency (ms)
            avg_imu_latency,                 # Average IMU latency (ms)
        ])

        # Publish tracking error
        m = Float32MultiArray()
        m.data = [self.t, self.x, self.y,
                  float(p_world[0]), float(p_world[1]),
                  float(e[0]), float(e[1]), norm_err,
                  u1_cmd, u2_cmd]
        self.err_pub.publish(m)

        # Log progress every 5 seconds
        if int(self.t) % 5 == 0 and int(self.t * 100) % 500 == 0:
            self.get_logger().info(
                f'Time {self.t:.2f}s | Err {norm_err:.3f}m | '
                f'Pi CPU {self.pi_cpu_percent:.1f}% | Pi Temp {self.pi_temperature:.1f}°C'
            )

        # Advance
        self.sigma += sigma_dot * dt
        self.t += dt

    # ========== Save Log ==========

    def save_log(self):
        """Save comprehensive logged data to CSV with auto-incrementing filename."""
        columns = [
            "Time", "X", "Y", "Theta", "vCmd", "wCmd", "v_R", "v_L",
            "imu_accel_x", "imu_accel_y", "imu_accel_z",
            "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
            "battery_voltage", "battery_current", "battery_percentage",
            "left_encoder", "right_encoder",
            "pi_cpu_percent", "pi_memory_percent", "pi_temperature",
            "net_bytes_sent", "net_bytes_recv",
            "net_packets_sent", "net_packets_recv",
            "tracking_error", "error_x", "error_y",
            "odom_msg_count", "joint_msg_count", "imu_msg_count",
            "odom_latency_ms", "joint_latency_ms", "imu_latency_ms"
        ]

        df = pd.DataFrame(self.log_data, columns=columns)

        # Auto-increment filename
        index = 1
        filename = f"enhanced_log_figure8_{index}.csv"
        while os.path.exists(filename):
            index += 1
            filename = f"enhanced_log_figure8_{index}.csv"

        try:
            df.to_csv(filename, index=False)
            self.get_logger().info(f"Enhanced log data saved to {filename}")
            self.get_logger().info(f"Total data points: {len(df)}")
            self.get_logger().info(f"Columns logged: {len(columns)}")
        except Exception as e:
            self.get_logger().error(f"Error saving log data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedTurtlebot3Figure8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
                if hasattr(node, 'log_data') and node.log_data:
                    node.save_log()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()