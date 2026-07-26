#!/usr/bin/env python3
"""
Single Unified ROS 2 Bridge & Telemetry Parser for ESP32 Hardware Controller.
Resolves:
 1. Serial Port Ownership Conflict: Single serial client for /dev/ttyACM0 or /dev/ttyUSB0.
 2. Command Format: Uses ASCII Text Protocol (CMD_VEL vx vy wz, MODE ros, dung, etc.).
 3. Ultrasonic Distance Collision: Distinct non-greedy regex for FRONT_DISTANCE and REAR_DISTANCE.
 4. IMU Quaternion: Converts Roll/Pitch/Yaw degrees to ROS 2 standard 4D Quaternion (x, y, z, w).
 5. Auto Telemetry Stream: Relies on default 20Hz text stream from ESP32 without 't on' deadlock.
"""

import sys
import time
import math
import re
import logging
import serial
import serial.tools.list_ports

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)

# Optional ROS 2 integration if rclpy is installed on Raspberry Pi
HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Quaternion
    from sensor_msgs.msg import Imu, Range, BatteryState
    from std_msgs.msg import Float32, String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler angles in degrees to Quaternion (x, y, z, w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

def parse_telemetry_line(line: str):
    """
    Parses ESP32 text telemetry line:
    [TELEMETRY] MODE: MANUAL | STATUS: READY | BATTERY: 12.40V (95%) | FRONT_DISTANCE: 45.0cm | REAR_DISTANCE: 70.0cm | IMU: Yaw=12.5° Roll=1.2° Pitch=-0.5° | ENCODER: Dist=3.45m
    """
    try:
        mode_m = re.search(r"MODE:\s*(\w+)", line)
        status_m = re.search(r"STATUS:\s*(\w+)", line)
        batt_m = re.search(r"BATTERY:\s*([\d\.]+)V\s*\((\d+)%\)", line)
        front_m = re.search(r"FRONT_DISTANCE:\s*([\d\.]+)cm", line)
        rear_m = re.search(r"REAR_DISTANCE:\s*([\d\.]+)cm", line)
        imu_m = re.search(r"IMU:\s*Yaw=([\-\d\.]+)°\s*Roll=([\-\d\.]+)°\s*Pitch=([\-\d\.]+)°", line)
        enc_m = re.search(r"ENCODER:\s*Dist=([\-\d\.]+)m", line)

        if not (front_m and rear_m and imu_m):
            return None

        yaw = float(imu_m.group(1))
        roll = float(imu_m.group(2))
        pitch = float(imu_m.group(3))
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        return {
            "mode": mode_m.group(1) if mode_m else "UNKNOWN",
            "status": status_m.group(1) if status_m else "READY",
            "battery_v": float(batt_m.group(1)) if batt_m else 0.0,
            "battery_pct": int(batt_m.group(2)) if batt_m else 0,
            "front_distance_cm": float(front_m.group(1)),
            "rear_distance_cm": float(rear_m.group(1)),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "quaternion": (qx, qy, qz, qw),
            "encoder_dist": float(enc_m.group(1)) if enc_m else 0.0
        }
    except Exception as e:
        logging.error(f"Telemetry parse error: {e}")
        return None

def auto_find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "ttyACM" in p.device or "ttyUSB" in p.device or "COM5" in p.device:
            return p.device
    if ports:
        return ports[0].device
    return "/dev/ttyACM0"

if HAS_ROS2:
    class ESP32ROS2BridgeNode(Node):
        def __init__(self):
            super().__init__('esp32_ros2_bridge')
            self.declare_parameter('port', '/dev/ttyACM0')
            self.declare_parameter('baudrate', 115200)

            port = self.get_parameter('port').get_parameter_value().string_value
            baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

            self.get_logger().info(f"Opening Serial Port {port} at {baudrate} baud...")
            self.ser = None
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.1)
                self.get_logger().info("Serial Port opened successfully!")
            except Exception as e:
                self.get_logger().error(f"Failed to open Serial Port: {e}")

            # Subscriptions
            self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

            # Publishers
            self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
            self.pub_front_dist = self.create_publisher(Float32, '/sensor/front_distance', 10)
            self.pub_rear_dist = self.create_publisher(Float32, '/sensor/rear_distance', 10)
            self.pub_battery = self.create_publisher(Float32, '/sensor/battery_voltage', 10)

            # Timer for Serial read loop
            self.timer = self.create_timer(0.01, self.read_serial_loop)
            self.buffer = ""

        def cmd_vel_callback(self, msg: Twist):
            if self.ser and self.ser.is_open:
                vx = msg.linear.x
                vy = msg.linear.y
                wz = msg.angular.z
                cmd_str = f"CMD_VEL {vx:.3f} {vy:.3f} {wz:.3f}\n"
                self.ser.write(cmd_str.encode('utf-8'))

        def read_serial_loop(self):
            if not (self.ser and self.ser.is_open):
                return
            try:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    self.buffer += chunk
                    while "\n" in self.buffer:
                        line, self.buffer = self.buffer.split("\n", 1)
                        line = line.strip()
                        if line.startswith("[TELEMETRY]"):
                            data = parse_telemetry_line(line)
                            if data:
                                self.publish_telemetry(data)
            except Exception as e:
                self.get_logger().error(f"Error reading Serial: {e}")

        def publish_telemetry(self, data):
            # Publish IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            qx, qy, qz, qw = data["quaternion"]
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.pub_imu.publish(imu_msg)

            # Publish Ultrasonic Distances
            f_msg = Float32()
            f_msg.data = data["front_distance_cm"]
            self.pub_front_dist.publish(f_msg)

            r_msg = Float32()
            r_msg.data = data["rear_distance_cm"]
            self.pub_rear_dist.publish(r_msg)

            # Publish Battery
            b_msg = Float32()
            b_msg.data = data["battery_v"]
            self.pub_battery.publish(b_msg)

def main():
    if HAS_ROS2:
        rclpy.init()
        node = ESP32ROS2BridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        logging.info("ROS 2 environment not detected. Running in standalone CLI test mode...")
        port = auto_find_port()
        logging.info(f"Opening port {port}...")
        ser = serial.Serial(port, 115200, timeout=0.5)
        buffer = ""
        while True:
            try:
                if ser.in_waiting > 0:
                    chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line.startswith("[TELEMETRY]"):
                            parsed = parse_telemetry_line(line)
                            logging.info(f"PARSED: {parsed}")
                time.sleep(0.01)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    main()
