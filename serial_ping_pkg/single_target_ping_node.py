#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial
import time

class SingleTargetPingNode(Node):
    def __init__(self):
        super().__init__('single_target_ping_node')

        # Load config from YAML and declare parameters
        config = load_yaml_config('serial_ping_pkg', 'single_target_ping_config.yaml')
        
        # Serial parameters
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('serial.timeout_threshold', config['serial']['timeout_threshold'])
        
        # Single target ping parameters
        self.declare_parameter('robot_name', config['single_target_ping']['robot_name'])
        self.declare_parameter('ping_command', config['single_target_ping']['ping_command'])
        self.declare_parameter('timer_period', config['single_target_ping']['timer_period'])
        self.declare_parameter('sound_velocity', config['single_target_ping']['sound_velocity'])
        self.declare_parameter('distance_topic_suffix', config['single_target_ping']['distance_topic_suffix'])

        # Get parameter values
        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.timeout_threshold = self.get_parameter('serial.timeout_threshold').get_parameter_value().double_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ping_command = self.get_parameter('ping_command').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.sound_velocity = self.get_parameter('sound_velocity').get_parameter_value().double_value
        self.distance_topic_suffix = self.get_parameter('distance_topic_suffix').get_parameter_value().string_value

        # Initialize serial connection
        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Position tracking
        self.current_position = None
        
        # Subscribe to position topic
        position_topic = f"/{self.robot_name}/smarc/latlon"
        self.get_logger().info(f"Subscribing to position topic: {position_topic}")
        self.position_subscription = self.create_subscription(
            GeoPoint,
            position_topic,
            self.position_callback,
            10
        )

        # Create distance publisher
        distance_topic = f"/{self.robot_name}/{self.distance_topic_suffix}"
        self.get_logger().info(f"Publishing distance to: {distance_topic}")
        self.distance_publisher = self.create_publisher(Float32, distance_topic, 10)

        # Create timer for periodic pinging
        self.ping_timer = self.create_timer(self.timer_period, self.ping_callback)
        
        self.get_logger().info(f"Single Target Ping Node initialized:")
        self.get_logger().info(f"  - Robot name: {self.robot_name}")
        self.get_logger().info(f"  - Ping command: {self.ping_command}")
        self.get_logger().info(f"  - Timer period: {self.timer_period}s")
        self.get_logger().info(f"  - Sound velocity: {self.sound_velocity} m/s")

    def position_callback(self, msg):
        """Callback to update current position from latlon topic"""
        self.current_position = msg
        self.get_logger().debug(f"Position updated: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

    def ping_callback(self):
        """Timer callback to periodically ping the target"""
        # Log current position status for debugging, but don't block pinging
        if self.current_position is None:
            self.get_logger().debug("No position data available yet, but continuing with ping")
        else:
            self.get_logger().debug(f"Current position: lat={self.current_position.latitude:.6f}, lon={self.current_position.longitude:.6f}")

        self.get_logger().info(f"Sending ping command: {self.ping_command}")
        
        try:
            ping_time = time.time()
            self.ser.write(self.ping_command.encode())
            
            read_so_far = ""
            timed_out = False
            
            while True:
                if time.time() - ping_time > self.timeout_threshold:
                    self.get_logger().error("Ping timed out")
                    timed_out = True
                    break

                response = self.ser.read(self.ser.in_waiting or 1)
                read_now = response.decode('utf-8').strip()
                read_so_far += read_now

                # Look for response pattern with "T" followed by timing data
                if "T" in read_so_far:
                    if len(read_so_far.split("T")[1]) >= 5:
                        break

            self.get_logger().info(f"Ping response: {read_so_far}")

            if timed_out:
                self.get_logger().error("Not publishing distance due to timeout")
                return

            try:
                # Parse response and calculate distance
                response_str = read_so_far.split("T")[1]
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Calculated distance: {dist:.3f} meters")

                # Always publish distance regardless of position availability
                msg = Float32()
                msg.data = dist
                self.distance_publisher.publish(msg)

            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse ping response: {e}")

        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SingleTargetPingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()