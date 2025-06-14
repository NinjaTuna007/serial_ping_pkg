import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import serial


class SerialBroadcastReceiver(Node):
    def __init__(self):
        super().__init__('SerialBroadcastReceiver')

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.buffer = ""
        self.timer = self.create_timer(0.5, self.read_serial)

        self.gps_pubs = {}   # Key: leader ID, Value: GPS publisher
        self.dist_pubs = {}  # Key: leader ID, Value: Distance publisher

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data
                while '\r\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\r\n', 1)
                    self.parse_message(line.strip())
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def parse_message(self, line):
        self.get_logger().info(f"Received line: {line}")

        if not line.startswith('#B00') or len(line) < 7:
            self.get_logger().warn(f"Invalid or short line: {line}")
            return

        try:
            leader_id = line[4]  # e.g., '1' from '#B001...'

            length_str = line[5:7]
            try:
                msg_len = int(length_str)
            except ValueError:
                self.get_logger().warn(f"Invalid length field: {length_str}")
                return

            expected_total_length = 7 + msg_len
            if len(line) < expected_total_length:
                self.get_logger().warn(f"Incomplete message: expected {expected_total_length} chars, got {len(line)}")
                return

            data = line[7:7 + msg_len]
            parts = data.split(',')

            if len(parts) != 3:
                self.get_logger().warn(f"Data format error, expected 3 comma-separated values: {data}")
                return

            lat = float(parts[0])
            lon = float(parts[1])
            dist = float(parts[2])

            gps_topic = f'/leader{leader_id}/gps'
            dist_topic = f'/leader{leader_id}/distance'

            # Create publishers if not already created
            if leader_id not in self.gps_pubs:
                self.gps_pubs[leader_id] = self.create_publisher(NavSatFix, gps_topic, 10)
                self.dist_pubs[leader_id] = self.create_publisher(Float32, dist_topic, 10)
                self.get_logger().info(f"Created publishers for leader {leader_id}")

            # Publish GPS
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps'
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = 0.0
            self.gps_pubs[leader_id].publish(gps_msg)
            self.get_logger().info(f"Published GPS to {gps_topic}: lat={lat}, lon={lon}")

            # Publish distance
            dist_msg = Float32()
            dist_msg.data = dist
            self.dist_pubs[leader_id].publish(dist_msg)
            self.get_logger().info(f"Published distance to {dist_topic}: {dist}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse line '{line}': {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBroadcastReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.ser.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()