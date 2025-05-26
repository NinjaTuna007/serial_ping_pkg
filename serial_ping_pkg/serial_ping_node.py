import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32  # 👈 Import message type
import serial
import time
import yaml
import os

class SerialPingNode(Node):
    def __init__(self):
        super().__init__('serial_ping_node')

        # Load config file
        config = self.load_config()

        self.port = config['serial']['port']
        self.port_fallback = config['serial']['port_fallback']
        self.baudrate = config['serial']['baudrate']
        self.leader1_ping_command = config['serial']['leader1']
        self.leader2_ping_command = config['serial']['leader2']
        self.sound_velocity = config['serial']['sound_velocity']
        self.timeout_threshold = config['serial']['timeout_threshold']

        self.last_pinged = None
        self.ping_time = None
        self.timed_out = False

        # Initialize serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            if self.ser.is_open:
                self.get_logger().info(f"Serial port {self.port} opened at {self.baudrate} baud")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.get_logger().info(f"Trying fallback port: {self.port_fallback}")
            try:
                self.ser = serial.Serial(self.port_fallback, self.baudrate, timeout=1)
                if self.ser.is_open:
                    self.get_logger().info(f"Serial port {self.port_fallback} opened at {self.baudrate} baud")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open fallback serial port {self.port_fallback}: {e}")
                self.get_logger().error("No serial port available. Exiting node.")
                rclpy.shutdown()
                return

        # Create publisher
        self.distance_1_pub = self.create_publisher(Float32, 'leader1/distance', 10)
        self.distance_2_pub = self.create_publisher(Float32, 'leader2/distance', 10)

        # Timer to run every second
        self.timer = self.create_timer(0.5, self.alternate_leader_ping)

    def load_config(self):
        config_path = os.path.join(
            get_package_share_directory('serial_ping_pkg'),
            'config',
            'serial_config.yaml'
        )
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)
        
    def alternate_leader_ping(self):
        """
        Alternate between sending ping commands to leader1 and leader2, one every second.
        """
        if self.last_pinged == None:
            self.command = self.leader1_ping_command
            self.last_pinged = 1
        elif self.last_pinged == 1:
            self.command = self.leader2_ping_command
            self.last_pinged = 2
        else:
            self.command = self.leader1_ping_command
            self.last_pinged = 1

        self.get_logger().info(f"Sending command: {self.command}")

        # Send the ping command without using the send_ping method. that thing is messed up man
        try:
            self.ping_time = time.time()
            self.ser.write(self.command.encode())
            read_so_far = ""
            while True:

                # check for timeout
                if time.time() - self.ping_time > self.timeout_threshold:
                    self.get_logger().error("Ping timed out")
                    self.timed_out = True
                    break


                response = self.ser.read(self.ser.in_waiting or 1)
                read_now = response.decode('utf-8')
                # remove leading and trailing whitespaces of all kinds: \r, \n, \t
                read_now = read_now.strip()
                read_so_far += read_now
                # self.get_logger().info(f"Raw Response: {read_now}")
                
                # check that there is a T in the response
                if "T" in read_so_far:
                    # check that there are 5 digits after the T
                    if len(read_so_far.split("T")[1]) >= 5:
                        break

            # Combine the read data into a single response
            # response = b''.join(read_so_far)
            self.get_logger().info(f"Combined Response: {read_so_far}")

            if self.timed_out:
                if self.last_pinged == 1:
                    self.get_logger().error("Not publishing any distance to Leader 1")
                else:
                    self.get_logger().error("Not publishing any distance to Leader 2")
                self.timed_out = False
                return


            

            try:
                response_str = read_so_far.split("T")[1]
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Distance: {dist:.3f} meters")

                # Publish it to the right topic!
                msg = Float32()
                msg.data = dist
                if self.last_pinged == 1:
                    self.distance_1_pub.publish(msg)
                else:
                    self.distance_2_pub.publish(msg)

            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse response: {e}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")


    def send_ping(self):
        try:
            self.ser.write(self.command.encode())
            time.sleep(1)
            response = self.ser.read(self.ser.in_waiting)

            if not response:
                self.get_logger().warn("No response from serial device")
                return

            self.get_logger().info(f"Raw Response: {response.decode('utf-8')}")

            # Parse the response
            try:
                response_str = response.decode('utf-8').split("T")[1]
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Distance: {dist:.3f} meters")

                # Publish it!
                msg = Float32()
                msg.data = dist
                self.distance_pub.publish(msg)

            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse response: {e}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialPingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
