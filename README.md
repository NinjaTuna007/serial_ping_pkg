# Serial Ping Package (ROS 2) - Succorfish Modems

A comprehensive ROS 2 package for underwater acoustic communication and ranging using Succorfish Delphis modems. This package provides nodes for acoustic pinging, distance measurement, position broadcasting, and autonomous leader-follower operations in underwater robotics applications.

## Overview

This repository contains both ROS 2 nodes and standalone scripts for working with Succorfish Delphis modems. These modems are used for underwater communication and data transfer in marine research and underwater robotics.

The Serial Ping Package enables underwater vehicles to:
- Measure distances to other vehicles using acoustic pings
- Broadcast and receive position information through acoustic modems
- Implement leader-follower behaviors with acoustic communication
- Interface with Succorfish Delphis acoustic modems via serial communication

## Features

### ROS 2 Nodes

1. **Serial Ping Node** (`serial_ping_node`)
   - Alternates between pinging two leaders
   - Calculates distances based on acoustic travel time
   - Publishes distance measurements to ROS topics

2. **SMARC Position Broadcast Node** (`smarc_pos_broadcast_node`)
   - Broadcasts robot position (latitude, longitude, depth, heading) via acoustic modem
   - Subscribes to SMARC navigation topics
   - Configurable robot identification

3. **SMARC Position Receiver Node** (`smarc_pos_receiver_node`)
   - Receives position broadcasts from other robots
   - Parses acoustic messages and publishes position data
   - Supports multiple robot configurations

4. **Serial Broadcast Receiver** (`SerialBroadcastReceiver`)
   - General-purpose acoustic message receiver
   - Publishes received messages to ROS topics

5. **Informed Leader Node** (`informed_leader_node`)
   - Advanced leader node with GPS integration
   - Supports scheduled acoustic transmissions
   - Configurable as master or slave leader

6. **Informed Follower Node** (`informed_follower_node`)
   - Follower node that responds to leader pings
   - Implements autonomous following behavior
   - Distance-based navigation

### Standalone Scripts

The package includes standalone Python scripts for direct modem interaction:

- **`scripts/pinger.py`** - Simple pinging script with CSV logging
- **`scripts/pingee.py`** - Response script for ping reception
- **`scripts/local_time.py`** - Time synchronization utilities

## Installation

### Prerequisites

- ROS 2 (tested with Humble/Iron)
- Python 3.x
- Succorfish Delphis acoustic modems

### Dependencies

Install required Python packages:
```bash
pip install -r requirements.txt
```

ROS 2 dependencies are managed through the package.xml:
- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard ROS message types
- `geographic_msgs` - Geographic coordinate messages
- `sensor_msgs` - Sensor data messages
- `pyyaml` - YAML configuration support

### Build

```bash
cd ~/your_ros2_workspace
colcon build --packages-select serial_ping_pkg
source install/setup.bash
```

## Configuration

The package uses YAML configuration files located in `config/serial_config.yaml`:

```yaml
serial:
  port: "/dev/ttyUSB0"           # Primary serial port
  port_fallback: "/dev/ttyUSB4"  # Fallback serial port
  baudrate: 9600                 # Serial communication baudrate
  leader1: "$P001"               # Ping command for leader 1
  leader2: "$P002"               # Ping command for leader 2
  sound_velocity: 1500.0         # Sound velocity in water (m/s)
  timeout_threshold: 0.5         # Communication timeout (seconds)

# Leader-follower configuration
leader_gps_topic: "/follower/leader2/core/gps"
leader_gps_msg_type: "NavSatFix"
ping_command: "$P111"
timer_period: 5.0

# Position receiver configuration
pos_receiver:
  robots:
    lolo:
      modem_id: "007"
      topic: "/relay_lolo/smarc/latlon"
    sam:
      modem_id: "111"
      topic: "/relay_sam/smarc/latlon"
```

## Usage

### Launch Files

Use the provided launch files to start nodes with proper configurations:

```bash
# Serial ping node
ros2 launch serial_ping_pkg serial_ping_node.launch

# Position broadcast node
ros2 launch serial_ping_pkg smarc_pos_broadcast_node.launch

# Leader-follower system
ros2 launch serial_ping_pkg informed_leader_node.launch
ros2 launch serial_ping_pkg informed_follower_node.launch
```

### Individual Nodes

```bash
# Start serial ping node
ros2 run serial_ping_pkg serial_ping_node

# Start position broadcast
ros2 run serial_ping_pkg smarc_pos_broadcast_node --ros-args -p robot_name:=vehicle1

# Start position receiver
ros2 run serial_ping_pkg smarc_pos_receiver_node
```

### Standalone Scripts

For direct modem testing without ROS:

```bash
# Run pinger script
python3 scripts/pinger.py

# Run pingee script (on second vehicle)
python3 scripts/pingee.py
```

## Hardware Setup

### Modem Connection

1. **Install Dependencies**:
   - Python 3.x
   - ROS 2 (for ROS nodes)

   You can install the required Python packages using pip:
   ```bash
   pip install -r requirements.txt
   ```

2. **Connect Modems**: 
   - Connect the modems to your computer via USB
   - When testing in air, ensure that the modems are aligned to ideally face each other for optimal communication

3. **Verify Ports**: Check available serial ports:
   ```bash
   ls /dev/ttyUSB*
   ```

4. **Test Communication**: Use screen to test direct communication:
   ```bash
   sudo screen /dev/ttyUSB0 9600
   ```
   Replace `/dev/ttyUSB0` with the path to the modem on your system.

### Direct Modem Communication

We use the screen command to communicate with the modems. To open a terminal session with the modem, run the following command:
```bash
sudo screen /dev/ttyUSB0 9600
```

Once you're in the terminal session, you can send commands to the modem. For example, to broadcast "Hello, World!" to all modems in the network, run the following command:
```bash
$B13Hello, World!
```

To exit the terminal session, disconnect the modem. You can also kill the screen session by pressing `Ctrl + A` followed by `K` and confirming the action.

### Physical Setup

- **Air Testing**: Align modems to face each other for optimal communication
- **Water Testing**: Deploy modems underwater with proper waterproof housings
- **Range**: Typical range varies based on environment (air: ~10m, water: 100-1000m)

## Acoustic Protocol

### Basic Commands

- **Ping Command**: `$P001` (ping node 001)
- **Broadcast**: `$B13Hello, World!` (broadcast 13-character message)
- **Timed Transmission**: `$B05HelloT01234567890123` (scheduled transmission)

### Timed Transmissions

For timed transmissions, the modem will only accept transmissions scheduled up to 2 seconds in the future. Use the following command format:
```bash
$B05HelloT01234567890123
```

To schedule a transmission, get the current system time and immediately schedule the transmission programmatically. The modem will then broadcast the message when the internal 1 MHz clock reaches the specified time.

### Timestamped Delays

- For scheduled transmissions, this is the time at which the modem begins the process of preparing the packet, powering up the transmit power supply, and then starting to emit the acoustic waveforms. So this equates to **about 7ms prior** to the acoustic emission. 
- For timestamped message arrivals, this occurs when the framesynch has been detected, so **at the end of the 30ms chirp arriving**.
- Future firmware versions might revisit this and perhaps set it such that the scheduled transmission time is when the actual acoustic emission begins.

### Message Format

The package handles various message formats:
- Distance measurements with timing data
- Position broadcasts with GPS coordinates
- Custom message payloads for inter-vehicle communication

### Timing Considerations

- **Scheduled Transmissions**: Modems accept transmissions up to 2 seconds in the future
- **Timestamp Accuracy**: 
  - Transmission timestamp: ~7ms before acoustic emission
  - Reception timestamp: End of 30ms chirp detection
- **Sound Velocity**: Configurable (default: 1500 m/s for seawater)

## Topics

### Published Topics

- `/leader1/distance` (std_msgs/Float32) - Distance to leader 1
- `/leader2/distance` (std_msgs/Float32) - Distance to leader 2  
- `/relay_{robot}/smarc/latlon` (geographic_msgs/GeoPoint) - Received positions
- `/acoustic/received_message` (std_msgs/String) - Raw acoustic messages

### Subscribed Topics

- `/{robot_name}/smarc/latlon` (geographic_msgs/GeoPoint) - Robot position
- `/{robot_name}/smarc/depth` (std_msgs/Float32) - Robot depth
- `/{robot_name}/smarc/heading` (std_msgs/Float32) - Robot heading

## Documentation

### Technical Manuals

- **User Manual/**: Succorfish Delphis technical documentation
- **NM3Firmware/**: Firmware update guides and tools
- **local/**: Additional technical notes and USB pinout diagrams

### Command Reference

- **local/commands.txt**: Complete list of modem commands
- **local/readme.md**: Additional setup notes
- **local/scripts/**: Testing and analysis utilities

## Development

### Project Structure

```
serial_ping_pkg/
├── serial_ping_pkg/           # Main ROS package
│   ├── serial_ping_node.py    # Distance measurement node
│   ├── smarc_pos_*_node.py    # Position broadcast/receive
│   ├── SerialBroadcastReceiver.py
│   ├── utils.py               # Common utilities
│   └── informed_leader_follower/
├── launch/                    # ROS launch files
├── config/                    # Configuration files
├── scripts/                   # Standalone scripts
├── data/                      # Data storage
├── local/                     # Local testing utilities
├── NM3Firmware/              # Firmware tools
└── User Manual/              # Hardware documentation
```

### Testing

Run package tests:
```bash
colcon test --packages-select serial_ping_pkg
```

### Contributing

1. Follow ROS 2 coding standards
2. Update configuration files for new features
3. Add appropriate launch files for new nodes
4. Document any new acoustic protocols

## Troubleshooting

### Common Issues

1. **Serial Port Access**: 
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

2. **Modem Not Responding**:
   - Check physical connections
   - Verify correct serial port in configuration
   - Test with screen command

3. **No Distance Measurements**:
   - Ensure modems are properly aligned
   - Check acoustic environment (avoid air bubbles)
   - Verify sound velocity setting

4. **Build Errors**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Maintainer

**Shekhar Devm Upadhyay** (sdup@kth.se)

## License

MIT License

## Acknowledgments

- Succorfish team for hardware support and documentation
- SMARC project for integration standards
- ROS 2 community for framework support

Special thanks to the Succorfish team for their support and documentation.