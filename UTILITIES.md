# Additional Utilities and Documentation

This document provides an overview of the additional utilities, scripts, and documentation included in this package.

## Standalone Scripts (`scripts/`)

### Core Testing Scripts

- **`pinger.py`** - Acoustic pinger script for distance measurement testing
  - Sends ping commands to acoustic modems
  - Logs responses with timestamps to CSV files
  - Calculates distances using configurable sound velocity
  - Usage: `python3 scripts/pinger.py`

- **`pingee.py`** - Acoustic responder script for ping reception
  - Listens for incoming pings and responds automatically
  - Implements timed transmission scheduling
  - Used for testing acoustic communication loops
  - Usage: `python3 scripts/pingee.py`

- **`local_time.py`** - Time synchronization utilities
  - Provides time synchronization functions for acoustic modems
  - Handles timing for scheduled transmissions

## Local Development Tools (`local/`)

### Testing Scripts (`local/scripts/`)

- **`network_test.py`** - Network connectivity testing for acoustic systems
- **`owtt.py`** - One-Way Travel Time measurement utilities
- **`plotter.py`** - Data visualization tools for acoustic measurements
- **`screamfest.py`** - Stress testing tool for acoustic communication
- **`sync_test.py`** - Time synchronization testing

### Hardware Documentation (`local/`)

- **`commands.txt`** - Complete reference of acoustic modem commands
- **`readme.md`** - Additional setup and configuration notes
- **`USB pinout/`** - USB to serial conversion diagrams and pinouts

## Firmware and Documentation

### Firmware (`NM3Firmware/`)

- **`nm3-acoustic-modem-serial-commands-20240215.pdf`** - Complete command reference
- **`nm3bootloader-20240215.exe`** - Firmware update bootloader
- **`NM3FlashingApplicationFirmware-20231018.pdf`** - Firmware update guide
- **`Readme.md`** - Firmware update instructions

### User Manuals (`User Manual/`)

- **`delphis-data-sheet_v9_3.2.pdf`** - Hardware specifications and capabilities
- **`delphis_cable_pin_out.pdf`** - Cable wiring and connection diagrams
- **`succorfish_delphis_user_guide_v4.pdf`** - Complete user manual and operation guide

## Data Storage (`data/`)

- Directory for storing experimental data, logs, and measurement results
- Includes placeholder files to maintain directory structure

## Quick Start for Standalone Testing

1. **Hardware Setup**:
   ```bash
   # Check available serial ports
   ls /dev/ttyUSB*
   
   # Test direct communication
   sudo screen /dev/ttyUSB0 9600
   ```

2. **Run Ping Test**:
   ```bash
   # Terminal 1 (pinger)
   python3 scripts/pinger.py
   
   # Terminal 2 (pingee) 
   python3 scripts/pingee.py
   ```

3. **Basic Commands**:
   - Ping: `$P001`
   - Broadcast: `$B13Hello, World!`
   - Clear timer: `$XTE`

## Integration with ROS 2

These utilities can be used alongside the ROS 2 nodes for:
- System debugging and validation
- Performance benchmarking
- Standalone testing without ROS overhead
- Hardware validation before deployment

## Configuration Notes

- Default baud rate: 9600
- Default sound velocity: 1500 m/s (seawater)
- Typical range: 100-1000m underwater, ~10m in air
- Communication timeout: 0.5-2.0 seconds depending on range

For detailed usage instructions, refer to the main README.md and individual script documentation.
