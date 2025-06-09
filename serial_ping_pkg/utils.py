import os
import serial
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml_config(package_name, yaml_file):
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        yaml_file
    )
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def init_serial(port, port_fallback, baudrate, logger):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        if ser.is_open:
            logger.info(f"Serial port {port} opened at {baudrate} baud")
            return ser
    except serial.SerialException as e:
        logger.error(f"Failed to open serial port {port}: {e}")
        logger.info(f"Trying fallback port: {port_fallback}")
        try:
            ser = serial.Serial(port_fallback, baudrate, timeout=1)
            if ser.is_open:
                logger.info(f"Serial port {port_fallback} opened at {baudrate} baud")
                return ser
        except serial.SerialException as e:
            logger.error(f"Failed to open fallback serial port {port_fallback}: {e}")
            logger.error("No serial port available. Exiting node.")
            return None
    return None