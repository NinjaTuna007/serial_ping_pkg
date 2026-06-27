import os
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
