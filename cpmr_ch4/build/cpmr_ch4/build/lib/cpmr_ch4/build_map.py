#!/usr/bin/env python3


# Populate the world with a json map file (cubes)
import sys
import os 
import json
import rclpy
from ament_index_python.packages import get_package_share_directory
from .add_obstacle import make_obstacle

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('build_map')

    # Default map file
    node.declare_parameter('map', 'default.json')
    map_name = node.get_parameter('map').get_parameter_value().string_value
    package_path = get_package_share_directory('cpmr_ch4')

    # Load the map file
    try:
        with open(os.path.join(package_path, 'maps', map_name)) as fd:
            map_data = json.load(fd)
    except Exception as e:
        node.get_logger().error(f"Unable to find/parse map in {package_path}/{map_name}")
        sys.exit(1)

    # Populate each cube in the world
    for o in map_data.keys():
        node.get_logger().info(f"Populating map with {map_data[o]}")
        # Pass x, y, and size for cube
        make_obstacle(node, o, map_data[o]['x'], map_data[o]['y'], map_data[o]['size'])

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


