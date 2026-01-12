import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import cv2
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class CollectLidar(Node):
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.05

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((CollectLidar._HEIGHT, CollectLidar._WIDTH), dtype=np.uint8)
        
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_t = 0.0

        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)


    def _scan_callback(self, msg):
        # this is what sees the environment
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        self.get_logger().info(f"lidar ({angle_min},{msg.angle_max},{angle_increment},{len(ranges)})")

        # go through all lidar beams
        for i, r in enumerate(ranges):
            if r < msg.range_max:  # check valid distance
                angle = angle_min + i * angle_increment

                # Get the robot's position
                bot_x = self._cur_x
                bot_y = self._cur_y
                bot_t = self._cur_t

                # LIDAR point in local frame
                lidar_x = r * math.cos(angle)
                lidar_y = r * math.sin(angle)

                # Transform to world frame
                world_x = bot_x + lidar_x * math.cos(bot_t) - lidar_y * math.sin(bot_t)
                world_y = bot_y + lidar_x * math.sin(bot_t) + lidar_y * math.cos(bot_t)

                # Convert world coordinates to grid coordinates
                grid_x = int((world_x / CollectLidar._M_PER_PIXEL) + (CollectLidar._WIDTH // 2))
                grid_y = int((world_y / CollectLidar._M_PER_PIXEL) + (CollectLidar._HEIGHT // 2))

                # Mark this cell as occupied on the map if it's within bounds
                if 0 <= grid_x < CollectLidar._WIDTH and 0 <= grid_y < CollectLidar._HEIGHT:
                    self._map[grid_y, grid_x] = 255  # Mark as occupied

        cv2.imshow('map', self._map)
        cv2.waitKey(10)

        
    def _odom_callback(self, msg):
    #the robots position
        pose = msg.pose.pose

	#first we define it as partt of the lidar class
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        self._cur_t = yaw
        
        self.get_logger().info(f"at ({self._cur_x},{self._cur_y },{self._cur_t})")

def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

