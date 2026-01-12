import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo, LaserScan
from nav_msgs.msg import Odometry      
from std_srvs.srv import SetBool       
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from packaging.version import parse

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

if parse(cv2.__version__) >= parse('4.7.0'):
    def local_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
        marker = np.array([[-marker_size /2, marker_size / 2, 0],
                           [marker_size /2, marker_size / 2, 0],
                           [marker_size /2, -marker_size / 2, 0],
                           [-marker_size /2, -marker_size / 2, 0]],
                           dtype = np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

class ArucoTarget(Node):
    _DICTS = {
        "apriltag_36h10" : cv2.aruco.DICT_APRILTAG_36H10,
        "4x4_100" : cv2.aruco.DICT_4X4_100,
    }

    def __init__(self, tag_set="apriltag_36h10", target_width=0.30):
        super().__init__('aruco_targetv2')
        self.get_logger().info(f'{self.get_name()} created')

        self._active = False 
        
        # --- CONFIGURATION ---
        self.start_point = [-4, 0, 0] # Phase 1 target
        self.origin_point = [0, 0, 0] # Phase 3 target (Return Home)
        
        self.target_list = [0, 1, 2, 3, 4]  # Phase 2 targets
        self.target_index = 0
        
        # STATE MACHINE FLAGS
        self.reached_start = False
        self.target_close_counter = 0
        self.required_frames_close = 15
        self.last_log_time = time.time()
        self.mission_complete = False
        
        # Position tracking
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0

        # ROS Setup
        self.declare_parameter('image', "/mycamera/image_raw")
        self.declare_parameter('info', "/mycamera/camera_info")
        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._info_topic = self.get_parameter('info').get_parameter_value().string_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self.create_subscription(CameraInfo, self._info_topic, self._info_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_service(SetBool, '/startup', self._startup_callback)

        self.obstacle_detected = False
        self.avoid_bias = 0.0
        self.stop_completely = False

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._bridge = CvBridge()

        # ArUco Setup
        dict_name = ArucoTarget._DICTS.get(tag_set.lower(), cv2.aruco.DICT_APRILTAG_36H10)
        if parse(cv2.__version__) < parse('4.7.0'):
            self._aruco_dict = cv2.aruco.Dictionary_get(dict_name)
            self._aruco_param = cv2.aruco.DetectorParameters_create()
        else:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_name)
            self._aruco_param = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_param)
        self._target_width = target_width
        self._image = None
        self._cameraMatrix = None
        self._distortion = None

    def _startup_callback(self, request, resp):
        if request.data:
            self.get_logger().info("Startup received. Robot Active.")
            self._active = True
            resp.success = True
            resp.message = "Robot Active"
        else:
            self._active = False
            resp.success = True
            resp.message = "Robot Stopped"
        return resp

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        _, _, yaw = euler_from_quaternion(pose.orientation)
        self._cur_theta = yaw

    def _short_angle(self, angle):
        if angle > math.pi: angle -= 2 * math.pi
        if angle < -math.pi: angle += 2 * math.pi
        return angle

    def _compute_speed(self, diff, max_speed, min_speed, gain):
        speed = abs(diff) * gain
        speed = min(max_speed, max(min_speed, speed))
        return math.copysign(speed, diff)

    def _drive_to_point(self, target):
        """ Returns True if reached, Twist command if driving """
        goal_x, goal_y, goal_theta = target[0], target[1], target[2]
        
        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff**2 + y_diff**2)
        
        twist = Twist()
        
        if dist < 0.15:
            # Reached position
            return True, twist # Stop

        # Navigate
        heading = math.atan2(y_diff, x_diff)
        diff = self._short_angle(heading - self._cur_theta)
        
        if abs(diff) > 0.2:
            twist.angular.z = self._compute_speed(diff, 0.5, 0.2, 2.0)
        else:
            twist.linear.x = self._compute_speed(dist, 0.3, 0.05, 0.5)
            twist.angular.z = diff * 0.5
            
        return False, twist

    def _scan_callback(self, msg):
        ranges = msg.ranges
        valid_entries = [(r, i) for i, r in enumerate(ranges) if np.isfinite(r) and r > 0.03]
        if len(valid_entries) > 0:
            closest_dist, closest_index = min(valid_entries, key=lambda x: x[0])
            total_indices = len(ranges)
            mid_point = total_indices / 2
            center_width = total_indices * 0.3
            center_start = mid_point - (center_width / 2)
            center_end = mid_point + (center_width / 2)
            
            if closest_dist < 0.5:
                self.obstacle_detected = True
                if center_start < closest_index < center_end:
                    self.avoid_bias = 0.0
                    self.stop_completely = True
                    self.get_logger().warn("OBSTACLE DETECTED: PLEASE MOVE ASIDE")
                elif closest_index < mid_point:
                    self.avoid_bias = -0.3
                    self.stop_completely = False
                else:
                    self.avoid_bias = 0.3
                    self.stop_completely = False
            else:
                self.obstacle_detected = False
                self.stop_completely = False
                self.avoid_bias = 0.0
        else:
            self.obstacle_detected = False
            self.stop_completely = False
            self.avoid_bias = 0.0

    def _info_callback(self, msg):
        self._distortion = np.reshape(msg.d, (1,5))
        self._cameraMatrix = np.reshape(msg.k, (3,3))

    def _image_callback(self, msg):
        if not self._active:
            return 
        try:
            self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        twist = Twist()

        # EMERGENCY STOP
        if self.obstacle_detected and self.stop_completely:
            self._cmd_pub.publish(Twist())
            if self._image is not None:
                cv2.imshow('window', self._image)
                cv2.waitKey(3)
            return

        # --- LOGGING ---
        if time.time() - self.last_log_time > 2.0:
            if not self.reached_start:
                 self.get_logger().info(f"Phase 1: Moving to Start (-4,0)...")
            elif self.target_index < len(self.target_list):
                 self.get_logger().info(f"Phase 2: Searching/Chasing ID {self.target_list[self.target_index]}")
            elif not self.mission_complete:
                 self.get_logger().info("Phase 3: All Targets Found. Returning to Origin (0,0)...")
            else:
                 self.get_logger().info("Mission Complete. Resting at Origin.")
            self.last_log_time = time.time()

        # ARUCO DETECTION
        grey = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
        if parse(cv2.__version__) < parse('4.7.0'):
            corners, ids, rejected = cv2.aruco.detectMarkers(grey, self._aruco_dict)
        else:
            corners, ids, rejected = self._aruco_detector.detectMarkers(grey)
        
        frame = cv2.aruco.drawDetectedMarkers(self._image, corners, ids)

        # === CONTROL LOGIC ===
        
        # 1. GO TO START POINT FIRST
        if not self.reached_start:
            reached, nav_twist = self._drive_to_point(self.start_point)
            if reached:
                self.get_logger().info("Start Position Reached. Beginning Search.")
                self.reached_start = True
                twist = Twist() # Stop
            else:
                twist = nav_twist
        
        # 2. SEARCH AND CHASE LOGIC
        else:
            if self.target_index >= len(self.target_list):
                # 3. RETURN TO ORIGIN (PHASE 3)
                reached_home, home_twist = self._drive_to_point(self.origin_point)
                
                if reached_home:
                    if not self.mission_complete:
                        self.get_logger().info("Returned to Origin (0,0). FULL MISSION COMPLETE.")
                        self.mission_complete = True
                    twist = Twist() # Stop
                else:
                    twist = home_twist
            else:
                # SEARCHING FOR TARGETS
                current_id = self.target_list[self.target_index]
                target_found = False
                dist_to_target = 0.0
                target_angle = 0.0

                if ids is not None:
                    ids_list = ids.flatten().tolist()
                    if current_id in ids_list:
                        target_found = True
                        idx = ids_list.index(current_id)
                        
                        if parse(cv2.__version__) < parse('4.7.0'):
                            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self._target_width, self._cameraMatrix, self._distortion)
                        else:
                            rvec, tvec, _ = local_estimatePoseSingleMarkers(corners, self._target_width, self._cameraMatrix, self._distortion)
                        
                        t = tvec[idx].flatten()
                        dist_to_target = math.sqrt(t[0]**2 + t[1]**2 + t[2]**2)
                        target_angle = math.atan2(t[0], t[2])

                        for r, t_vec in zip(rvec, tvec):
                            if parse(cv2.__version__) < parse('4.7.0'):
                                cv2.aruco.drawAxis(frame, self._cameraMatrix, self._distortion, r, t_vec, self._target_width)
                            else:
                                cv2.drawFrameAxes(frame, self._cameraMatrix, self._distortion, r, t_vec, self._target_width)

                if target_found:
                    # CHECK DISTANCE
                    if dist_to_target < 1.0:
                        self.target_close_counter += 1
                        if self.target_close_counter > self.required_frames_close:
                            self.get_logger().info(f"Target {current_id} Reached (<1m). Switching to next.")
                            self.target_index += 1
                            self.target_close_counter = 0
                            twist = Twist() # Stop briefly
                        else:
                            # Keep chasing to be sure
                            steer = -1.0 * (target_angle + self.avoid_bias)
                            twist.angular.z = steer
                            twist.linear.x = 0.15
                    else:
                        # FOUND BUT FAR -> CHASE
                        self.target_close_counter = 0
                        steer = -1.2 * (target_angle + self.avoid_bias)
                        twist.angular.z = steer
                        
                        if abs(target_angle) < 0.5 and not self.obstacle_detected:
                            twist.linear.x = 0.25
                        else:
                            twist.linear.x = 0.05 
                else:
                    # NOT FOUND -> SPIN SEARCH
                    self.target_close_counter = 0
                    twist.angular.z = 0.4 # Constant spin
                    twist.linear.x = 0.0

        if self._image is not None:
            cv2.imshow('window', frame)
            cv2.waitKey(3)
        self._cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    # Ensure target_width is correct for simulation
    node = ArucoTarget(tag_set="apriltag_36h10", target_width=0.20)
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
                
  



