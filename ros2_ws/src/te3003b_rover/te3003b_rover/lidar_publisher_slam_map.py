#!/usr/bin/env python3
import rclpy, math, time
import numpy as np
import cv2 as cv
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist, Pose, Point
from geometry_msgs.msg import Vector3
import rclpy.timer
import rclpy.wait_for_message
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from . import d_star_lite
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        self.declare_parameter('control', 'keyboard')
        self.declare_parameter('planner_map_size', 300)
        self.control = self.get_parameter('control').get_parameter_value().string_value
        side_size = self.get_parameter('planner_map_size').get_parameter_value().integer_value
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.map_pos_publisher_ = self.create_publisher(Vector3, 'planner_pos', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.pose_publisher_ = self.create_publisher(Pose, 'rover_pose', 10)
        self.timer_rate = 0.5
        self.timer = self.create_timer(self.timer_rate, self.timer_callback)  # Publish every 0.1 seconds
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Path planner variables
        self.alg = None
        self.first_run = True
        self.done = False
        self.goal = None

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)

        # Get LiDAR object handle from CoppeliaSim
        self.lidar_handle = self.sim.getObjectHandle('/SickTIM310')  # Replace with your actual object name in CoppeliaSim

        # Movement parameters
        self.copp_pos_x, self.copp_pos_y, _ = self.sim.getObjectPosition(self.lidar_handle)
        _, _, self.copp_yaw = self.sim.getObjectOrientation(self.lidar_handle)
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.broadcast_transforms()

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180
        
        # Subscribe to the /map topic
        self.create_subscription(Vector3, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Twist, '/key_vel', self.key_callback, 10)

        # Initialize variable to hold map data
        self.map_data = None
        self.map_res = 0.05
        self.scan_range = 5
        self.key_input_vel = Twist(linear=self.list_to_vector3(), angular=self.list_to_vector3())
        self.zero_cov_36 = [0.0]*36

        self.start_time = self.get_clock().now()

        self.planner_twist = Twist(linear=self.list_to_vector3(), angular=self.list_to_vector3())
        #self.map_updates = 1
        self.map_size = [side_size, side_size]
        self.nav_start_x = self.map_size[1] // 2
        self.nav_start_y = self.map_size[1] // 2
        self.goal_start_x = self.map_size[1] // 2
        self.goal_start_y = self.map_size[1] // 2
        self.map_pos_x = self.map_size[1] // 2
        self.map_pos_y = self.map_size[1] // 2
        self.navmap = np.zeros(tuple(self.map_size))

    def init_planner(self):
        self.alg = d_star_lite.StateGraph(self.navmap, start=[self.goal_start_x, self.goal_start_y], goal=self.goal)
        self.s_start = self.alg.start_state_id
        self.path_to_start = [[self.goal_start_x, self.goal_start_y]]
        self.s_last = self.s_start
        self.alg.Initialize(self.s_start, self.first_run, False)
        self.alg.ComputeShortestPath(self.s_start)
    
    def timer_callback(self):
        match self.control:
            case 'path_planner':
                self.iterate_nav_alg()
            case 'avoid_obstacles':
                self.navigate_based_on_map()
            case _:
                self.navigate_based_on_keystrokes()

        # Broadcast transformation for ROS2 TFs
        self.broadcast_transforms()

        # Publish sim Odometry data
        self.publish_odom_data()

        # Send position and orientation to CoppeliaSim
        self.set_lidar_position_in_coppeliasim()

        # Publish LiDAR data as usual
        self.publish_lidar_data()

    def iterate_nav_alg(self):            
        if not self.map_ready() or self.done or self.goal is None:
            return
        if self.alg is None:
            self.init_planner()
        if self.s_start != self.alg.goal_state_id:

            self.s_start, self.s_last = self.alg.RunProcedure(self.s_start, self.s_last, self.scan_range)

            if self.s_start is None or self.s_last is None:
                self.get_logger().info("REINITIALIZING!")
                self.first_run = False
                self.goal_start_x = self.path_to_start[0][0] if len(self.path_to_start) < 5 else self.path_to_start[-5][0]
                self.goal_start_y = self.path_to_start[0][1] if len(self.path_to_start) < 5 else self.path_to_start[-5][1]
                self.walk_back()
                self.init_planner()
            else:
                next_point = self.alg.StateIDToCoods(self.s_start)
                self.map_pos_x = next_point[0]
                self.map_pos_y = next_point[1]
                self.position_x = (next_point[0] - self.nav_start_x)*self.map_res
                self.position_y = (next_point[1] - self.nav_start_y)*self.map_res # img/alg coords are different than map coords
                self.planner_twist.linear.x = (next_point[0] - self.path_to_start[-1][0]) * self.map_res * (1/self.timer_rate)
                self.planner_twist.linear.y = (next_point[1] - self.path_to_start[-1][1]) * self.map_res * (1/self.timer_rate)
                self.get_logger().info(f"RAN MAP STEP! ({next_point[0]},{next_point[1]}) aka ({self.position_x:.2f},{self.position_y:.2f})")
                self.path_to_start.append(next_point)
                self.map_pos_publisher_.publish(Vector3(x=float(self.map_pos_x), y=float(self.map_pos_y), z=0.0))

        else:
            self.get_logger().info("GOAL REACHED!")
            self.map_pos_x, self.map_pos_x = self.alg.StateIDToCoods(self.s_start)
            self.map_pos_publisher_.publish(Vector3(x=float(self.map_pos_x), y=float(self.map_pos_y), z=0.0))
            self.done = True

    def navigate_based_on_map(self):
        # Parameters for navigation behavior
        movement_step = 0.005  # Step size for movement in meters
        rotation_step = 0.005  # Step size for rotation in radians

        # Define a target point slightly ahead of the current position
        target_x = self.position_x + movement_step * math.cos(self.yaw)
        target_y = self.position_y + movement_step * math.sin(self.yaw)

        # Check if there’s an obstacle in the target position based on the map
        obstacle_detected = self.check_obstacle_in_map(target_x, target_y)

        if obstacle_detected:
            # If an obstacle is detected, decide to turn left
            self.yaw += rotation_step  # Turn left as an example
            self.get_logger().info("Obstacle detected; turning left.")
        else:
            # If no obstacle, move forward
            self.position_x = target_x
            self.position_y = target_y
            self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y})")

    def navigate_based_on_keystrokes(self):
        self.position_x += (self.key_input_vel.linear.x * math.cos(self.yaw) \
                                    + self.key_input_vel.linear.y * -math.sin(self.yaw))
        self.position_y += (self.key_input_vel.linear.x * math.sin(self.yaw) \
                                    + self.key_input_vel.linear.y * math.cos(self.yaw))
        self.yaw += self.key_input_vel.angular.z
        #self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y}, {self.yaw})")

    def check_obstacle_in_map(self, target_x, target_y):
        # Check if the map data is available
        if self.map_data is None:
            return False  # No map data available

        # Get the map's metadata
        resolution = self.map_data.info.resolution  # Size of each cell in meters
        origin_x = self.map_data.info.origin.position.x  # X position of the map's origin
        origin_y = self.map_data.info.origin.position.y  # Y position of the map's origin

        # Convert target position to map coordinates
        map_x = int((target_x - origin_x) / resolution)
        map_y = int((target_y - origin_y) / resolution)

        # Check if the indices are within the bounds of the map
        if 0 <= map_x < self.map_data.info.width and 0 <= map_y < self.map_data.info.height:
            index = map_y * self.map_data.info.width + map_x  # Convert 2D coordinates to 1D index
            occupancy_value = self.map_data.data[index]  # Get the occupancy value

            # Assuming a value of 100 means occupied, 0 means free
            if occupancy_value > 12:  # Threshold for obstacle detection
                return True  # Obstacle detected

        return False  # No obstacle detected

    def map_callback(self, msg):
        # Store the latest map data
        self.map_received = True
        self.map_data = msg
        self.map_res = msg.info.resolution
        #self.get_logger().info(f"GOT {msg.info.height}x{msg.info.width} MAP")
        #pos = msg.info.origin.position
        #self.get_logger().info(f"MAP ORIGIN: ({pos.x}, {pos.y})")
        origin_x_cell = int((msg.info.origin.position.x // self.map_res) + (self.map_size[1] // 2))
        origin_y_cell = int((msg.info.origin.position.y // self.map_res) + (self.map_size[0] // 2))
        if self.control == 'path_planner':
            map_np_arr = np.array(msg.data, dtype=np.uint8)
            # set unknown values as free
            map_np_arr[map_np_arr == -1] = 0
            # divide by 255 to normalize, change sign and add one for 1->0 and 0->1 (occupancy -> color)
            # set all cells with less than 23% occupancy (>0.77) to white (255)
            transf_map = (((-(map_np_arr.reshape((msg.info.height, msg.info.width)) / 255) + 1) > 0.77) * 255).astype(np.uint8)
            # erode to prevent clipping into walls
            transf_map = cv.erode(transf_map, cv.getStructuringElement(cv.MORPH_ELLIPSE, (3,3)))
            self.navmap[origin_y_cell : origin_y_cell + msg.info.height, origin_x_cell : origin_x_cell + msg.info.width] = transf_map
            #np.save(f"/home/thecubicjedi/lidar/map_captures/map_{self.map_updates}.npy", self.navmap)
            #self.map_updates += 1
            if self.alg is not None:
                self.alg.obstacles = self.navmap

    def key_callback(self, msg):
        key_vel : Twist = msg
        key_vel.angular.z *= 0.05
        key_vel.linear.x *= 0.05
        key_vel.linear.y *= 0.05
        self.key_input_vel = key_vel

    def goal_callback(self, msg):
        rec = [int(msg.x), int(msg.y)]
        self.get_logger().info(f"Got goal: {rec}")
        if self.goal is None:
            self.goal = rec
        elif self.goal != rec:
            self.goal = rec
            self.done = False
            self.goal_start_x = self.map_pos_x
            self.goal_start_y = self.map_pos_y
            self.init_planner()

    def broadcast_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        self.quaternion = self.euler_to_quaternion(0.0, 0.0, self.yaw)
        t.transform.rotation = self.quaternion

        self.tf_broadcaster.sendTransform(t)
        self.pose_publisher_.publish(Pose(position=Point(x=self.position_x, y=self.position_y, z=0.0), orientation=self.quaternion))
        self.get_logger().info(f"Map to reference: ({self.position_x:.2f}, {self.position_y:.2f}), Yaw: {self.yaw:.2f}")


    def set_lidar_position_in_coppeliasim(self):
        pos_x = self.copp_pos_x + (self.position_x * math.cos(self.copp_yaw) \
                                    + self.position_y * -math.sin(self.copp_yaw))
        pos_y = self.copp_pos_y + (self.position_x * math.sin(self.copp_yaw) \
                                    + self.position_y * math.cos(self.copp_yaw))
        # Set position in CoppeliaSim
        self.sim.setObjectPosition(self.lidar_handle, -1, [pos_x, pos_y, 0.1])

        # Set orientation in CoppeliaSim
        self.sim.setObjectOrientation(self.lidar_handle, -1, [0, 0, self.copp_yaw+self.yaw])


    def publish_lidar_data(self):
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        if lidar_data:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'base_link'
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.range_min = 0.1
            scan_msg.range_max = self.max_range

            ranges = []

            st_angle = int(math.degrees(lidar_data[1]))
            end_angle = int(math.degrees(lidar_data[-1]))
            last_deg = st_angle

            # add data for missing angles at beginning of scan
            for i in range(int(math.degrees(self.angle_min)), st_angle):
                ranges.append(self.max_range + 10)

            for i in range(0, len(lidar_data), 2):
                distance = lidar_data[i]
                angle = lidar_data[i + 1]
                deg = int(math.degrees(angle))
                diff = deg - last_deg
                # only add in 1 deg increments, pad data when an angle is skipped
                for i in range(diff):
                    ranges.append(distance) 
                last_deg = deg

            # add data for missing angles at end of scan
            for i in range(end_angle, int(math.degrees(self.angle_max))+1):
                ranges.append(self.max_range + 10)

            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
            #self.get_logger().info("Published LaserScan data to RViz.")
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')

    def publish_odom_data(self):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion
        odom.pose.covariance = self.zero_cov_36
        if self.control == 'path_planner':
            odom.twist.twist = self.planner_twist
        else:
            odom.twist.twist = self.key_input_vel
        odom.twist.covariance = self.zero_cov_36
        self.odom_publisher_.publish(odom)
            
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return TransformStamped().transform.rotation.__class__(x=qx, y=qy, z=qz, w=qw)
    
    def list_to_vector3(self, l=[0.0, 0.0, 0.0]):
        if len(l) != 3:
            return None
        v = Vector3()
        v.x = l[0]
        v.y = l[1]
        v.z = l[2]
        return v

    def map_ready(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time) > rclpy.duration.Duration(seconds=8)
        return self.map_data is not None and elapsed
    
    def walk_back(self):
        last_pos = self.path_to_start.pop()
        if len(self.path_to_start) < 5:
            self.path_to_start += [last_pos]*(len(self.path_to_start) - 5)
        for pos in self.path_to_start[-1:-6:-1]:
            self.planner_twist.linear.x = (pos[0] - last_pos[0]) * self.map_res * 10
            self.planner_twist.linear.y = (pos[1] - last_pos[1]) * self.map_res * 10
            self.position_x = (pos[0] - self.nav_start_x)*self.map_res
            self.position_y = (pos[1] - self.nav_start_y)*self.map_res # img/alg coords are different than map coords
            self.broadcast_transforms()
            self.publish_odom_data()
            self.map_pos_publisher_.publish(Vector3(x=float(pos[0]), y=float(pos[1]), z=0.0))
            self.set_lidar_position_in_coppeliasim()
            self.publish_lidar_data()
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()

    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()

