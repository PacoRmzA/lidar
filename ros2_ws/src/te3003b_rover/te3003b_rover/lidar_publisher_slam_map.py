#!/usr/bin/env python3
import rclpy, math, time
import numpy as np
import cv2 as cv
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
from geometry_msgs.msg import Vector3
import rclpy.timer
import rclpy.wait_for_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32  
from . import d_star_lite
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        self.declare_parameter('planner_map_size', 300)
        side_size = self.get_parameter('planner_map_size').get_parameter_value().integer_value
        # Publisher for LaserScan
        self.lidar_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.map_pos_publisher_ = self.create_publisher(Vector3, 'planner_pos', 10)
        self.pose_publisher_ = self.create_publisher(Pose, 'rover_pose', 10)
        self.left_motor_pub = self.create_publisher(Float32, '/leftMotorSpeed', 10)
        self.right_motor_pub = self.create_publisher(Float32, '/rightMotorSpeed', 10)
        self.timer_rate = 0.1
        self.timer = self.create_timer(self.timer_rate, self.timer_callback)  # Publish every 0.1 seconds
        
        # Path planner variables
        self.alg = None
        self.first_run = True
        self.done = False
        self.goal = None

        self.controlled = False
        self.eps_lin = 0.1
        self.eps_ang = 15 * math.pi/180
        self.k_lin = 0.5
        self.k_ang = 0.1
        self.wheel_radius = 0.05
        self.wheel_base = 0.3

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

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180
        
        # Subscribe to the /map topic
        self.create_subscription(Vector3, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize variable to hold map data
        self.map_data = None
        self.map_res = 0.05
        self.scan_range = 5

        self.start_time = self.get_clock().now()

        #self.map_updates = 1
        self.map_size = [side_size, side_size]
        self.nav_start_x = self.map_size[1] // 2
        self.nav_start_y = self.map_size[1] // 2
        self.goal_start_x = self.map_size[1] // 2
        self.goal_start_y = self.map_size[1] // 2
        self.setpoint_position_x = 0.0
        self.setpoint_position_y = 0.0
        self.map_pos_x = self.map_size[1] // 2
        self.map_pos_y = self.map_size[1] // 2
        self.navmap = np.zeros(tuple(self.map_size))
        self.wb_i = 0
        self.walking_back = False

    def init_planner(self):
        self.alg = d_star_lite.StateGraph(self.navmap, start=[self.goal_start_x, self.goal_start_y], goal=self.goal)
        self.s_start = self.alg.start_state_id
        self.path_to_start = [[self.goal_start_x, self.goal_start_y]]
        self.s_last = self.s_start
        self.alg.Initialize(self.s_start, self.first_run, False)
        self.alg.ComputeShortestPath(self.s_start)
    
    def timer_callback(self):
        if self.controlled:
            self.publish_motor_speeds(0.0, 0.0)
            self.iterate_nav_alg()
        else:
            self.control_pos()

        self.publish_lidar_data()

    def iterate_nav_alg(self):            
        if not self.map_ready() or self.done or self.goal is None:
            return
        if self.walking_back:
            self.walk_back_it()
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
                self.walking_back = True
            else:
                next_point = self.alg.StateIDToCoods(self.s_start)
                self.map_pos_x = next_point[0]
                self.map_pos_y = next_point[1]
                self.setpoint_position_x = (next_point[0] - self.nav_start_x)*self.map_res
                self.setpoint_position_y = (next_point[1] - self.nav_start_y)*self.map_res # img/alg coords are different than map coords
                self.controlled = False
                self.get_logger().info(f"RAN MAP STEP! ({next_point[0]},{next_point[1]}) aka ({self.position_x:.2f},{self.position_y:.2f})")
                self.path_to_start.append(next_point)
                self.map_pos_publisher_.publish(Vector3(x=float(self.map_pos_x), y=float(self.map_pos_y), z=0.0))

        else:
            self.get_logger().info("GOAL REACHED!")
            self.map_pos_x, self.map_pos_x = self.alg.StateIDToCoods(self.s_start)
            self.map_pos_publisher_.publish(Vector3(x=float(self.map_pos_x), y=float(self.map_pos_y), z=0.0))
            self.done = True

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
        #self.get_logger().info(f"Got goal: {rec}")
        if self.goal is None:
            self.goal = rec
        elif self.goal != rec:
            self.goal = rec
            self.done = False
            self.goal_start_x = self.map_pos_x
            self.goal_start_y = self.map_pos_y
            self.init_planner()

    def odom_callback(self, msg):
        # Update position and orientation based on odometry data
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        q = msg.pose.pose.orientation
        self.pose_publisher_.publish(Pose(position=Point(x=self.position_x, y=self.position_y, z=0.0), orientation=q))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_pos(self):
        if int(time.time()*10) % 30 == 0:
            self.get_logger().info(f"Controlling to ({self.setpoint_position_x:.2f},{self.setpoint_position_y:.2f}) Cur pos ({self.position_x:.2f},{self.position_y:.2f})")
        error_xy = [self.setpoint_position_x - self.position_x, self.setpoint_position_y - self.position_y]
        if abs(error_xy[0]) < self.eps_lin and abs(error_xy[1]) < self.eps_lin:
            self.controlled = True
            self.publish_motor_speeds(0.0, 0.0)
            #self.get_logger().info(f"Successfully controlled pos to ({self.setpoint_position_x:.2f},{self.setpoint_position_y:.2f})")
        else:
            theta_d = math.atan2(error_xy[1], error_xy[0])
            error_th = theta_d - self.yaw
            om = self.k_ang*error_th
            if abs(error_th) > self.eps_ang:
                v_rot = (self.wheel_base*om*2)/(2*self.wheel_radius)
                self.publish_motor_speeds(v_rot, v_rot)
            else: 
                v = self.k_lin*math.sqrt(error_xy[0]**2 + error_xy[1]**2)
                v_l = v/self.wheel_radius - (self.wheel_base*om)/(2*self.wheel_radius)
                v_r = v/self.wheel_radius + (self.wheel_base*om)/(2*self.wheel_radius)
                self.publish_motor_speeds(-v_l, v_r)

    def publish_lidar_data(self):
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        if lidar_data:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'robot_base_respondable'
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
            self.lidar_publisher_.publish(scan_msg)
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')

    def publish_motor_speeds(self, left_speed, right_speed):
        # Publish speeds to the motor topics
        self.left_motor_pub.publish(Float32(data=left_speed))
        self.right_motor_pub.publish(Float32(data=right_speed))

    def map_ready(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time) > rclpy.duration.Duration(seconds=8)
        return self.map_data is not None and elapsed

    def walk_back_it(self):
        if self.wb_i == 0:
            last_pos = self.path_to_start.pop()
            if len(self.path_to_start) < 5:
                self.path_to_start += [last_pos]*(len(self.path_to_start) - 5)
        pos = self.path_to_start[-(self.wb_i + 1)] # [-1:-6:-1]
        self.setpoint_position_x = (pos[0] - self.nav_start_x)*self.map_res
        self.setpoint_position_y = (pos[1] - self.nav_start_y)*self.map_res # img/alg coords are different than map coords
        self.get_logger().info(f'STEPPING BACK TO ({self.setpoint_position_x:.2f},{self.setpoint_position_y:.2f})')
        self.controlled = False
        self.map_pos_publisher_.publish(Vector3(x=float(pos[0]), y=float(pos[1]), z=0.0))
        self.wb_i += 1
        if self.wb_i == 5:
            self.walking_back = False
            self.wb_i = 0
            self.init_planner()

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

