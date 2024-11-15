#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        self.declare_parameter('control', 'keyboard')
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        #self.imu_publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)

        # Get LiDAR object handle from CoppeliaSim
        self.lidar_handle = self.sim.getObjectHandle('/SickTIM310')  # Replace with your actual object name in CoppeliaSim

        # Movement parameters
        self.position_x, self.position_y, _ = self.sim.getObjectPosition(self.lidar_handle)
        _, _, self.yaw = self.sim.getObjectOrientation(self.lidar_handle)

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180
        
        # Subscribe to the /map topic
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Twist, '/key_vel', self.key_callback, 10)
        # Initialize variable to hold map data
        self.map_data = None
        self.key_input_vel = Twist(linear=self.list_to_vector3(), angular=self.list_to_vector3())
        # self.last_key_input_vel = None
        # self.quaternion = None
        # self.zero_cov_9 = [0.0]*9
        self.zero_cov_36 = [0.0]*36
        
    
    def timer_callback(self):
        control = self.get_parameter('control').get_parameter_value().string_value

        match control:
            case 'path_planner':
                pass
            case 'avoid_obstacles':
                self.navigate_based_on_map()
            case _:
                self.navigate_based_on_keystrokes()

        # Broadcast transformation for ROS2 TFs
        self.broadcast_transforms()

        # Send position and orientation to CoppeliaSim
        self.set_lidar_position_in_coppeliasim()

        # Publish LiDAR data as usual
        self.publish_lidar_data()

        # Publish fake IMU data
        #self.publish_imu_data()
        self.publish_odom_data()

        
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
        self.map_data = msg

    def key_callback(self, msg):
        key_vel : Twist = msg
        key_vel.angular.z *= 0.05
        key_vel.linear.x *= 0.05
        key_vel.linear.y *= 0.05
        self.key_input_vel = key_vel

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
        #self.get_logger().info(f"Map to reference: ({self.position_x}, {self.position_y}), Yaw: {self.yaw}")


    def set_lidar_position_in_coppeliasim(self):
        # Set position in CoppeliaSim
        self.sim.setObjectPosition(self.lidar_handle, -1, [self.position_x, self.position_y, 0.1])

        # Set orientation in CoppeliaSim
        self.sim.setObjectOrientation(self.lidar_handle, -1, [0, 0, self.yaw])


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

    # def publish_imu_data(self):
    #     imu_msg = Imu()
    #     imu_msg.header.stamp = self.get_clock().now().to_msg()
    #     imu_msg.angular_velocity_covariance = self.zero_cov
    #     imu_msg.orientation_covariance = self.zero_cov
    #     imu_msg.linear_acceleration_covariance = self.zero_cov
    #     imu_msg.orientation = self.quaternion
    #     imu_msg.angular_velocity = self.list_to_vector3([0.0, 0.0, self.key_input_vel.angular.z*10])
    #     if self.last_key_input_vel is not None:
    #         imu_msg.linear_acceleration = self.list_to_vector3([(self.key_input_vel.linear.x - self.last_key_input_vel.linear.x)*10,\
    #                                                             (self.key_input_vel.linear.y - self.last_key_input_vel.linear.y)*10,
    #                                                             (self.key_input_vel.linear.z - self.last_key_input_vel.linear.z)*10])
    #     else:
    #         imu_msg.linear_acceleration = self.list_to_vector3([0.0, 0.0, 0.0])
    #     self.last_key_input_vel = self.key_input_vel
    #     self.imu_publisher_.publish(imu_msg)
    #     #self.get_logger().info(f'Publishing fake imu data: {imu_msg}')

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
