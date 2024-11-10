#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)

        # Get LiDAR object handle from CoppeliaSim
        self.lidar_handle = self.sim.getObjectHandle('/SickTIM310')  # Replace with your actual object name in CoppeliaSim

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180

        # Movement parameters
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        
        # Initialize variable to hold scan data
        self.scan_data = None
        # Subscribe to the /scan topic
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Flag for the initial scan
        self.initial_scan_done = False
        
    
    def timer_callback(self):
        # Check if initial scan has been completed
        if not self.initial_scan_done:
            self.initial_scan()  # Perform the initial scan
        else:
            # After initial scan, switch to navigation
            self.navigate_based_on_map()
            self.broadcast_transforms(self.position_x, self.position_y, self.yaw)
            self.set_lidar_position_in_coppeliasim(self.position_x, self.position_y, self.yaw)
            self.publish_lidar_data()
    
    
    def initial_scan(self):
    	# algo de que se quede 10 segundos, luego lo haga
        # Rotate in place to perform a 360-degree scan
        rotation_step = (90*math.pi)/180  # Adjust for smoother rotation if needed
        self.yaw += rotation_step
        
        # Broadcast transformation for ROS2 TFs
        self.broadcast_transforms(self.position_x, self.position_y, self.yaw)
        
        # Set LiDAR position in CoppeliaSim
        self.set_lidar_position_in_coppeliasim(self.position_x, self.position_y, self.yaw)
        
        # Publish LiDAR data to create the map
        self.publish_lidar_data()
        
    def navigate_based_on_map(self):
        # Parameters for navigation behavior
        movement_step = 0.01  # Step size for movement in meters
        rotation_step = 0.01  # Step size for rotation in radians
        obstacle_distance_threshold = 0.5  # Threshold for obstacle distance in meters

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

    def check_obstacle_in_map(self, target_x, target_y):
        # Check for obstacles based on scan data
        if self.scan_data is None:
            return False  # No scan data available

        # Calculate the angle to the target position
        target_angle = math.atan2(target_y - self.position_y, target_x - self.position_x)
        angle_to_target = target_angle - self.yaw  # Angle relative to the robot's orientation

        # Normalize angle to be within [-pi, pi]
        angle_to_target = (angle_to_target + math.pi) % (2 * math.pi) - math.pi

        # Find the closest angle in the scan data
        index = int((angle_to_target - self.angle_min) / self.angle_increment)

        if 0 <= index < len(self.scan_data.ranges):
            distance_to_obstacle = self.scan_data.ranges[index]
            if distance_to_obstacle < 1.0:  # Check if obstacle is within 1 meter
                return True  # Obstacle detected

        return False  # No obstacle detected

    def scan_callback(self, msg):
        # Store the latest scan data for later use
        self.scan_data = msg

    def broadcast_transforms(self, pos_x, pos_y, yaw):
        # Broadcast the transform from map to reference_point
        self.broadcast_map_to_reference_point(pos_x, pos_y, yaw)

        # Broadcast the transform from reference_point to base_link
        #self.broadcast_reference_point_to_base_link(pos_x, pos_y, yaw)

    def broadcast_map_to_reference_point(self, pos_x, pos_y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, yaw)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Map to reference: ({self.position_x}, {self.position_y}), Yaw: {self.yaw}")

    def broadcast_reference_point_to_base_link(self, pos_x, pos_y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'reference_point'
        t.transform.translation.x = pos_x  # Assuming base_link is at the origin of reference_point
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, yaw)  # Or adjust as needed
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Reference to base position: ({self.position_x}, {self.position_y}), Yaw: {self.yaw}")



    def set_lidar_position_in_coppeliasim(self, pos_x, pos_y, yaw):
        # Set position in CoppeliaSim
        self.sim.setObjectPosition(self.lidar_handle, -1, [pos_x, pos_y, 0.1])

        # Set orientation in CoppeliaSim
        self.sim.setObjectOrientation(self.lidar_handle, -1, [0, 0, yaw])

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
            for i in range(0, len(lidar_data), 2):
                ranges.append(lidar_data[i])
            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
            self.get_logger().info("Published LaserScan data to RViz.")
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')
            
    

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

def main(args=None):
    rclpy.init(args=args)
    nodeh = LidarPublisher()

    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()

