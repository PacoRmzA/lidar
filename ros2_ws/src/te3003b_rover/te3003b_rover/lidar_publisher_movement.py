#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds

        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = time.time()
        self.prev_yaw = None
        
        # Motion parameters (update these for radius and velocity)
        self.radius = 0.2  # <-- Modify this value to change the radius of the circular motion (in meters)
        self.linear_velocity = 0.01  # <-- Modify this value to change the linear velocity (in meters per second)

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)

        # Get LiDAR object handle from CoppeliaSim
        self.lidar_handle = self.sim.getObjectHandle('/SickTIM310')  # Replace 'lidar' with your actual object name in CoppeliaSim

        # LiDAR setup parameters
        self.max_range = 4.0
        self.angle_min = -math.pi * 135 / 180
        self.angle_max = math.pi * 135 / 180
        self.angle_increment = math.pi / 180

    def timer_callback(self):
        # Circular motion parameters
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
         # Calculate angular velocity based on linear velocity and radius
        angular_velocity = self.linear_velocity / self.radius  # <-- Adjusted with radius and velocity

        # Calculate position in circular path based on angular velocity
        angle = angular_velocity * current_time  # Angular position

        # Calculate position in circular path
        angle = angular_velocity * current_time  # Angular position
        pos_x = self.radius * math.sin(angle)
        pos_y = self.radius * math.cos(angle)
        yaw = math.atan2(pos_y, pos_x)

        if self.prev_yaw is not None:
            yaw = self.unwrap_angle(self.prev_yaw, yaw)
        self.prev_yaw = yaw

        # Broadcast transformation for ROS2 TFs
        self.broadcast_base_link_transform(pos_x, pos_y, yaw)

        # Send position and orientation to CoppeliaSim
        self.set_lidar_position_in_coppeliasim(pos_x, pos_y, yaw)

        # Publish LiDAR data as usual
        self.publish_lidar_data()

    def broadcast_base_link_transform(self, pos_x, pos_y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'reference_point'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, yaw)
        self.tf_broadcaster.sendTransform(t)

    def set_lidar_position_in_coppeliasim(self, pos_x, pos_y, yaw):
        # Set position in CoppeliaSim
        self.sim.setObjectPosition(self.lidar_handle, -1, [pos_x, pos_y, 0.1])

        # Set orientation in CoppeliaSim
        self.sim.setObjectOrientation(self.lidar_handle, -1, [0, 0, yaw])

    def publish_lidar_data(self):
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        if lidar_data:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'reference_point'
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.range_min = 0.1
            scan_msg.range_max = self.max_range

            ranges = []
            for i in range(0, len(lidar_data), 2):
                distance = lidar_data[i]
                angle = lidar_data[i + 1]
                index = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= index < int((self.angle_max - self.angle_min) / self.angle_increment):
                    ranges.append(distance)
            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
            self.get_logger().info("Published LaserScan data to RViz.")
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')

    def unwrap_angle(self, prev_yaw, current_yaw):
        delta = current_yaw - prev_yaw
        if delta > math.pi:
            current_yaw -= 2 * math.pi
        elif delta < -math.pi:
            current_yaw += 2 * math.pi
        return current_yaw

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

        return TransformStamped().transform.rotation.__class__(
            x=qx, y=qy, z=qz, w=qw
        )

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

