#!/usr/bin/env python3
import rclpy,time,math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        time.sleep(1)  # Ensure connection is stable

        # LiDAR setup parameters
        self.max_range = 4.0  # Maximum range of the LiDAR
        self.angle_min = -math.pi * 135 / 180  # Start angle
        self.angle_max = math.pi * 135 / 180   # End angle
        self.angle_increment = math.pi / 180  # Resolution: 1 degree per measurement

    def timer_callback(self):
        # Fetch data from CoppeliaSim
        lidar_data = self.sim.readCustomTableData(self.sim.handle_scene, "lidarData")
        
        if lidar_data:
            # Prepare LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'lidar_frame'
            scan_msg.header.stamp = self.get_clock().now().to_msg()

            # Set up angle and range values
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.range_min = 0.1  # Minimum range
            scan_msg.range_max = self.max_range

            # Populate ranges array from lidar_data
            ranges = []
            for i in range(0, len(lidar_data), 2):
                distance = lidar_data[i]
                angle = lidar_data[i + 1]
                
                # Fill ranges with distance values mapped to the angle positions
                index = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= index < int((self.angle_max - self.angle_min) / self.angle_increment):
                    ranges.append(distance)

            # Fill in the message
            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)
            self.get_logger().info("Published LaserScan data to RViz.")
        else:
            self.get_logger().info('No data retrieved from CoppeliaSim.')



def main(args=None):
	rclpy.init(args=args)
	nodeh = LidarPublisher()

	
	try: rclpy.spin(nodeh)
	except Exception as error: print(error)
	except KeyboardInterrupt: print("aios ctrl c")
	
if __name__ == "__main__":
	main()

