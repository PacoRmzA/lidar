#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster

class KeyboardNavigator(Node):
    def __init__(self):
        super().__init__('keyboard_navigator')
        # Publisher for LaserScan
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        
        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)

        # Movement parameters
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        
        # Subscribe to the /key_vel topic
        self.create_subscription(Twist, '/key_vel', self.key_callback, 10)
        
        # Initialize variable to hold map data
        self.key_input_vel = None
        
    
    def timer_callback(self):
        self.navigate_based_on_keystrokes()
        self.broadcast_transforms(self.position_x, self.position_y, self.yaw)


    def navigate_based_on_keystrokes(self):
        if self.key_input_vel is not None:
            self.position_x += 0.05 * (self.key_input_vel.linear.x * math.cos(self.yaw) \
                                      + self.key_input_vel.linear.y * -math.sin(self.yaw))
            self.position_y += 0.05 * (self.key_input_vel.linear.x * math.sin(self.yaw) \
                                      + self.key_input_vel.linear.y * math.cos(self.yaw))
            self.yaw += 0.05 * self.key_input_vel.angular.z

    def key_callback(self, msg):
        self.key_input_vel = msg


    def broadcast_transforms(self, pos_x, pos_y, yaw):
        # Broadcast the transform from map to reference_point
        self.broadcast_map_to_reference_point(pos_x, pos_y, yaw)


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
    node = KeyboardNavigator()

    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()

