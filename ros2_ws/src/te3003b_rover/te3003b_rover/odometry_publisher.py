#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        # Get handles for motors, wheels, and base
        self.motor_left = self.sim.getObject('/leftMotor')
        self.motor_right = self.sim.getObject('/rightMotor')
        self.robot_base = self.sim.getObject('/robot_base_respondable')
        self.left_wheel = self.sim.getObject('/l_wheel')
        self.right_wheel = self.sim.getObject('/r_wheel')

        # ROS2 subscribers for motor velocity control
        self.left_motor_sub = self.create_subscription(Float32, 'leftMotorSpeed', self.set_left_motor_velocity, 10)
        self.right_motor_sub = self.create_subscription(Float32, 'rightMotorSpeed', self.set_right_motor_velocity, 10)
        
        # ROS2 publishers for position
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to periodically publish positions and TF
        self.timer = self.create_timer(0.1, self.publish_data)

        # Odometry variables
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()
        self.wheel_radius = 0.05  # Adjust to your robot's wheel radius
        self.wheel_base = 0.3    # Adjust to your robot's wheel base

        self.map_initial_position = self.sim.getObjectPosition(self.robot_base, -1)


    def set_left_motor_velocity(self, msg):
        if self.motor_left is not None:
            if msg.data > 0:
                vel = min(msg.data, 10.0)
            else:
                vel = max(msg.data, -10.0)
            self.sim.setJointTargetVelocity(self.motor_left, vel)
        else:
            self.get_logger().error("Left motor handle is invalid.")

    def set_right_motor_velocity(self, msg):
        if self.motor_right is not None:
            if msg.data > 0:
                vel = min(msg.data, 10.0)
            else:
                vel = max(msg.data, -10.0)
            self.sim.setJointTargetVelocity(self.motor_right, vel)
        else:
            self.get_logger().error("Right motor handle is invalid.")
    
    def calculate_and_publish_odometry(self):
        # Calculate odometry based on wheel velocities
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        left_vel = -self.sim.getJointTargetVelocity(self.motor_left) * self.wheel_radius
        right_vel = self.sim.getJointTargetVelocity(self.motor_right) * self.wheel_radius

        #self.get_logger().info(f"Motor vels: (L {left_vel:.2f}, R {right_vel:.2f})")

        linear_velocity = (right_vel + left_vel) / 2
        angular_velocity = (right_vel - left_vel) / self.wheel_base

        self.odom_theta += angular_velocity * dt
        self.odom_x += linear_velocity * math.cos(self.odom_theta) * dt
        self.odom_y += linear_velocity * math.sin(self.odom_theta) * dt

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'robot_base_respondable'

        # Pose
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.odom_theta / 2),
            w=math.cos(self.odom_theta / 2)
        )

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

    def publish_data(self):
        # Publish the robot's position and TF
        position = self.sim.getObjectPosition(self.robot_base, -1)
        orientation = self.sim.getObjectQuaternion(self.robot_base, -1)
        self.publish_tf(position, orientation, 'map', 'robot_base_respondable')
        self.publish_tf(position, orientation, 'map', 'odom')

        # Publish wheel transforms
        left_wheel_pos = self.sim.getObjectPosition(self.left_wheel, self.robot_base)
        left_wheel_ori = self.sim.getObjectQuaternion(self.left_wheel, self.robot_base)
        self.publish_tf(left_wheel_pos, left_wheel_ori, 'robot_base_respondable', 'l_wheel')

        right_wheel_pos = self.sim.getObjectPosition(self.right_wheel, self.robot_base)
        right_wheel_ori = self.sim.getObjectQuaternion(self.right_wheel, self.robot_base)
        self.publish_tf(right_wheel_pos, right_wheel_ori, 'robot_base_respondable', 'r_wheel')

        self.calculate_and_publish_odometry()

    def publish_tf(self, position, orientation, parent_frame, child_frame):
        # Broadcast a transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
    
        if parent_frame == 'map':
            # Offset the position to ensure the map's origin is the robot's initial position
            t.transform.translation.x = position[0] - self.map_initial_position[0]
            t.transform.translation.y = position[1] - self.map_initial_position[1]
            t.transform.translation.z = position[2] - self.map_initial_position[2]
        else:
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
    
        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = OdometryPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    image_publisher.c.close()
    rclpy.shutdown()

if __name__ == '__main__':
   main()