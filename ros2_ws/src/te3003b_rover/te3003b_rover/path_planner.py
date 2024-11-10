#!/usr/bin/env python3
import rclpy, math, time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid
from . import d_star_lite

"""
what this node should do:
- set new position --> later change this to a setpoint for the position controller
- check pos (right now does nothing, later change to check if pos setpoint has been reached)
- update map (convert from occ grid to something the d* lite impl format)
"""

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds

        # Set up the TransformBroadcaster for TF messages
        self.tf_broadcaster = TransformBroadcaster(self)

        # Movement parameters
        self.position_x = 0.0
        self.position_y = 0.0
        self.broadcast_transforms()
        
        # Initialize variable to hold map data
        self.alg = None
        self.map = None
        self.first_run = True
        self.done = False

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
    
    def timer_callback(self):
        # static map!! (for now)
        if self.alg is None and self.map is not None:
            self.broadcast_transforms()
            self.alg = d_star_lite.StateGraph(self.map, start=[10,10], goal=[190,140])
            self.scan_range = 50
            self.s_start = self.alg.start_state_id
            self.path_to_start = []
            self.s_last = self.s_start
            self.path_possible = True
            self.alg.Initialize(self.s_start, self.first_run, False)
            self.alg.ComputeShortestPath(self.s_start)
            
        if self.alg is not None and self.path_possible and not self.done:
            if self.s_start != self.alg.goal_state_id:

                self.s_start, self.s_last = self.alg.RunProcedure(self.s_start, self.s_last, self.scan_range)

                if self.s_start is None or self.s_last is None:
                    self.get_logger().info("REINITIALIZING!")
                    self.alg = None
                    self.first_run = False
                else:
                    self.position_x, self.position_y = self.alg.StateIDToCoods(self.s_start)
                    self.path_to_start.append([self.position_x, self.position_y])

            else:
                self.get_logger().info("GOAL REACHED!")
                self.done = True
            self.get_logger().info(f"Moving to target position: ({self.position_x}, {self.position_y})")
        self.broadcast_transforms()

    def map_callback(self, msg):
        map_np_arr = np.array(msg.data, dtype=np.uint8)
        # for some reason, the map is read inverted in color and flipped vertically
        # we also need to convert the ranges from 0-100 to 0-255
        self.map = (-(map_np_arr.reshape((msg.info.height, msg.info.width)) // 100) + 1) * 255


    def broadcast_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.position_x*0.05) # scaled to match rviz coordinates
        t.transform.translation.y = float(self.position_y*0.05)
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0.0, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Map to reference: ({self.position_x}, {self.position_y})")

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
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()


