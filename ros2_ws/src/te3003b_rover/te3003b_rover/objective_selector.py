#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3

class ObjectiveSelector(Node):
    def __init__(self):
        super().__init__('objective_selector')

        self.declare_parameter('max_distance', 10000.0)
        self.max_dist = self.get_parameter('max_distance').get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds

        # Movement parameters
        self.position_x = 0.0
        self.position_y = 0.0
        self.distance_traveled = 0.0
        self.est_distance_remaining = 0.0

        # create custom message with position, rem distance and distance traveled so far
        self.create_subscription(Distance, '/distance', self.dist_callback, 10)
        self.publisher_ = self.create_publisher(Vector3, 'goal', 10)
        self.dist_est = 0
        self.goals = [[0,0],[0,0],[0,0]]
    
    def timer_callback(self):
        # choose goal from self.goal based on distance
        return
    
    def dist_callback(self, msg):
        self.position_x = msg.position.x
        self.position_y = msg.position.y
        self.distance_traveled = msg.distance_traveled
        self.est_distance_remaining = msg.est_distance_remaining


def main(args=None):
    rclpy.init(args=args)
    node = ObjectiveSelector()

    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Shutdown initiated.")

if __name__ == "__main__":
    main()


