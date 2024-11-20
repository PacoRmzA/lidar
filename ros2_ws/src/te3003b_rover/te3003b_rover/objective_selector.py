#!/usr/bin/env python3
import rclpy, math
from functools import cmp_to_key
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class ObjectiveSelector(Node):
    def __init__(self):
        super().__init__('objective_selector')

        self.declare_parameter('max_distance', 10000.0)
        self.declare_parameter('goals', [[0,0],[0,0],[0,0]])
        self.declare_parameter('planner_map_size', 300)
        self.dist_left = self.get_parameter('max_distance').get_parameter_value().double_value
        goal_arr = self.get_parameter('goals').get_parameter_value().integer_array_value
        self.goals = [[goal_arr[i],goal_arr[i+1]] for i in range(0,len(goal_arr),2)]
        self.distances = [0.0]*len(self.goals)
        self.distances_goal_base = [0.0]*len(self.goals)
        self.map_size = self.get_parameter('planner_map_size').get_parameter_value().integer_value

        self.map_res = 0.05
        self.route_factor = 1.5 # factor that accounts for the effect of obstacles in the distance

        # Movement parameters
        self.start_x = self.map_size // 2
        self.start_y = self.map_size // 2
        self.start = [self.start_x, self.start_y]
        self.map_pos_x = self.start_x
        self.map_pos_y = self.start_y
        self.last_x = self.start_x
        self.last_y = self.start_y
        self.distance_traveled = 0.0
        self.est_distance_remaining = 0.0
        self.dist_to_start = 0.0

        # create custom message with position, rem distance and distance traveled so far
        self.create_subscription(Vector3, '/planner_pos', self.pos_callback, 10)
        self.publisher_ = self.create_publisher(Vector3, 'goal', 10)

        # sort goals based on distance from current position
        self.update_goals()
        self.get_logger().info(f'INITIAL GOAL AT ({self.goals[0][0]},{self.goals[0][1]})')
        self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
    
    def pos_callback(self, msg):
        self.last_x = self.map_pos_x
        self.last_y = self.map_pos_y
        self.map_pos_x = msg.x
        self.map_pos_y = msg.y
        self.distance_traveled += self.distance([self.last_x, self.last_x], [msg.x, msg.y])
        self.distance_to_goal = self.distance([msg.x, msg.y], self.goals[0])

        if self.map_pos_x == self.goals[0][0] and self.map_pos_y == self.goals[0][1]: # goal reached
            self.goals.pop(0)
            self.update_goals()
            self.dist_left -= self.distance_traveled
            # have enough energy to go to the next goal and return to base
            if len(self.goals) > 0 and self.dist_left >= (self.distance[0] + self.distances_goal_base[0])*self.route_factor:
                self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
                self.get_logger().info(f'GOING TO NEW GOAL AT ({self.goals[0][0]},{self.goals[0][1]})')
            # back in base (whether all goals were reached or not)
            elif self.map_pos_x == self.start_x and self.map_pos_y == self.start_y:
                self.get_logger().info('BASE REACHED; MISSION COMPLETE')
            # not enough energy for next goal but enough to get back to base (or ideally all goals already done)
            elif self.dist_left >= (self.dist_to_start)*self.route_factor:
                self.goals = [self.start]
                self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
                self.get_logger().info(f'RETURNING TO BASE AT MAP COORDS ({self.start_x},{self.start_y})')
            # not enough energy to go anywhere
            else:
                self.get_logger().info('NOT ENOUGH ENERGY FOR NEXT GOAL OR RETURN TO BASE; SEND SUPPORT')

        else: # goal not yet reached
            # not enough energy to reach goal and return to base
            if self.dist_left < self.distance_traveled + (self.distance_to_goal + self.distances_goal_base[0])*self.route_factor:
                self.goals.pop(0)
                self.dist_left -= self.distance_traveled
                self.update_goals()
                # enough energy for new goal
                if self.dist_left >= (self.distances[0] + self.distances_goal_base[0])*self.route_factor:
                    self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
                    self.get_logger().info(f'SWITCHING TO GOAL AT ({self.goals[0][0]},{self.goals[0][1]})')
                # enough energy to go back to base
                elif self.dist_left >= self.dist_to_start*self.route_factor:
                    self.goals = [self.start]
                    self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
                    self.get_logger().info(f'RETURNING TO BASE AT MAP COORDS ({self.start_x},{self.start_y})')
                # not enough energy to go anywhere
                else:
                    # set current position as goal to prevent further movement
                    self.goals = [[msg.x, msg.y]]
                    self.publisher_.publish(Vector3(x=self.goals[0][0], y=self.goals[0][1], z=0.0))
                    self.get_logger().info('NOT ENOUGH ENERGY FOR NEXT GOAL OR RETURN TO BASE; SEND SUPPORT')

    # goals are sorted by energy required to reach them and then return to base
    def update_goals(self):
        self.goals = sorted(self.goals, key=cmp_to_key(self.compare_goals))
        pos = [self.map_pos_x, self.map_pos_y]
        self.distances = [self.distance(pos, goal) for goal in self.goals]
        self.distances_goal_base = [self.distance(self.start, goal) for goal in self.goals]
        self.dist_to_start = self.distance(pos, self.start)

    def compare_goals(self, a, b):
        pos = [self.map_pos_x, self.map_pos_y]
        return (self.distance(b, pos)+self.distance(b, self.start)) - (self.distance(a, pos)+self.distance(a, self.start))
    
    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    
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
