#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 

import math
from a_star_hoang_fazil_scaled import PathNode
from a_star_hoang_fazil_scaled import a_star, backtrack, create_map, SCALE_FACTOR


# Function to convert quaternion to euler coordinate
def euler_from_quaternion(x, y, z, w):
       """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
       """
       t0 = +2.0 * (w * x + y * z)
       t1 = +1.0 - 2.0 * (x * x + y * y)
       roll_x = math.atan2(t0, t1)
     
       t2 = +2.0 * (w * y - z * x)
       t2 = +1.0 if t2 > +1.0 else t2
       t2 = -1.0 if t2 < -1.0 else t2
       pitch_y = math.asin(t2)
     
       t3 = +2.0 * (w * z + x * y)
       t4 = +1.0 - 2.0 * (y * y + z * z)
       yaw_z = math.atan2(t3, t4)
     
       return roll_x, pitch_y, yaw_z 

# Define a custom QoS profile
custom_qos_profile = QoSProfile(
    depth=10,  # Depth of the subscription queue
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Best effort reliability
    history=HistoryPolicy.KEEP_LAST  # Keep last message
)
    
class TurtlebotNavigator(Node):
    def __init__(self):
        super().__init__('turtlebot_navigator')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=custom_qos_profile)
        self.optimal_path = []
        self.current_waypoint = 0
        self.tolerance = 0.1  # Tolerance for reaching a waypoint
     
        self.rpm1 = 800.0
        self.rpm2 = 1200.0
        self.clearance = 50.0/SCALE_FACTOR
        
        # Updated to create Node instances with float values
        self.start_node = PathNode((500 / float(SCALE_FACTOR), 1000 / float(SCALE_FACTOR), 0))
        self.goal_node = PathNode((5750 / float(SCALE_FACTOR), 1000 / float(SCALE_FACTOR), 0))

        # self.get_user_input()
        self.run_a_star()

    def run_a_star(self):
        canvas, height, obstacle_matrix = create_map(clearance=self.clearance)

        dt = 0.1
        result, goal_node, _ = a_star(self.start_node, self.goal_node, self.rpm1, self.rpm2, dt, obstacle_matrix, height)

        if result == "Success":
            print("Path found!")
            self.optimal_path = backtrack(goal_node)

            print("Optimal Path (Scaled-down):")
            for node in self.optimal_path:
                print(f"({node.x}, {node.y}, {node.state[2]})")

            # # Reverse the scaling of the optimal path
            # self.optimal_path = [(node.x * SCALE_FACTOR, node.y * SCALE_FACTOR, node.state[2]) for node in self.optimal_path]
            
            # Convert the optimal path from millimeters to meters
            self.optimal_path = [(node.x * SCALE_FACTOR / 1000, node.y * SCALE_FACTOR / 1000, node.state[2]) for node in self.optimal_path]
            
            # Apply offset to the optimal path
            offset_x = 0.5
            offset_y = 1.0
            self.optimal_path = [(x - offset_x, y - offset_y, theta) for x, y, theta in self.optimal_path]

            print("Optimal Path (Unscaled and Offset):")
            for x, y, theta in self.optimal_path:
                print(f"({x}, {y}, {theta})")
        else:
            print("No path found.")
            rclpy.shutdown()

    def odom_callback(self, msg):
        if not self.optimal_path:
            # If there's no path, there's nothing to follow.
            return

        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation

        if self.current_waypoint < len(self.optimal_path):
            target_waypoint = self.optimal_path[self.current_waypoint]
            # Extract x, y coordinates and orientation from the target waypoint
            target_position = (target_waypoint[0], target_waypoint[1])
            target_orientation = target_waypoint[2]

            # Calculate the Euclidean distance to the target waypoint
            distance = math.sqrt((target_position[0] - current_position.x) ** 2 +
                                (target_position[1] - current_position.y) ** 2)

            if distance < self.tolerance:
                self.get_logger().info(f"Waypoint {self.current_waypoint} reached. Moving to next waypoint.")
                self.current_waypoint += 1  # Move to the next waypoint
                if self.current_waypoint == len(self.optimal_path):
                    # If this was the last waypoint, stop the robot.
                    self.get_logger().info("Final waypoint reached. Stopping robot.")
                    self.stop_robot()
            else:
                # Navigate to the current target waypoint
                self.navigate_to_waypoint(current_position, current_orientation, target_position, target_orientation)
        else:
            # Explicitly handle the case when all waypoints have been processed
            self.get_logger().info("All waypoints processed. Stopping robot.")
            self.stop_robot()

    def navigate_to_waypoint(self, current_position, current_orientation, target_position, target_orientation):
        cmd_vel = Twist()

        # Calculate the heading error
        current_euler = euler_from_quaternion(current_orientation.x, current_orientation.y,
                                            current_orientation.z, current_orientation.w)
        current_heading = current_euler[2]
        target_heading = math.atan2(target_position[1] - current_position.y, target_position[0] - current_position.x)
        heading_error = target_heading - current_heading

        # Normalize the heading error to be within [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Log current position and heading error
        # self.get_logger().info(f"Current Position: X: {current_position.x}, Y: {current_position.y}")
        # self.get_logger().info(f"Heading Error: {math.degrees(heading_error)} degrees")

        # Define a threshold for "sufficient alignment" before moving forward
        alignment_threshold = math.radians(5)  # Adjusted to 2 degrees for finer control

        if abs(heading_error) > alignment_threshold:
            # Turn in place
            cmd_vel.angular.z = 2.0 * heading_error
            cmd_vel.linear.x = 0.1  # Slow down while turning
        else:
            # Move forward once aligned
            cmd_vel.angular.z = 0.0  # Stop turning
            cmd_vel.linear.x = 0.3  # Move forward

        self.cmd_vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    navigator = TurtlebotNavigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()