#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from a_star import AStarPlanner  # Import the A* planner

class WallCenteringAndObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('wall_centering_obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.min_distance = 0.5  # Minimum distance to maintain from obstacles
        self.linear_speed = 0.2  # Base forward speed
        self.correction_gain = 0.5  # Gain for centering correction
        self.centering_threshold = 0.05  # Threshold to determine if centered between walls
        self.path = []
        self.current_goal_index = 0

        # Initialize the grid for A* (example grid, should be dynamically set based on environment)
        self.grid = [[0] * 10 for _ in range(10)]  # Placeholder grid
        self.astar = AStarPlanner(self.grid)
        self.start = (0, 0)
        self.goal = (9, 9)
        self.path = self.astar.plan(self.start, self.goal)

    def lidar_callback(self, msg):
        if not self.path:
            return

        ranges = np.array(msg.ranges)
        valid_ranges = np.where(np.isinf(ranges) | np.isnan(ranges), np.inf, ranges)

        # Define angle segments for front, back, left, and right parts
        front_angles = 10  # ±10 degrees for front obstacle detection
        back_angles = 10  # ±10 degrees for back wall detection
        left_angles = slice(90, 180)  # 90 to 180 degrees for left wall
        right_angles = slice(-180, -90)  # -180 to -90 degrees for right wall

        # Extract front ranges considering the small angle for obstacle detection
        front_ranges = np.concatenate((valid_ranges[:front_angles], valid_ranges[-front_angles:]))
        back_ranges = valid_ranges[180-back_angles:180+back_angles]  # Back detection within ±10 degrees around 180

        # Extract left and right wall distances
        left_ranges = valid_ranges[left_angles]
        right_ranges = valid_ranges[right_angles]

        # Compute distances by averaging valid (non-infinite) measurements
        left_distance = np.mean(left_ranges[left_ranges < np.inf])
        right_distance = np.mean(right_ranges[right_ranges < np.inf])
        front_distance = np.min(front_ranges)  # Use min to detect the closest obstacle in the front
        back_distance = np.min(back_ranges)  # Use min to detect the closest wall in the back

        # Check for valid distances
        if np.isnan(left_distance) or np.isnan(right_distance):
            self.get_logger().warn("Invalid distances: one of the distances is NaN")
            return

        # Calculate room dimensions
        room_length = front_distance + back_distance
        room_width = left_distance + right_distance

        # Log distances and room dimensions
        self.get_logger().info(f"Front distance: {front_distance:.2f} m, Back distance: {back_distance:.2f} m")
        self.get_logger().info(f"Left distance: {left_distance:.2f} m, Right distance: {right_distance:.2f} m")
        self.get_logger().info(f"Room length: {room_length:.2f} m, Room width: {room_width:.2f} m")

        twist_msg = Twist()

        if front_distance < self.min_distance:
            # Stop if an obstacle is too close
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info("Obstacle detected! Stopping.")
        else:
            if room_length > room_width:
                # Centering control when room length is greater than room width
                centering_error = left_distance - right_distance
                centering_correction = self.correction_gain * centering_error
                if abs(centering_error) > self.centering_threshold:
                    twist_msg.linear.x = self.linear_speed
                    twist_msg.angular.z = centering_correction
                    self.get_logger().info(f"Centering... Error: {centering_error:.2f} m, Correction: {centering_correction:.2f} rad/s")
                else:
                    twist_msg.linear.x = self.linear_speed
                    twist_msg.angular.z = 0.0
                    self.get_logger().info("Robot is centered between walls, moving forward.")
            else:
                twist_msg.linear.x = self.linear_speed
                twist_msg.angular.z = 0.0
                self.get_logger().info("Room width is greater or equal to length, moving forward without centering.")

        self.publisher.publish(twist_msg)

        # Follow the A* path
        if self.current_goal_index < len(self.path):
            current_goal = self.path[self.current_goal_index]
            self.move_towards_goal(current_goal)

    def move_towards_goal(self, goal):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        # Calculate the angle to the goal and set angular.z accordingly
        # Example logic for moving towards the goal, needs actual implementation
        self.publisher.publish(twist_msg)
        # Check if the robot has reached the goal
        if self.reached_goal(goal):
            self.current_goal_index += 1

    def reached_goal(self, goal):
        # Logic to determine if the robot has reached the current goal
        return False  # Placeholder, implement actual distance check

def main(args=None):
    rclpy.init(args=args)
    wall_centering_obstacle_avoidance = WallCenteringAndObstacleAvoidance()
    rclpy.spin(wall_centering_obstacle_avoidance)
    wall_centering_obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

