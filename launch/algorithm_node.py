import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class CenteringAlgorithm(Node):
    def __init__(self):
        super().__init__('centering_algorithm')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Extract range measurements from the lidar data
        lidar_ranges = msg.ranges

        # Process lidar data to find obstacles
        obstacles = self.detect_obstacles(lidar_ranges)

        # Calculate center point between two obstacles
        center_point = self.calculate_center_point(obstacles)

        # Control robot movement based on center point
        self.control_robot(center_point)

    def detect_obstacles(self, lidar_ranges):
        # Simple thresholding to detect obstacles
        obstacles = []
        for i, range_measurement in enumerate(lidar_ranges):
            if range_measurement < 10.0:  # Example threshold for obstacle detection
                obstacles.append((i, range_measurement))
        return obstacles

    def calculate_center_point(self, obstacles):
        if len(obstacles) < 2:
            self.get_logger().info('Not enough obstacles detected.')
            return None

        # Find indices of the two nearest obstacles
        nearest_obstacles = sorted(obstacles, key=lambda x: x[1])[:2]

        # Calculate center point between the two nearest obstacles
        center_point = np.mean([obstacle[1] for obstacle in nearest_obstacles])
        return center_point

    def control_robot(self, center_point):
        if center_point is None:
            return
        # Implement control logic here to move the robot towards the center point

def main(args=None):
    rclpy.init(args=args)
    centering_algorithm = CenteringAlgorithm()
    rclpy.spin(centering_algorithm)
    centering_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

