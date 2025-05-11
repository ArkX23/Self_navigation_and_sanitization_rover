#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
import numpy as np
import math

class SimpleNavigation(Node):
    def __init__(self):
        super().__init__('simple_navigation')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.goal_pose = None
        self.robot_pose = None
        self.scan_data = None
        
        # Parameters
        self.min_range = 0.18  # Ignore < 18 cm
        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0
        self.costmap_resolution = 0.02
        self.costmap_width = 4.0
        self.costmap_height = 4.0
        self.inflation_radius = 0.3
        
        self.create_timer(0.1, self.control_loop)
        
    def scan_callback(self, msg):
        self.scan_data = msg
        
    def goal_callback(self, msg):
        if msg.header.frame_id == 'odom':
            self.goal_pose = msg.pose
            self.get_logger().info(f'Goal received: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.robot_pose = trans.transform
            return True
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return False
        
    def publish_costmap(self):
        if not self.scan_data:
            return
        
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'odom'
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info.resolution = self.costmap_resolution
        costmap.info.width = int(self.costmap_width / self.costmap_resolution)
        costmap.info.height = int(self.costmap_height / self.costmap_resolution)
        costmap.info.origin.position.x = -self.costmap_width / 2.0
        costmap.info.origin.position.y = -self.costmap_height / 2.0
        costmap.info.origin.orientation.w = 1.0
        
        # Initialize grid
        grid = np.zeros((costmap.info.height, costmap.info.width), dtype=np.int8)
        
        if self.robot_pose:
            rx = self.robot_pose.translation.x
            ry = self.robot_pose.translation.y
            
            # Convert scan to costmap
            for i, r in enumerate(self.scan_data.ranges):
                if r < self.min_range or r > self.scan_data.range_max:
                    continue
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                x = rx + r * math.cos(angle)
                y = ry + r * math.sin(angle)
                
                # Map to grid
                gx = int((x - costmap.info.origin.position.x) / self.costmap_resolution)
                gy = int((y - costmap.info.origin.position.y) / self.costmap_resolution)
                
                if 0 <= gx < costmap.info.width and 0 <= gy < costmap.info.height:
                    grid[gy, gx] = 100  # Obstacle
            
            # Inflate obstacles
            for y in range(costmap.info.height):
                for x in range(costmap.info.width):
                    if grid[y, x] == 100:
                        for dy in range(-int(self.inflation_radius / self.costmap_resolution),
                                      int(self.inflation_radius / self.costmap_resolution) + 1):
                            for dx in range(-int(self.inflation_radius / self.costmap_resolution),
                                          int(self.inflation_radius / self.costmap_resolution) + 1):
                                if 0 <= y + dy < costmap.info.height and 0 <= x + dx < costmap.info.width:
                                    dist = math.sqrt(dx**2 + dy**2) * self.costmap_resolution
                                    if dist <= self.inflation_radius and grid[y + dy, x + dx] == 0:
                                        grid[y + dy, x + dx] = int(99 * (1.0 - dist / self.inflation_radius))
        
        costmap.data = grid.flatten().tolist()
        self.costmap_pub.publish(costmap)
        
    def control_loop(self):
        if not self.get_robot_pose() or not self.goal_pose or not self.scan_data:
            return
        
        # Calculate error to goal
        dx = self.goal_pose.position.x - self.robot_pose.translation.x
        dy = self.goal_pose.position.y - self.robot_pose.translation.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            self.get_logger().info('Goal reached!')
            self.goal_pose = None
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Heading to goal
        goal_angle = math.atan2(dy, dx)
        yaw = self.get_yaw(self.robot_pose.rotation)
        angle_error = self.normalize_angle(goal_angle - yaw)
        
        # Obstacle avoidance
        cmd = Twist()
        obstacle_detected = False
        min_dist = float('inf')
        safe_dist = 0.5
        
        for i, r in enumerate(self.scan_data.ranges):
            if self.min_range < r < safe_dist:
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                if abs(self.normalize_angle(angle)) < math.pi / 4:  # Front 90 degrees
                    obstacle_detected = True
                    min_dist = min(min_dist, r)
        
        if obstacle_detected:
            cmd.angular.z = self.max_angular_speed if angle_error > 0 else -self.max_angular_speed
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.max_linear_speed * distance, self.max_linear_speed)
            cmd.angular.z = max(min(angle_error * 2.0, self.max_angular_speed), -self.max_angular_speed)
        
        self.cmd_vel_pub.publish(cmd)
        self.publish_costmap()
        
    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
