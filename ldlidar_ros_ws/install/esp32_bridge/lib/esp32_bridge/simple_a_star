#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import numpy as np
import math
from heapq import heappush, heappop

class SimpleAStar(Node):
    def __init__(self):
        super().__init__('simple_a_star')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        # Parameters
        self.min_range = 0.16
        self.safety_radius = 0.15
        self.max_linear_speed = 0.3
        self.min_angular_speed = 4.5
        self.angle_tolerance = math.radians(20)  # Increased from 15
        self.min_distance = 0.1
        self.grid_resolution = 0.1
        self.grid_width = 4.0
        self.grid_height = 4.0
        self.waypoint_delay = 0.5
        
        self.scan_data = None
        self.current_pose = None
        self.goal_pose = None
        self.grid = None
        self.path = []
        self.path_index = 0
        self.at_waypoint = False
        self.waypoint_arrival_time = 0.0
        
        self.last_plan_time = 0.0
        self.plan_interval = 1.0
        self.last_command_time = 0.0
        self.command_interval = 0.1
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Simple A* navigation started')

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        }
        self.get_logger().debug(f"Odometry: x={self.current_pose['x']:.2f}, y={self.current_pose['y']:.2f}, yaw={self.current_pose['yaw']:.2f}")

    def scan_callback(self, msg):
        self.scan_data = msg
        self.get_logger().debug(f"Received scan: {len(msg.ranges)} ranges")
        self.update_costmap()

    def goal_callback(self, msg):
        if msg.header.frame_id == 'odom':
            self.goal_pose = msg.pose
            self.path = []
            self.path_index = 0
            self.at_waypoint = False
            self.get_logger().info(f'Goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
            self.plan_path()

    def update_costmap(self):
        if not self.scan_data or not self.current_pose:
            self.get_logger().warn(f"No costmap update: scan_data={'None' if self.scan_data is None else 'OK'}, current_pose={'None' if self.current_pose is None else 'OK'}")
            return
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'odom'
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info.resolution = self.grid_resolution
        costmap.info.width = int(self.grid_width / self.grid_resolution)
        costmap.info.height = int(self.grid_height / self.grid_resolution)
        costmap.info.origin.position.x = self.current_pose['x'] - self.grid_width / 2.0
        costmap.info.origin.position.y = self.current_pose['y'] - self.grid_height / 2.0
        costmap.info.origin.orientation.w = 1.0
        grid = np.zeros((costmap.info.height, costmap.info.width), dtype=np.int8)
        obstacle_count = 0
        for i, r in enumerate(self.scan_data.ranges):
            if self.min_range < r < self.scan_data.range_max and not math.isnan(r) and not math.isinf(r):
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                x_laser = r * math.cos(angle)
                y_laser = r * math.sin(angle)
                x_odom = x_laser + self.current_pose['x']
                y_odom = y_laser + self.current_pose['y']
                gx = int((x_odom - costmap.info.origin.position.x) / self.grid_resolution)
                gy = int((y_odom - costmap.info.origin.position.y) / self.grid_resolution)
                if 0 <= gx < costmap.info.width and 0 <= gy < costmap.info.height:
                    grid[gy, gx] = 100
                    obstacle_count += 1
        costmap.data = grid.flatten().tolist()
        self.grid = grid
        self.costmap_pub.publish(costmap)
        self.get_logger().info(f"Published costmap: {obstacle_count} obstacles")

    def plan_path(self):
        if self.grid is None or self.goal_pose is None or self.current_pose is None:
            return
        start_x = self.current_pose['x']
        start_y = self.current_pose['y']
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        start_gx = int((start_x - (self.current_pose['x'] - self.grid_width / 2.0)) / self.grid_resolution)
        start_gy = int((start_y - (self.current_pose['y'] - self.grid_height / 2.0)) / self.grid_resolution)
        goal_gx = int((goal_x - (self.current_pose['x'] - self.grid_width / 2.0)) / self.grid_resolution)
        goal_gy = int((goal_y - (self.current_pose['y'] - self.grid_height / 2.0)) / self.grid_resolution)
        if not (0 <= start_gx < self.grid.shape[1] and 0 <= start_gy < self.grid.shape[0] and
                0 <= goal_gx < self.grid.shape[1] and 0 <= goal_gy < self.grid.shape[0]):
            self.get_logger().warn('Start or goal outside grid')
            return
        open_list = []
        heappush(open_list, (0, (start_gx, start_gy)))
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        f_score = {(start_gx, start_gy): math.sqrt((goal_gx - start_gx)**2 + (goal_gy - start_gy)**2)}
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        while open_list:
            _, current = heappop(open_list)
            cx, cy = current
            if (cx, cy) == (goal_gx, goal_gy):
                self.path = []
                while current in came_from:
                    self.path.append(current)
                    current = came_from[current]
                self.path.append((start_gx, start_gy))
                self.path.reverse()
                self.at_waypoint = False
                self.publish_path()
                return
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0]):
                    continue
                if self.grid[ny, nx] >= 50:
                    continue
                tentative_g = g_score[(cx, cy)] + math.sqrt(dx**2 + dy**2) * self.grid_resolution
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f_score[(nx, ny)] = tentative_g + math.sqrt((goal_gx - nx)**2 + (goal_gy - ny)**2)
                    heappush(open_list, (f_score[(nx, ny)], (nx, ny)))
        self.get_logger().warn('No path found')
        self.path = []
        self.path_index = 0
        self.at_waypoint = False

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        origin_x = self.current_pose['x'] - self.grid_width / 2.0
        origin_y = self.current_pose['y'] - self.grid_height / 2.0
        for gx, gy in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(gx * self.grid_resolution + origin_x)
            pose.pose.position.y = float(gy * self.grid_resolution + origin_y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def control_loop(self):
        if not self.path or not self.current_pose:
            self.get_logger().debug('No path or pose')
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        if self.path_index >= len(self.path) - 1:
            self.get_logger().info('Goal reached at second-to-last waypoint!')
            self.path = []
            self.path_index = 0
            self.goal_pose = None
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_plan_time >= self.plan_interval:
            self.plan_path()
            self.last_plan_time = current_time
        if current_time - self.last_command_time < self.command_interval:
            return
        
        # Handle waypoint delay
        if self.at_waypoint:
            self.get_logger().debug(f"At waypoint {self.path_index}, time left: {self.waypoint_delay - (current_time - self.waypoint_arrival_time):.2f}s")
            if current_time - self.waypoint_arrival_time < self.waypoint_delay:
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                self.at_waypoint = False
                self.path_index += 1
                self.get_logger().info(f"Advancing to waypoint {self.path_index}")
                if self.path_index >= len(self.path) - 1:
                    self.get_logger().info('Goal reached at second-to-last waypoint!')
                    self.path = []
                    self.path_index = 0
                    self.goal_pose = None
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    return
        
        target_gx, target_gy = self.path[self.path_index]
        origin_x = self.current_pose['x'] - self.grid_width / 2.0
        origin_y = self.current_pose['y'] - self.grid_height / 2.0
        target_x = target_gx * self.grid_resolution + origin_x
        target_y = target_gy * self.grid_resolution + origin_y
        dx = target_x - self.current_pose['x']
        dy = target_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose['yaw']
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        cmd = Twist()
        self.get_logger().debug(f"Waypoint {self.path_index}: distance={distance:.2f}m, angle_diff={math.degrees(angle_diff):.2f}deg")
        if distance < self.min_distance:
            self.get_logger().info(f'Waypoint {self.path_index} reached, pausing for {self.waypoint_delay}s')
            self.at_waypoint = True
            self.waypoint_arrival_time = current_time
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif abs(angle_diff) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(self.min_angular_speed, angle_diff)
            self.get_logger().info(f'Rotating: angular.z={cmd.angular.z:.2f}')
        else:
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Moving: linear.x={cmd.linear.x:.2f}')
        self.cmd_vel_pub.publish(cmd)
        self.last_command_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
