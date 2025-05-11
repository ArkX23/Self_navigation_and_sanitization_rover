#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import numpy as np
import math
from heapq import heappush, heappop
import socket
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class SimpleAStarNav(Node):
    def __init__(self):
        super().__init__('simple_a_star_nav')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.min_range = 0.18
        self.safe_dist = 0.5
        self.max_linear_speed = 0.4
        self.max_angular_speed = 6.0
        self.min_angular_speed = 5.0
        self.grid_resolution = 0.1
        self.grid_width = 4.0
        self.grid_height = 4.0
        self.inflation_radius = 0.2
        
        self.scan_data = None
        self.goal_pose = None
        self.grid = None
        self.path = []
        self.current_pose = None
        
        # UDP setup
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp32_ip = "192.168.121.1"
        self.odom_port = 12346
        self.cmd_port = 12347
        self.reset_port = 12348
        self.udp_sock.bind(('', self.odom_port))
        self.udp_sock.settimeout(0.1)
        
        self.create_timer(0.1, self.odom_callback)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Simple A* navigation started')

    def odom_callback(self):
        try:
            data, _ = self.udp_sock.recvfrom(64)
            x, y, yaw = map(float, data.decode().split(','))
            
            self.current_pose = {'x': x, 'y': y, 'yaw': yaw}
            
            # Publish Odometry
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.odom_pub.publish(odom)
            
            # Broadcast TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self.tf_broadcaster.sendTransform(t)
            
            self.get_logger().info(f'Odometry: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f'Odom error: {e}')

    def scan_callback(self, msg):
        self.scan_data = msg
        valid_ranges = [r for r in msg.ranges if self.min_range < r < msg.range_max and not math.isnan(r) and not math.isinf(r)]
        self.get_logger().info(f'Received scan with {len(valid_ranges)} valid ranges')
        self.update_costmap()

    def goal_callback(self, msg):
        if msg.header.frame_id == 'base_link':
            self.goal_pose = msg.pose
            self.get_logger().info(f'Goal received: x={msg.pose.position.x}, y={msg.pose.position.y}')
            self.plan_path()

    def update_costmap(self):
        if not self.scan_data:
            self.get_logger().warn('No scan data available')
            return
            
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'base_link'
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info.resolution = self.grid_resolution
        costmap.info.width = int(self.grid_width / self.grid_resolution)
        costmap.info.height = int(self.grid_height / self.grid_resolution)
        costmap.info.origin.position.x = -self.grid_width / 2.0
        costmap.info.origin.position.y = -self.grid_height / 2.0
        costmap.info.origin.orientation.w = 1.0
        
        grid = np.zeros((costmap.info.height, costmap.info.width), dtype=np.int8)
        
        valid_points = 0
        try:
            for i, r in enumerate(self.scan_data.ranges):
                if r < self.min_range or r > self.scan_data.range_max or math.isnan(r) or math.isinf(r):
                    continue
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                
                if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                    continue
                
                # Transform laser point to base_link
                point = PointStamped()
                point.header.frame_id = 'laser'
                point.header.stamp = self.scan_data.header.stamp
                point.point.x = x
                point.point.y = y
                point.point.z = 0.0
                
                try:
                    transform = self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
                    point_transformed = do_transform_point(point, transform)
                    x = point_transformed.point.x
                    y = point_transformed.point.y
                except Exception as e:
                    self.get_logger().warn(f'TF error: {e}')
                    continue
                
                gx = int((x - costmap.info.origin.position.x) / self.grid_resolution)
                gy = int((y - costmap.info.origin.position.y) / self.grid_resolution)
                
                if 0 <= gx < costmap.info.width and 0 <= gy < costmap.info.height:
                    grid[gy, gx] = 100
                    valid_points += 1
        
            self.get_logger().info(f'Costmap updated with {valid_points} obstacle points')
            
            for y in range(costmap.info.height):
                for x in range(costmap.info.width):
                    if grid[y, x] == 100:
                        for dy in range(-int(self.inflation_radius / self.grid_resolution),
                                      int(self.inflation_radius / self.grid_resolution) + 1):
                            for dx in range(-int(self.inflation_radius / self.grid_resolution),
                                          int(self.inflation_radius / self.grid_resolution) + 1):
                                if 0 <= y + dy < costmap.info.height and 0 <= x + dx < costmap.info.width:
                                    dist = math.sqrt(dx**2 + dy**2) * self.grid_resolution
                                    if dist <= self.inflation_radius and grid[y + dy, x + dx] == 0:
                                        grid[y + dy, x + dx] = int(99 * (1.0 - dist / self.inflation_radius))
        
            costmap.data = grid.flatten().tolist()
            self.grid = grid
            self.costmap_pub.publish(costmap)
        except Exception as e:
            self.get_logger().error(f'Costmap update failed: {e}')

    def plan_path(self):
        if self.grid is None or self.goal_pose is None or self.current_pose is None:
            self.get_logger().warn('Cannot plan: missing grid, goal, or pose')
            return
            
        start_x = self.current_pose['x']
        start_y = self.current_pose['y']
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        start_gx = int((start_x + self.grid_width / 2.0) / self.grid_resolution)
        start_gy = int((start_y + self.grid_height / 2.0) / self.grid_resolution)
        goal_gx = int((goal_x + self.grid_width / 2.0) / self.grid_resolution)
        goal_gy = int((goal_y + self.grid_height / 2.0) / self.grid_resolution)
        
        if not (0 <= start_gx < self.grid.shape[1] and 0 <= start_gy < self.grid.shape[0] and
                0 <= goal_gx < self.grid.shape[1] and 0 <= goal_gy < self.grid.shape[0]):
            self.get_logger().warn('Start or goal outside grid')
            return
        
        open_list = []
        heappush(open_list, (0, (start_gx, start_gy)))
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        f_score = {(start_gx, start_gy): self.heuristic(start_gx, start_gy, goal_gx, goal_gy)}
        
        while open_list:
            _, current = heappop(open_list)
            cx, cy = current
            
            if (cx, cy) == (goal_gx, goal_gy):
                self.path = self.reconstruct_path(came_from, current)
                self.publish_path()
                self.get_logger().info('Path planned')
                return
            
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0]):
                    continue
                if self.grid[ny, nx] >= 50:
                    continue
                
                tentative_g = g_score[(cx, cy)] + self.grid_resolution
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f_score[(nx, ny)] = tentative_g + self.heuristic(nx, ny, goal_gx, goal_gy)
                    heappush(open_list, (f_score[(nx, ny)], (nx, ny)))
        
        self.get_logger().warn('No path found')
        self.path = []

    def heuristic(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2) * self.grid_resolution

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'base_link'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for gx, gy in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = (gx * self.grid_resolution) - (self.grid_width / 2.0)
            pose.pose.position.y = (gy * self.grid_resolution) - (self.grid_height / 2.0)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.get_logger().info(f'Publishing path with {len(self.path)} waypoints')
        self.path_pub.publish(path_msg)

    def reset_odom(self):
        try:
            self.udp_sock.sendto(b"reset", (self.esp32_ip, self.reset_port))
            self.get_logger().info('Sent reset command to ESP32')
            self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            
            # Publish reset odometry
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = 0.0
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            
            # Broadcast reset TF
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warn(f'Failed to send reset: {e}')

    def control_loop(self):
        if not self.path or not self.scan_data or not self.current_pose:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().warn('No path, scan, or pose, stopping')
            return
            
        next_gx, next_gy = self.path[0]
        next_x = (next_gx * self.grid_resolution) - (self.grid_width / 2.0)
        next_y = (next_gy * self.grid_resolution) - (self.grid_height / 2.0)
        
        dx = next_x - self.current_pose['x']
        dy = next_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx) - self.current_pose['yaw']
        goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
        
        self.get_logger().info(f'Target: x={next_x:.2f}, y={next_y:.2f}, dist={distance:.2f}, angle={goal_angle:.2f}')
        
        if distance < 0.4:
            self.path.pop(0)
            self.get_logger().info(f'Waypoint reached, {len(self.path)} left')
            self.publish_path()
            if not self.path:
                self.get_logger().info('Goal reached!')
                self.reset_odom()
                self.goal_pose = None
                self.path = []
                self.publish_path()
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return
        
        cmd = Twist()
        obstacle_detected = False
        front_angle = math.pi / 4
        for i, r in enumerate(self.scan_data.ranges):
            if self.min_range < r < self.safe_dist:
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                if abs(angle) < front_angle:
                    obstacle_detected = True
                    break
        
        if obstacle_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed
            self.get_logger().info('Obstacle detected, turning')
        else:
            cmd.linear.x = self.max_linear_speed if abs(goal_angle) < math.pi / 8 else 0.0
            if abs(goal_angle) > 0.05:
                cmd.angular.z = math.copysign(max(self.min_angular_speed, abs(goal_angle * 1.0)), goal_angle)
            else:
                cmd.angular.z = 0.0
            self.get_logger().info(f'Moving: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}')
        
        self.cmd_vel_pub.publish(cmd)
        packet = f"{cmd.linear.x:.3f},{cmd.angular.z:.3f}"
        self.udp_sock.sendto(packet.encode(), (self.esp32_ip, self.cmd_port))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStarNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
