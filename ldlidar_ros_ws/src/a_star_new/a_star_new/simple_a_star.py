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

class SimpleAStar(Node):
    def __init__(self):
        super().__init__('simple_a_star')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
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
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
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
        
        # Reset odometry on startup
        self.reset_odom()
        self.get_logger().info('A* navigation started')

    def reset_odom(self):
        try:
            self.udp_sock.sendto(b"reset", (self.esp32_ip, self.reset_port))
            self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info('Odometry reset')
        except Exception as e:
            self.get_logger().warn(f'Reset failed: {e}')

    def odom_callback(self):
        try:
            data, _ = self.udp_sock.recvfrom(64)
            x, y, yaw = map(float, data.decode().split(','))
            self.current_pose = {'x': x, 'y': y, 'yaw': yaw}
            
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.odom_pub.publish(odom)
            
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self.tf_broadcaster.sendTransform(t)
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f'Odom error: {e}')

    def scan_callback(self, msg):
        self.scan_data = msg
        self.update_costmap()

    def goal_callback(self, msg):
        if msg.header.frame_id == 'odom':
            self.goal_pose = msg.pose
            self.get_logger().info(f'Goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
            self.plan_path()

    def update_costmap(self):
        if not self.scan_data:
            return
            
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'odom'
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info.resolution = self.grid_resolution
        costmap.info.width = int(self.grid_width / self.grid_resolution)
        costmap.info.height = int(self.grid_height / self.grid_resolution)
        costmap.info.origin.position.x = -self.grid_width / 2.0
        costmap.info.origin.position.y = -self.grid_height / 2.0
        costmap.info.origin.orientation.w = 1.0
        
        grid = np.zeros((costmap.info.height, costmap.info.width), dtype=np.int8)
        
        for i, r in enumerate(self.scan_data.ranges):
            if self.min_range < r < self.scan_data.range_max and not math.isnan(r) and not math.isinf(r):
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                x = r * math.cos(angle + self.current_pose['yaw']) + self.current_pose['x']
                y = r * math.sin(angle + self.current_pose['yaw']) + self.current_pose['y']
                
                gx = int((x - costmap.info.origin.position.x) / self.grid_resolution)
                gy = int((y - costmap.info.origin.position.y) / self.grid_resolution)
                
                if 0 <= gx < costmap.info.width and 0 <= gy < costmap.info.height:
                    grid[gy, gx] = 100
        
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

    def plan_path(self):
        if self.grid is None or self.goal_pose is None or self.current_pose is None:
            return
            
        start_gx = int((self.current_pose['x'] + self.grid_width / 2.0) / self.grid_resolution)
        start_gy = int((self.current_pose['y'] + self.grid_height / 2.0) / self.grid_resolution)
        goal_gx = int((self.goal_pose.position.x + self.grid_width / 2.0) / self.grid_resolution)
        goal_gy = int((self.goal_pose.position.y + self.grid_height / 2.0) / self.grid_resolution)
        
        if not (0 <= start_gx < self.grid.shape[1] and 0 <= start_gy < self.grid.shape[0] and
                0 <= goal_gx < self.grid.shape[1] and 0 <= goal_gy < self.grid.shape[0]):
            self.get_logger().warn('Start or goal outside grid')
            return
        
        open_list = []
        heappush(open_list, (0, (start_gx, start_gy)))
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        f_score = {(start_gx, start_gy): math.sqrt((goal_gx - start_gx)**2 + (goal_gy - start_gy)**2)}
        
        while open_list:
            _, current = heappop(open_list)
            cx, cy = current
            
            if (cx, cy) == (goal_gx, goal_gy):
                self.path = []
                while current in came_from:
                    self.path.append(current)
                    current = came_from[current]
                self.path.append(current)
                self.path = self.path[::-1]
                self.publish_path()
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
                    f_score[(nx, ny)] = tentative_g + math.sqrt((goal_gx - nx)**2 + (goal_gy - ny)**2)
                    heappush(open_list, (f_score[(nx, ny)], (nx, ny)))
        
        self.get_logger().warn('No path found')
        self.path = []

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for gx, gy in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = (gx * self.grid_resolution) - (self.grid_width / 2.0)
            pose.pose.position.y = (gy * self.grid_resolution) - (self.grid_height / 2.0)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def control_loop(self):
        if not self.path or not self.scan_data or not self.current_pose:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
            
        next_gx, next_gy = self.path[0]
        next_x = (next_gx * self.grid_resolution) - (self.grid_width / 2.0)
        next_y = (next_gy * self.grid_resolution) - (self.grid_height / 2.0)
        
        dx = next_x - self.current_pose['x']
        dy = next_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx) - self.current_pose['yaw']
        goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))
        
        if distance < 0.1:
            self.path.pop(0)
            self.publish_path()
            if not self.path:
                self.get_logger().info('Goal reached!')
                self.reset_odom()
                self.goal_pose = None
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return
        
        cmd = Twist()
        obstacle_detected = False
        for i, r in enumerate(self.scan_data.ranges):
            if self.min_range < r < self.safe_dist:
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                if abs(angle) < math.pi / 4:
                    obstacle_detected = True
                    break
        
        if obstacle_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed
        else:
            cmd.linear.x = self.max_linear_speed if abs(goal_angle) < math.pi / 8 else 0.0
            cmd.angular.z = math.copysign(max(self.min_angular_speed, abs(goal_angle)), goal_angle)
        
        self.cmd_vel_pub.publish(cmd)
        packet = f"{cmd.linear.x:.3f},{cmd.angular.z:.3f}"
        self.udp_sock.sendto(packet.encode(), (self.esp32_ip, self.cmd_port))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

