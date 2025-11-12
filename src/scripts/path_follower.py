import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simulated_annealing import solve_tsp
from useful_functions import get_tsp_waypoints
import sys
import math
import json

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        maze_name = sys.argv[1]
        with open('../worlds/config/maze_mapping.json', 'r') as file:
            data = json.load(file)
        
        self.expected_start_x = data[maze_name]["start_point"]["world"]["x"]
        self.expected_start_y = data[maze_name]["start_point"]["world"]["y"]
        
        start_x_grid = data[maze_name]["start_point"]["grid"]["x"]
        start_y_grid = data[maze_name]["start_point"]["grid"]["y"]
        end_x_grid = data[maze_name]["end_point"]["grid"]["x"]
        end_y_grid = data[maze_name]["end_point"]["grid"]["y"]
        start_point = (start_x_grid, start_y_grid)
        end_point = (end_x_grid, end_y_grid)
        occupancy_grid = data[maze_name]["occupancy_grid"]
        points_to_visit = get_tsp_waypoints(occupancy_grid)

        tsp_route = solve_tsp(maze_name, start_point, end_point, occupancy_grid, points_to_visit)
        if not tsp_route:
            print("\nExiting...")
            rclpy.shutdown()
            sys.exit(0)
        self.waypoints_original = self.gridToWorld(tsp_route)

        self.waypoints = []
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.waypoints_adjusted = False
        
        # Current state
        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.start_x = None
        self.start_y = None
        
        self.max_linear_speed = 0.5  
        self.max_angular_speed = 0.5  
        self.position_tolerance = 0.08  
        self.angle_tolerance = 0.08  
        
        # Proportional control gains
        self.kp_linear = 1.0   # Proportional gain for linear velocity
        self.kp_angular = 2.0  # Proportional gain for angular velocity
        
        # State machine
        self.state = 'ROTATE'  # ROTATE or MOVE
        
        self.log_counter = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waypoint Navigator initialized')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'Position tolerance: {self.position_tolerance} m')
        self.get_logger().info(f'Angle tolerance: {math.degrees(self.angle_tolerance):.1f}°')
        self.get_logger().info('=' * 60)
    
    def gridToWorld(self, path, width=9, height=9, cell_size=2.0):
        new_path = []
        for coordinates in path:
            grid_x = coordinates[0]
            grid_y = height - 1 - coordinates[1]  
            
            world_x = (grid_x - width / 2) * cell_size
            world_y = (grid_y - height / 2) * cell_size
            
            new_path.append((world_x, world_y))
        return new_path
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        
        if not self.odom_received:
            self.odom_received = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            
            self.offset_x = self.current_x - self.expected_start_x
            self.offset_y = self.current_y - self.expected_start_y
            
            self.waypoints = []
            for wx, wy in self.waypoints_original:
                adjusted_x = wx + self.offset_x
                adjusted_y = wy + self.offset_y
                self.waypoints.append((adjusted_x, adjusted_y))
            
            self.waypoints = self.waypoints[1:]
            
            self.get_logger().info(f'First odometry received!')
            self.get_logger().info(f'Expected start: ({self.expected_start_x}, {self.expected_start_y})')
            self.get_logger().info(f'Actual start: ({self.current_x:.2f}, {self.current_y:.2f})')
            self.get_logger().info(f'Offset applied: ({self.offset_x:.2f}, {self.offset_y:.2f})')
            self.get_logger().info(f'Starting orientation: {math.degrees(self.current_yaw):.1f}°')
            self.get_logger().info(f'Adjusted waypoints: {len(self.waypoints)} points')
            self.get_logger().info('-' * 60)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_target_angle(self, target_x, target_y):
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.atan2(dy, dx)
    
    def get_distance_to_target(self, target_x, target_y):
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def control_loop(self):
        if not self.odom_received:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('=' * 60)
            self.get_logger().info('All waypoints reached! Stopping.')
            self.get_logger().info(f'Final position: ({self.current_x:.2f}, {self.current_y:.2f})')
            self.get_logger().info('=' * 60)
            self.stop_robot()
            self.timer.cancel()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        distance = self.get_distance_to_target(target_x, target_y)
        target_angle = self.get_target_angle(target_x, target_y)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        twist = Twist()
        
        should_log = (self.log_counter % 10 == 0)
        self.log_counter += 1
        
        if self.state == 'ROTATE':
            if abs(angle_error) > self.angle_tolerance:
                angular_vel = self.kp_angular * angle_error
                twist.angular.z = max(-self.max_angular_speed, 
                                     min(self.max_angular_speed, angular_vel))
                
                if should_log:
                    self.get_logger().info(
                        f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
                        f'ROTATING | Angle error: {math.degrees(angle_error):.1f}° | '
                        f'Distance: {distance:.2f}m'
                    )
            else:
                self.state = 'MOVE'
                self.get_logger().info(
                    f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
                    f'Rotation complete! Moving forward...'
                )
        
        elif self.state == 'MOVE':
            if distance > self.position_tolerance:
                if abs(angle_error) > self.angle_tolerance * 3:
                    self.state = 'ROTATE'
                    self.get_logger().info(
                        f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
                        f'Course correction needed (angle error: {math.degrees(angle_error):.1f}°)'
                    )
                else:
                    linear_vel = self.kp_linear * distance
                    linear_vel = max(0.2, min(self.max_linear_speed, linear_vel))
                    
                    angular_vel = self.kp_angular * angle_error * 0.3  
                    angular_vel = max(-self.max_angular_speed * 0.5, 
                                     min(self.max_angular_speed * 0.5, angular_vel))
                    
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                    
                    if should_log:
                        self.get_logger().info(
                            f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
                            f'MOVING | Distance: {distance:.2f}m | Speed: {linear_vel:.2f} m/s | '
                            f'Angle: {math.degrees(angle_error):.1f}°'
                        )
            else:
                self.get_logger().info(
                    f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
                    f'REACHED waypoint at ({target_x}, {target_y}) | '
                    f'Actual: ({self.current_x:.2f}, {self.current_y:.2f}) | '
                    f'Error: {distance*100:.1f}cm'
                )
                self.get_logger().info('-' * 60)
                self.current_waypoint_idx += 1
                self.state = 'ROTATE'
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Keyboard interrupt, stopping...')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()