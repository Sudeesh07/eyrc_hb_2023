#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import math

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.angular_speed = 0.5  # radians per second (slower rotation)
        self.linear_speed = 0.5  # meters per second (faster movement)
        self.time_to_draw_circle = 2 * math.pi / self.angular_speed  # Single rotation
        self.elapsed_time = 0
        self.turtlebot2_spawned = False

    def timer_callback(self):
        twist = Twist()
        if self.elapsed_time < self.time_to_draw_circle:
            twist.angular.z = self.angular_speed
            twist.linear.x = self.linear_speed
            self.publisher_.publish(twist)
            self.elapsed_time += self.timer_period
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            self.timer.cancel()
            if not self.turtlebot2_spawned:
                self.turtlebot2_spawned = True
                self.spawn_turtlebot2()

    def spawn_turtlebot2(self):
        rob_name = "turtle2"
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "/spawn" not available, waiting...')

        request = Spawn.Request()
        request.name = rob_name
        request.x = 5.544445  # Initial X-coordinate
        request.y = 5.544445 # Initial Y-coordinate
        request.theta = 0.0  # Initial orientation (in radians)

        future = client.call_async(request)
        timeout_sec = 10.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        self.draw_circle_with_turtlebot2('turtle2')

    def draw_circle_with_turtlebot2(self, turtlebot2_name):
        self.pub = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.timer1 = self.create_timer(self.timer_period, self.timer_callback)
        self.angular_speed = -0.5  # Negative angular speed (radians per second) for clockwise rotation
        self.linear_speed = 1.0  # meters per second (forward movement)
        self.elapsed_time = 0

        twist = Twist()
        while(self.elapsed_time<2280):
            twist.angular.z = self.angular_speed
            twist.linear.x = self.linear_speed
            self.pub.publish(twist)
            self.elapsed_time += self.timer_period
            self.get_logger().info(f'time = {self.elapsed_time}')
        
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    draw_circle_node = DrawCircleNode()
    rclpy.spin(draw_circle_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
