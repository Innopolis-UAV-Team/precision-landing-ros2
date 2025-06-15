#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MinimalTimerTest(Node):
    def __init__(self):
        super().__init__('minimal_timer_test')
        self.get_logger().info("Minimal timer test node created")
        
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Timer created - waiting for callbacks...")
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"TIMER WORKS! Call #{self.counter}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalTimerTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()