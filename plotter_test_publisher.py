#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, Float32
import time

class SimplePlotterTest(Node):
    def __init__(self):
        super().__init__('simple_plotter_test')
        
        # Publisher for plotter data
        self.publisher = self.create_publisher(
            ByteMultiArray, 
            '/plotter/data', 
            10
        )
        
        # Subscriber for buffer status
        self.subscriber = self.create_subscription(
            Float32,
            '/plotter/buffer_status',
            self.buffer_callback,
            10
        )
        
        self.buffer_free = 100.0
        self.get_logger().info('Simple plotter test started')
        
    def buffer_callback(self, msg):
        self.buffer_free = msg.data
        self.get_logger().info(f'Buffer: {self.buffer_free:.1f}% free')
    
    def send_bytes(self, byte_list):
        """Send bytes to plotter"""
        msg = ByteMultiArray()
        # Ensure we're sending bytes, not a list
        if isinstance(byte_list, list):
            msg.data = bytes(byte_list)
        else:
            msg.data = byte_list
            
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent {len(msg.data)} bytes')
        
    def test_patterns(self):
        """Send various test patterns"""
        
        # Pattern 1: Simple movement right (nibble 0x1)
        self.get_logger().info('Pattern 1: Move right')
        self.send_bytes([0x11, 0x11, 0x11, 0x11])  # 8 moves right
        time.sleep(1)
        
        # Pattern 2: Move up (nibble 0x2)
        self.get_logger().info('Pattern 2: Move up')
        self.send_bytes([0x22, 0x22, 0x22, 0x22])  # 8 moves up
        time.sleep(1)
        
        # Pattern 3: Diagonal (nibbles 0x3)
        self.get_logger().info('Pattern 3: Diagonal')
        self.send_bytes([0x33, 0x33, 0x33, 0x33])  # 8 diagonal moves
        time.sleep(1)
        
        # Pattern 4: Square
        self.get_logger().info('Pattern 4: Square')
        self.send_bytes([
            0x11, 0x11,  # Right
            0x22, 0x22,  # Up
            0x44, 0x44,  # Left
            0x88, 0x88   # Down
        ])
        time.sleep(1)
        
        # Pattern 5: Pen up/down (0xF for pen command)
        self.get_logger().info('Pattern 5: Pen control')
        self.send_bytes([
            0xF0,  # Pen up
            0x11, 0x11,  # Move right with pen up
            0xF1,  # Pen down
            0x11, 0x11   # Move right with pen down
        ])

def main():
    rclpy.init()
    node = SimplePlotterTest()
    
    try:
        # Wait a bit for connection
        time.sleep(1)
        
        # Run test patterns
        node.test_patterns()
        
        # Keep node alive to receive responses
        rclpy.spin_once(node, timeout_sec=2.0)
        
        node.get_logger().info('Test complete!')
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
