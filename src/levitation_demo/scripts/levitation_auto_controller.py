#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LevitationAutoController(Node):
    def __init__(self):
        



def main():
    rclpy.init()
    node = Node("lev_auto_control_node")
    node.get_logger().info("Levitation Auto Controller Node has started!")
    rclpy.shutdown()

main()  
