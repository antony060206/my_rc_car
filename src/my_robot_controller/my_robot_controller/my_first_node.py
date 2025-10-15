#!/usr/bin/env python3 
import rclpy # python library for ROS 2
from rclpy.node import Node

class MyNode(Node):

    def __init__(self): 
        super().__init__("first_node")
        #self.get_logger().info("Hello from ROS2")
        self.counter_= 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_+=1 


#create a main function
def main(args=None):
    rclpy.init(args=args) 
    #nodes
    node = MyNode()#creats an object 
    rclpy.spin(node)#This is going to continue to run unti ^C
    rclpy.shutdown()

if  __name__ == '__main__':
    main()

    