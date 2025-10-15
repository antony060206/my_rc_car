#! usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class rc_control(Node): 
    def __init__(self, _base_cls=None   ):
        super().__init__('rc_controller')
        
        #creates the subscriber that subscribes to the data from joysticks 
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        #creates the publisher 
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.connected = False
        #set up the angle and the speed
        self.maxspeed = 1.0
        self.anglespeed = 1.0

        self.get_logger().info("Remote control car has been started")

    
    #interpret the data from the joystick
    def joy_callback(self, msg):
        self.connected = True

        #get velocity and angle
        twist = Twist()
        left_trigger = 1 - msg.axes[2] #assume axis 3 is the left trigger moves from -1 to 1
        right_trigger = 1 - msg.axes[5] #assume axis 6 is the right trigger
        steering = msg.axes[0]

        linear_speed = (right_trigger - left_trigger)/2 * self.maxspeed
        angular_speed = -steering
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        self.publisher.publish(twist)



    def check_connection(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = rc_control(Node)
    rclpy.spin(xbox_controller_node)
    xbox_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()