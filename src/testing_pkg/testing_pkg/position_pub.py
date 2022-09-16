from termios import PARENB
from tkinter import Y
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import numpy as np
from time import sleep
POSITION_TOPIC_PARAM = 'outputPosition'
FLOAT_VALUE_TOPIC_PARAM = 'outputValueForPos'

class PositionPublisher(Node):
        def __init__(self, name):
                super().__init__(name)
                self.get_logger().info('Pozdrav!')
                
                self.declare_node_params()
                # self.get_ros_params()
                # self.load_resources_paths()

                # self.initialize_subscribers()
                self.initialize_publishers()

                self.initialize_class_objects()

                timer_period = 0.02 # seconds
     
                self.timer = self.create_timer(timer_period, self.timer_callback)
        
        def timer_callback(self):
                # self.get_logger().info("Timer callback message.")
                
                try:
                        x = next(self.X)
                except StopIteration:
                        
                        try:
                                self.y = next(self.Y)
                        except StopIteration:
                                sleep(0.5)
                                self.destroy_node()
                
                        self.X_space = np.flip(self.X_space)
                        self.X = iter(self.X_space)
                        x = next(self.X)
                
                try:
                        z = next(self.Z)
                except StopIteration:
                        self.Z_space = np.flip(self.Z_space)
                        self.Z = iter(self.Z_space)
                        z = next(self.Z)

                try:
                        v = next(self.V)
                except StopIteration:
                        self.V_space = np.flip(self.V_space)
                        self.V = iter(self.V_space)
                        v = next(self.V)

                # self.get_logger().info(f"x:{x}, y:{self.y}, z:{z}")
                msg = Vector3Stamped()
                msg.header.stamp = self.get_clock().now().to_msg()

                msg.vector.x = x
                msg.vector.y = self.y
                msg.vector.z = z
                self.pos_pub.publish(msg)

                msg = Float32()
                msg.data = v
                self.val_pub.publish(msg)

        def declare_node_params(self):
                self.declare_parameter(POSITION_TOPIC_PARAM)
                self.declare_parameter(FLOAT_VALUE_TOPIC_PARAM)
        
        def initialize_publishers(self):
                self.pos_pub = self.create_publisher(Vector3Stamped, self.get_parameter(POSITION_TOPIC_PARAM).get_parameter_value().string_value, 10)
                self.val_pub = self.create_publisher(Float32, self.get_parameter(FLOAT_VALUE_TOPIC_PARAM).get_parameter_value().string_value, 10)

        def initialize_class_objects(self):
                self.X_space = np.linspace(-100, 100, 200)
                self.Y_space = np.linspace(30, 0, 10)
                self.Z_space = np.linspace(0, 5, 50)
                self.V_space= np.linspace(0, 100, 200)

                self.X = iter(self.X_space)
                self.Y = iter(self.Y_space)
                self.Z = iter(self.Z_space)
                self.V = iter(self.V_space)

                self.y = next(self.Y)

def main(args=None):
    rclpy.init(args=args)
    
    # try:
    node = PositionPublisher('pos_pub')
    rclpy.spin(node)
    # except Exception as e:
    #    print(e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()