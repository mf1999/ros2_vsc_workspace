import rclpy
from rclpy.node import Node
from simple_pid import PID
from geometry_msgs.msg import Vector3Stamped, Twist

POSITION_TOPIC_PARAM = 'inputRelativePosition'
THRUSTER_TOPIC_PARAM = 'outputThrusterForce'

class Controller(Node):
        def __init__(self, name):
                super().__init__(name)
                self.get_logger().info('Pozdrav!')
                
                self.declare_node_params()
                # self.get_ros_params()
                # self.load_resources_paths()

                self.initialize_subscribers()
                self.initialize_publishers()

                self.initialize_class_objects()

        def declare_node_params(self):
            self.declare_parameter(POSITION_TOPIC_PARAM, '/placeholder_position_topic')
            self.declare_parameter(THRUSTER_TOPIC_PARAM, '/placeholder_thruster_topic')

        def initialize_subscribers(self):
            self.create_subscription(Vector3Stamped, self.get_parameter(POSITION_TOPIC_PARAM).value, self.position_callback, 10)

        def initialize_publishers(self):
            self.thruster_pub = self.create_publisher(Twist, self.get_parameter(THRUSTER_TOPIC_PARAM).value, 10)

        def position_callback(self, msg):
            x = msg.vector.x
            y = msg.vector.y
            z = msg.vector.z
            x_output = self.pid(x)
            self.get_logger().info(f"Integral: {self.pid._integral}")

            msg = Twist()
            msg.linear.x = x_output
            self.thruster_pub.publish(msg)

        def initialize_class_objects(self):
            self.pid = PID(1, 0.1, 0.05, setpoint=2000.0)
            self.pid.sample_time = 1.0                  #seconds
            self.pid.output_limits = (-0.4, 0.4)

def main(args=None):
    rclpy.init(args=args)
    
    #try:
    node = Controller('distance_controller')
    rclpy.spin(node)
    #except Exception as e:
    #    print(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()