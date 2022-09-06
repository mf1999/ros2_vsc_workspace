import rclpy
from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float32

DISTANCE_TOPIC_PARAM = 'inputDistanceToCage'
THRUSTER_TOPIC_PARAM = 'outputThrusterForce'

class Controller(Node):
        def __init__(self, name):
                super().__init__(name)
                self.get_logger().info('Pozdrav brate!')
                
                self.declare_node_params()
                # self.get_ros_params()
                # self.load_resources_paths()

                self.initialize_subscribers()
                self.initialize_publishers()

                self.initialize_class_objects()

        def declare_node_params(self):
            self.declare_parameter(DISTANCE_TOPIC_PARAM, '/placeholder_distance_topic')
            self.declare_parameter(THRUSTER_TOPIC_PARAM, '/placeholder_thruster_topic')

        def initialize_subscribers(self):
            self.create_subscription(Float32, self.get_parameter(DISTANCE_TOPIC_PARAM).value, self.distance_callback, 10)

        def initialize_publishers(self):
            self.thruster_pub = self.create_publisher(Float32, self.get_parameter(THRUSTER_TOPIC_PARAM).value, 10)

        def distance_callback(self, msg):
            output = self.pid(msg.data)

            msg = Float32()
            msg.data = output
            self.thruster_pub.publish(msg)

        def initialize_class_objects(self):
            self.pid = PID(1, 0.1, 0.05, setpoint=200)
            self.pid.sample_time = 1.0                  #seconds
            self.pid.output_limits = (-1.0, 1.0)

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