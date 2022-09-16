import rclpy
from rclpy.node import Node
from simple_pid import PID

from geometry_msgs.msg import Vector3Stamped, Twist, Pose
from std_msgs.msg import Float32
from biofouling_interfaces.msg import Float64Custom
from numpy import arctan2, arcsin, pi
import numpy as np

POSITION_TOPIC_PARAM    = 'inputRelativePosition'
THRUSTER_TOPIC_PARAM    = 'outputThrusterForce'
SLOW_GAIN_TOPIC_PARAM   = 'outputSlowGainRef'
START_POSITION_PARAM    = 'starting_position'
LEFT_LIMIT_PARAM        = 'limit_left'
RIGHT_LIMIT_PARAM       = 'limit_right'
DEPTH_LIMIT_PARAM       = 'limit_depth'
X_DSCRTZ_PARAM          = 'x_discretization'
DEPTH_DSCRTZ_PARAM      = 'depth_discretization'
TOLERANCE_PARAM         = 'tolerance'
SLOW_GAIN_REF_PARAM     = 'slow_gain_ref'
MAX_THRUST_POS_PARAM    = 'max_thrust_pos'
MAX_THRUST_NEG_PARAM    = 'max_thrust_neg'

MOVE_TO_START   = 0
LEFT_TO_RIGHT   = 1
RIGHT_TO_LEFT   = 2
DIVE            = 3


def get_lower_upper(x, epsilon):
    
    # if x >= 0.0:
    lower = x - epsilon
    upper = x + epsilon 
    # else:
    #     upper = x - epsilon
    #     lower = x + epsilon 

    return lower, upper

class State:

    def position_callback(self, msg: Vector3Stamped):
        pass

    def prepare_exit(self):
        pass

class MoveToStart(State):
    def __init__(self, controller: Node, limit_l, limit_r, start_depth, limit_depth, distance,\
        x_discretization=10, depth_discretization=3, tolerance=0.05):

        self.controller = controller

        self.limit_l = limit_l
        self.limit_r = limit_r

        self.start_depth = start_depth
        self.limit_depth = limit_depth

        self.distance = distance
        
        self.x_discretization = x_discretization
        self.depth_discretization = depth_discretization

        self.tolerance = tolerance
        
        self.pid_x = PID(1.0, 1.0, 0.0, setpoint=limit_l)
        self.pid_x.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_x.output_limits = (-1.0, 1.0) 

        self.pid_y = PID(1.0, 1.0, 0.0, setpoint=start_depth)
        self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_y.output_limits = (-1.0, 1.0)

        self.pid_z = PID(1.0, 1.0, 0.0, setpoint=distance)
        self.pid_z.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_z.output_limits = (-1.0, 1.0)
        
        self.start_check = 0
        self.depths = self.calculate_depths()

    def position_callback(self, msg: Vector3Stamped):
        pub_msg = Twist()
        pos_vector = msg.vector

        ###
        x = self.limit_l
        y = self.depths[0]
        z = self.distance
        self.controller.get_logger().info(f"Wanted -> x: {x}, y: {y}, z: {z}")        
        ### 

        lower, upper = get_lower_upper(x, self.tolerance)
        # self.controller.get_logger().info(f"X Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.x < upper:
            pub_msg.linear.x = self.pid_x(pos_vector.x)
            p, i, d = self.pid_x.components
            self.controller.get_logger().info(f"p, i, d: {p}, {i}, {d}") 
            # self.controller.get_logger().info(f"Setting X to {self.pid_x(pos_vector.x)} according to pid.")

        lower, upper = get_lower_upper(y, self.tolerance)
        # self.controller.get_logger().info(f"Y Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.y < upper:
            pub_msg.linear.y = self.pid_y(pos_vector.y)
        #     self.controller.get_logger().info(f"Setting Y to {self.pid_y(pos_vector.y)} according to pid.")

        lower, upper = get_lower_upper(z, self.tolerance)
        # self.controller.get_logger().info(f"Z Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.z < upper:
            pub_msg.linear.z = self.pid_z(pos_vector.z)
        #     self.controller.get_logger().info(f"Setting Z to {self.pid_z(pos_vector.z)} according to pid.")
        
        self.controller.thruster_pub.publish(pub_msg)

        if pub_msg.linear.x == 0.0 and pub_msg.linear.y == 0.0 and pub_msg.linear.z == 0.0:
            self.start_check += 1
        else:
            self.start_check = 0

        if self.start_check >= 2:
            self.controller.get_logger().info(f"Switching to LeftToRight state...")
            new_state = LeftToRight(self.controller, self.limit_l, self.limit_r, self.depths, 0, \
                self.distance, MOVE_TO_START, self.x_discretization, self.tolerance)
            self.controller.change_state(new_state)

    def prepare_exit(self):
        self.controller.get_logger().info(f"Leaving MoveToStart state...")
        self.controller.thruster_pub.publish(Twist())

    def calculate_depths(self):
        return np.linspace(self.start_depth, self.limit_depth, self.depth_discretization)

class LeftToRight(State):
    
    def __init__(self, controller: Node, limit_l, limit_r, depths, depth_level, distance, prev_state,\
        x_discretization=10, tolerance=0.05):

        self.controller = controller

        self.limit_l = limit_l
        self.limit_r = limit_r

        self.depths = depths
        self.distance = distance
        
        self.prev_state = prev_state

        self.x_discretization = x_discretization
        self.depth_level = depth_level

        self.tolerance = tolerance
        
        self.pid_x = PID(1.0, 1.0, 0.0, setpoint=limit_l)
        self.pid_x.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_x.output_limits = (-1.0, 1.0) 

        self.pid_y = PID(1.0, 1.0, 0.0, setpoint=depths[depth_level])
        self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_y.output_limits = (-1.0, 1.0)

        self.pid_z = PID(1.0, 1.0, 0.0, setpoint=distance)
        self.pid_z.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_z.output_limits = (-1.0, 1.0)
        
        self.dscrtz_idx = 0
        self.photos_taken_at_pos = 0
        self.X = np.linspace(self.limit_l, self.limit_r, self.x_discretization)

    def position_callback(self, msg):
        pos_vector = msg.vector
        pub_msg = Twist() 

        ###
        x = self.X[self.dscrtz_idx]
        y = self.depths[self.depth_level]
        z = self.distance
        self.controller.get_logger().info(f"Wanted -> x: {x}, y: {y}, z: {z}")
        ### 

        lower, upper = get_lower_upper(x, self.tolerance)
        self.controller.get_logger().info(f"X Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.x < upper:
            pub_msg.linear.x = self.pid_x(pos_vector.x)
            self.controller.get_logger().info(f"Setting X to {self.pid_x(pos_vector.x)} according to pid.")

        lower, upper = get_lower_upper(y, self.tolerance)
        # self.controller.get_logger().info(f"Y Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.y < upper:
            pub_msg.linear.y = self.pid_y(pos_vector.y)
        #     self.controller.get_logger().info(f"Setting Y to {self.pid_y(pos_vector.y)} according to pid.")

        lower, upper = get_lower_upper(z, self.tolerance)
        # self.controller.get_logger().info(f"Z Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.z < upper:
            pub_msg.linear.z = self.pid_z(pos_vector.z)
        #     self.controller.get_logger().info(f"Setting Z to {self.pid_z(pos_vector.z)} according to pid.")

        self.controller.thruster_pub.publish(pub_msg)

        if pub_msg.linear.x == 0.0 and pub_msg.linear.y == 0.0 and pub_msg.linear.z == 0.0:
            # call biofouling module to take a photo?
            self.controller.get_logger().info(f"Taking photo.")
            self.photos_taken_at_pos += 1
            
            if self.photos_taken_at_pos == 2:
                if self.dscrtz_idx == len(self.X) - 1:
                    self.controller.get_logger().info(f"Switching to Dive state...")
                    new_state = Dive(self.controller, self.limit_l, self.limit_r, self.depths,\
                        self.depth_level + 1, self.distance, LEFT_TO_RIGHT, self.x_discretization, self.tolerance)
                    self.controller.change_state(new_state)
                else:
                    self.dscrtz_idx += 1
                    self.photos_taken_at_pos = 0
                    self.pid_x.setpoint = self.X[self.dscrtz_idx]

    def prepare_exit(self):
        self.controller.get_logger().info(f"Leaving LeftToRight state...")
        self.controller.thruster_pub.publish(Twist())

class RightToLeft(State):
    
    def __init__(self, controller: Node, limit_l, limit_r, depths, depth_level, distance, prev_state,\
        x_discretization=10, tolerance=0.05):

        self.controller = controller

        self.limit_l = limit_l
        self.limit_r = limit_r

        self.depths = depths
        self.distance = distance
        
        self.prev_state = prev_state

        self.x_discretization = x_discretization
        self.depth_level = depth_level

        self.tolerance = tolerance
        
        self.pid_x = PID(1.0, 1.0, 0.0, setpoint=limit_r)
        self.pid_x.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_x.output_limits = (-1.0, 1.0) 

        self.pid_y = PID(1.0, 1.0, 0.0, setpoint=depths[depth_level])
        self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_y.output_limits = (-1.0, 1.0)

        self.pid_z = PID(1.0, 1.0, 0.0, setpoint=distance)
        self.pid_z.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_z.output_limits = (-1.0, 1.0)
        
        self.dscrtz_idx = 0
        self.photos_taken_at_pos = 0
        self.X = np.linspace(self.limit_r, self.limit_l, self.x_discretization)

    def position_callback(self, msg):
        pos_vector = msg.vector
        pub_msg = Twist() 

        ###
        x = self.X[self.dscrtz_idx]
        y = self.depths[self.depth_level]
        z = self.distance
        self.controller.get_logger().info(f"Wanted -> x: {x}, y: {y}, z: {z}")
        ### 

        lower, upper = get_lower_upper(x, self.tolerance)
        # self.controller.get_logger().info(f"X Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.x < upper:
            pub_msg.linear.x = self.pid_x(pos_vector.x)
            # self.controller.get_logger().info(f"Setting X to {self.pid_x(pos_vector.x)} according to pid.")

        lower, upper = get_lower_upper(y, self.tolerance)
        # self.controller.get_logger().info(f"Y Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.y < upper:
            pub_msg.linear.y = self.pid_y(pos_vector.y)
        #     self.controller.get_logger().info(f"Setting Y to {self.pid_y(pos_vector.y)} according to pid.")

        lower, upper = get_lower_upper(z, self.tolerance)
        # self.controller.get_logger().info(f"Z Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.z < upper:
            pub_msg.linear.z = self.pid_z(pos_vector.z)
        #     self.controller.get_logger().info(f"Setting Z to {self.pid_z(pos_vector.z)} according to pid.")

        self.controller.thruster_pub.publish(pub_msg)

        if pub_msg.linear.x == 0.0 and pub_msg.linear.y == 0.0 and pub_msg.linear.z == 0.0:
            # call biofouling module to take a photo?
            self.controller.get_logger().info(f"Taking photo.")
            self.photos_taken_at_pos += 1
            
            if self.photos_taken_at_pos == 2:
                if self.dscrtz_idx == len(self.X) - 1:
                    self.controller.get_logger().info(f"Switching to Dive state...")
                    new_state = Dive(self.controller, self.limit_l, self.limit_r, self.depths,\
                        self.depth_level + 1, self.distance, RIGHT_TO_LEFT, self.x_discretization, self.tolerance)
                    self.controller.change_state(new_state)
                else:
                    self.dscrtz_idx += 1
                    self.photos_taken_at_pos = 0
                    self.pid_x.setpoint = self.X[self.dscrtz_idx]

    def prepare_exit(self):
        self.controller.get_logger().info(f"Leaving RightToLeft state...")
        self.controller.thruster_pub.publish(Twist())

class Dive(State):
    def __init__(self, controller: Node, limit_l, limit_r, depths, depth_level, distance, prev_state,\
        x_discretization=10, tolerance=0.05):

        self.controller = controller   

        self.limit_l = limit_l
        self.limit_r = limit_r

        self.depths = depths
        self.distance = distance
        
        self.prev_state = prev_state

        self.x_discretization = x_discretization
        self.depth_level = depth_level

        self.tolerance = tolerance
        
        if prev_state == LEFT_TO_RIGHT:
            x_setpoint = limit_r
        else:
            x_setpoint = limit_l
        self.pid_x = PID(1.0, 1.0, 0.0, setpoint=x_setpoint)
        self.pid_x.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_x.output_limits = (-1.0, 1.0) 

        try:
            self.pid_y = PID(1.0, 1.0, 0.0, setpoint=depths[depth_level])
            self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
            self.pid_y.output_limits = (-1.0, 1.0)
        except IndexError:
            self.controller.get_logger().info(f"DEALING WITH INDEX ERROR")
            self.pid_y = PID(1.0, 1.0, 0.0, setpoint=depths[depth_level - 1])
            self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
            self.pid_y.output_limits = (-1.0, 1.0)

        self.pid_z = PID(1.0, 1.0, 0.0, setpoint=distance)
        self.pid_z.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_z.output_limits = (-1.0, 1.0)

        self.start_check = 0

    def position_callback(self, msg: Vector3Stamped):
        pub_msg = Twist()
        pos_vector = msg.vector

        ###
        if self.prev_state == LEFT_TO_RIGHT:
            x = self.limit_r
        else:
            x = self.limit_l

        if self.depth_level >= len(self.depths):
            new_state = Resurface(self.controller, 0.0, 0.0, 0.0)
            self.controller.change_state(new_state)
            self.controller.get_logger().info(f"RETURNING BECAUSE OF DEPTH LEVEL FROM DIVE CALLBACK")
            return
        y = self.depths[self.depth_level]

        z = self.distance
        self.controller.get_logger().info(f"Wanted -> x: {x}, y: {y}, z: {z}")
        ### 

        lower, upper = get_lower_upper(x, self.tolerance)
        # self.controller.get_logger().info(f"X Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.x < upper:
            pub_msg.linear.x = self.pid_x(pos_vector.x)
            # self.controller.get_logger().info(f"Setting X to {self.pid_x(pos_vector.x)} according to pid.")

        lower, upper = get_lower_upper(y, self.tolerance)
        # self.controller.get_logger().info(f"Y Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.y < upper:
            pub_msg.linear.y = self.pid_y(pos_vector.y)
        #     self.controller.get_logger().info(f"Setting Y to {self.pid_y(pos_vector.y)} according to pid.")

        lower, upper = get_lower_upper(z, self.tolerance)
        # self.controller.get_logger().info(f"Z Lower: {lower}, Actual: {pos_vector.x}, Upper: {upper}")
        if not lower < pos_vector.z < upper:
            pub_msg.linear.z = self.pid_z(pos_vector.z)
        #     self.controller.get_logger().info(f"Setting Z to {self.pid_z(pos_vector.z)} according to pid.")
        
        self.controller.thruster_pub.publish(pub_msg)

        if pub_msg.linear.x == 0.0 and pub_msg.linear.y == 0.0 and pub_msg.linear.z == 0.0:
            self.start_check += 1
        else:
            self.start_check = 0

        if self.start_check >= 2:
            if self.prev_state == LEFT_TO_RIGHT:
                self.controller.get_logger().info(f"Switching to RightToLeft state...")
                new_state = RightToLeft(self.controller, self.limit_l, self.limit_r, self.depths, self.depth_level, \
                    self.distance, DIVE, self.x_discretization, self.tolerance)
            else:
                self.controller.get_logger().info(f"Switching to LeftToRight state...")
                new_state = LeftToRight(self.controller, self.limit_l, self.limit_r, self.depths, self.depth_level, \
                    self.distance, DIVE, self.x_discretization, self.tolerance)
            self.controller.change_state(new_state)

    def prepare_exit(self):
        self.controller.get_logger().info(f"Leaving Dive state...")
        self.controller.thruster_pub.publish(Twist())

class Resurface(State):
    def __init__(self, controller: Node, x, y, z):
    
        self.controller = controller

        self.pid_x = PID(1.0, 1.0, 0.0, setpoint=x)
        self.pid_x.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_x.output_limits = (-1.0, 1.0) 

        self.pid_y = PID(1.0, 1.0, 0.0, setpoint=y)
        self.pid_y.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_y.output_limits = (-1.0, 1.0)

        self.pid_z = PID(1.0, 1.0, 0.0, setpoint=z)
        self.pid_z.sample_time = 1.0  # Update every 0.01 seconds
        self.pid_z.output_limits = (-1.0, 1.0)

        self.start_check = 0


    def position_callback(self, msg: Vector3Stamped):
        self.controller.get_logger().info(f"Callback from Resurface state...")
        pub_msg = Twist()
        pos_vector = msg.vector

        pub_msg.linear.x = self.pid_x(pos_vector.x)
        pub_msg.linear.y = self.pid_y(pos_vector.y)
        pub_msg.linear.z = self.pid_z(pos_vector.z)
        
        self.controller.thruster_pub.publish(pub_msg)

    def prepare_exit(self):
        self.controller.thruster_pub.publish(Twist()) 

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
            self.declare_parameter(SLOW_GAIN_TOPIC_PARAM, '/placeholder_slow_gain')
            self.declare_parameter(START_POSITION_PARAM)
            self.declare_parameter(LEFT_LIMIT_PARAM)
            self.declare_parameter(RIGHT_LIMIT_PARAM)
            self.declare_parameter(DEPTH_LIMIT_PARAM)
            self.declare_parameter(X_DSCRTZ_PARAM)
            self.declare_parameter(DEPTH_DSCRTZ_PARAM)
            self.declare_parameter(TOLERANCE_PARAM)
            self.declare_parameter(SLOW_GAIN_REF_PARAM)
            self.declare_parameter(MAX_THRUST_POS_PARAM)
            self.declare_parameter(MAX_THRUST_NEG_PARAM)

        def initialize_subscribers(self):
            self.create_subscription(Vector3Stamped, 
                self.get_parameter(POSITION_TOPIC_PARAM).value, self.position_callback, 10)
            
            # self.create_subscription(Float64Custom, 
            #     self.get_parameter(MONOCULAR_DISTANCE_TOPIC_PARAM).value, self.monocular_callback, 10)

            # self.create_subscription(Pose,
            #     self.get_parameter(POSE_TOPIC_PARAM).value, self.pose_callback, 10)

        def initialize_publishers(self):
            self.thruster_pub = self.create_publisher(Twist, self.get_parameter(THRUSTER_TOPIC_PARAM).value, 10)
            self.slow_gain_pub = self.create_publisher(Float32, self.get_parameter(SLOW_GAIN_TOPIC_PARAM).value, 10)

        def position_callback(self, msg):
            self.current_state.position_callback(msg)
        
        def monocular_callback(self, msg):
            self.get_logger().info(f"Received distance from monocular topic: {msg.data}")

        def pose_callback(self, msg):
            q = msg.orientation
            yaw = arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
            pitch = arcsin(-2.0*(q.x*q.z - q.w*q.y))
            roll = arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

            self.get_logger().info(f"yaw: {round(yaw/pi * 180, 2)}")
            self.get_logger().info(f"pitch: {round(pitch/pi * 180, 2)}")
            self.get_logger().info(f"roll: {round(roll/pi * 180, 2)}")
            self.get_logger().info(f" ")

        def initialize_class_objects(self):
            self.set_slow_gain(1.0)
            start_vector = self.get_parameter(START_POSITION_PARAM).get_parameter_value().double_array_value
            limit_l = self.get_parameter(LEFT_LIMIT_PARAM).get_parameter_value().double_value
            limit_r = self.get_parameter(RIGHT_LIMIT_PARAM).get_parameter_value().double_value
            limit_depth = self.get_parameter(DEPTH_LIMIT_PARAM).get_parameter_value().double_value
            x_discretization = self.get_parameter(X_DSCRTZ_PARAM).get_parameter_value().integer_value
            depth_discretization = self.get_parameter(DEPTH_DSCRTZ_PARAM).get_parameter_value().integer_value
            tolerance = self.get_parameter(TOLERANCE_PARAM).get_parameter_value().double_value

            self.current_state = MoveToStart(self, limit_l, limit_r, start_vector[1], limit_depth,\
                start_vector[2], x_discretization, depth_discretization, tolerance)

        def change_state(self, new_state):
            self.current_state.prepare_exit()
            self.current_state = new_state
            self.get_logger().info(f"AFTER SWITCH: {self.current_state}")

        def set_slow_gain(self, value):
            msg = Float32()
            msg.data = value
            self.slow_gain_pub.publish(msg)

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