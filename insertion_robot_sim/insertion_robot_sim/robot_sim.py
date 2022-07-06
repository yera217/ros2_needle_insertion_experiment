import numpy as np
from functools import partial

# ROS2 packages
import rclpy
from rclpy.node import Node
from rclpy import Parameter

# messages
from geometry_msgs.msg import PoseArray, Point, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

class RobotSim(Node):

    def __init__(self, name='robot_sim'):
        super().__init__(name)

        ### For simulation
        # subscribers
        self.m_sub_AxAbsCommandX = self.create_subscription(
            Float32,
            'stage/axis/command/absolute/x',
            lambda msg: self.topic_callbackAxisCommand(msg,0,True),
            10)
        self.m_sub_AxAbsCommandY = self.create_subscription(
            Float32,
            'stage/axis/command/absolute/y',
            lambda msg: self.topic_callbackAxisCommand(msg,1,True),
            10)
        self.m_sub_AxAbsCommandz = self.create_subscription(
            Float32,
            'stage/axis/command/absolute/z',
            lambda msg: self.topic_callbackAxisCommand(msg,2,True),
            10)
        self.m_sub_AxAbsCommandLS = self.create_subscription(
            Float32,
            'stage/axis/command/absolute/linear_stage',
            lambda msg: self.topic_callbackAxisCommand(msg,3,True),
            10)

        #publishers
        self.m_pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.x_axis_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/absolute/x', 10)




        ### For dynamic reconfiguration GUI of insertion robot control

        self.cli = self.create_client(Trigger, "/axis/state/toggle/x")
        self.req = Trigger.Request()



        # Parameters description
        x_axis_toggle_descriptor = ParameterDescriptor(description='Toggle x-axis')
        y_axis_toggle_descriptor = ParameterDescriptor(description='Toggle y-axis')
        z_axis_toggle_descriptor = ParameterDescriptor(description='Toggle z-axis')
        ls_axis_toggle_descriptor = ParameterDescriptor(description='Toggle linear stage axis')


        x_axis_command_range = FloatingPointRange(from_value=0.0, to_value=100.0, step=0.01)
        y_axis_command_range = FloatingPointRange(from_value=0.0, to_value=100.0, step=0.01)
        z_axis_command_range = FloatingPointRange(from_value=0.0, to_value=100.0, step=0.01)
        ls_axis_command_range = FloatingPointRange(from_value=0.0, to_value=100.0, step=0.01)
        x_axis_cmd_descriptor = ParameterDescriptor(description='Position command for x-axis', floating_point_range=[x_axis_command_range])
        y_axis_cmd_descriptor = ParameterDescriptor(description='Position command for y-axis', floating_point_range=[y_axis_command_range])
        z_axis_cmd_descriptor = ParameterDescriptor(description='Position command for z-axis', floating_point_range=[z_axis_command_range])
        ls_axis_cmd_descriptor = ParameterDescriptor(description='Position command for linear stage axis', floating_point_range=[ls_axis_command_range])




        # Parameters declaration
        self.declare_parameter("x_axis_toggle", False, x_axis_toggle_descriptor)
        self.declare_parameter("y_axis_toggle", False, y_axis_toggle_descriptor)
        self.declare_parameter("z_axis_toggle", False, z_axis_toggle_descriptor)
        self.declare_parameter("ls_axis_toggle", False, ls_axis_toggle_descriptor)
        self.declare_parameter("x_axis_cmd", 0.0, x_axis_cmd_descriptor)
        self.declare_parameter("y_axis_cmd", 0.0, y_axis_cmd_descriptor)
        self.declare_parameter("z_axis_cmd", 0.0, z_axis_cmd_descriptor)
        self.declare_parameter("ls_axis_cmd", 0.0, ls_axis_cmd_descriptor)



        # Save parameters in class properties
        self.x_axis_toggle = self.get_parameter('x_axis_toggle').value
        self.y_axis_toggle = self.get_parameter('y_axis_toggle').value
        self.z_axis_toggle = self.get_parameter('z_axis_toggle').value
        self.ls_axis_toggle = self.get_parameter('ls_axis_toggle').value
        self.x_axis_cmd = self.get_parameter('x_axis_cmd').value
        self.y_axis_cmd = self.get_parameter('y_axis_cmd').value
        self.z_axis_cmd = self.get_parameter('z_axis_cmd').value
        self.ls_axis_cmd = self.get_parameter('ls_axis_cmd').value

        # Assign callback function for parameters
        self.add_on_set_parameters_callback(self.parameters_callback)

    def send_request(self):
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def topic_callbackAxisCommand(self, msg, axis, is_absolute):
        self.get_logger().info('TEST axis ID: %i; cmd value %f' % (axis, msg.data))

        self.m_pub_js.publish()


    def parameters_callback(self, params):
        for param in params:
            # print(vars(param))
            if param.name == "x_axis_toggle":
                self.x_axis_toggle = param.value
                #call service named "/axis/state/toggle/x"
                self.send_request()

            if param.name == "x_axis_cmd":
                self.x_axis_cmd = param.value
                #publish to the corresponding topic
                msg = Float32()
                msg.data=param.value
                self.x_axis_cmd_pub.publish(msg)

        return SetParametersResult(successful=True)




def main( args=None ):
    rclpy.init( args=args )

    simrobot_node = RobotSim()

    try:
        rclpy.spin( simrobot_node )

    except KeyboardInterrupt:
        pass

    # clean-up
    simrobot_node.destroy_node()
    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if __main__