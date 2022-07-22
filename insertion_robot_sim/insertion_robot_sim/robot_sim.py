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
        # Subscribers

        # self.m_sub_AxAbsCommandX = self.create_subscription(
        #     Float32,
        #     'stage/axis/command/absolute/x',
        #     lambda msg: self.topic_callbackAxisCommand(msg,0,True),
        #     10)
        # self.m_sub_AxAbsCommandY = self.create_subscription(
        #     Float32,
        #     'stage/axis/command/absolute/y',
        #     lambda msg: self.topic_callbackAxisCommand(msg,1,True),
        #     10)
        # self.m_sub_AxAbsCommandz = self.create_subscription(
        #     Float32,
        #     'stage/axis/command/absolute/z',
        #     lambda msg: self.topic_callbackAxisCommand(msg,2,True),
        #     10)
        # self.m_sub_AxAbsCommandLS = self.create_subscription(
        #     Float32,
        #     'stage/axis/command/absolute/linear_stage',
        #     lambda msg: self.topic_callbackAxisCommand(msg,3,True),
        #     10)

        # Publishers
        # self.m_pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.x_axis_abs_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/absolute/x', 10)
        self.y_axis_abs_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/absolute/y', 10)
        self.z_axis_abs_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/absolute/z', 10)
        self.ls_axis_abs_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/absolute/linear_stage', 10)
        self.x_axis_rel_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/relative/x', 10)
        self.y_axis_rel_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/relative/y', 10)
        self.z_axis_rel_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/relative/z', 10)
        self.ls_axis_rel_cmd_pub = self.create_publisher(Float32, 'stage/axis/command/relative/linear_stage', 10)


        ### For dynamic reconfiguration GUI of insertion robot control
        self.abort_cli = self.create_client(Trigger, "/stage/abort")
        self.x_toggle_cli = self.create_client(Trigger, "/stage/axis/state/toggle/x")
        self.y_toggle_cli = self.create_client(Trigger, "/stage/axis/state/toggle/y")
        self.z_toggle_cli = self.create_client(Trigger, "/stage/axis/state/toggle/z")
        self.ls_toggle_cli = self.create_client(Trigger, "/stage/axis/state/toggle/linear_stage")
        self.x_zero_cli = self.create_client(Trigger, "/stage/axis/zero/x")
        self.y_zero_cli = self.create_client(Trigger, "/stage/axis/zero/y")
        self.z_zero_cli = self.create_client(Trigger, "/stage/axis/zero/z")
        self.ls_zero_cli = self.create_client(Trigger, "/stage/axis/zero/linear_stage")
        self.req = Trigger.Request()


        # Parameters description
        abort_descriptor = ParameterDescriptor(description='Aborting all movements')

        x_axis_toggle_descriptor = ParameterDescriptor(description='Toggle x-axis')
        y_axis_toggle_descriptor = ParameterDescriptor(description='Toggle y-axis')
        z_axis_toggle_descriptor = ParameterDescriptor(description='Toggle z-axis')
        ls_axis_toggle_descriptor = ParameterDescriptor(description='Toggle linear stage axis')

        x_axis_zero_descriptor = ParameterDescriptor(description='Zero x-axis')
        y_axis_zero_descriptor = ParameterDescriptor(description='Zero y-axis')
        z_axis_zero_descriptor = ParameterDescriptor(description='Zero z-axis')
        ls_axis_zero_descriptor = ParameterDescriptor(description='Zero linear stage axis')

        x_axis_command_range = FloatingPointRange(from_value=-100.0, to_value=100.0, step=0.01)
        y_axis_command_range = FloatingPointRange(from_value=-100.0, to_value=100.0, step=0.01)
        z_axis_command_range = FloatingPointRange(from_value=-100.0, to_value=100.0, step=0.01)
        ls_axis_command_range = FloatingPointRange(from_value=-100.0, to_value=100.0, step=0.01)
        x_axis_cmd_descriptor = ParameterDescriptor(description='Position command for x-axis', floating_point_range=[x_axis_command_range])
        y_axis_cmd_descriptor = ParameterDescriptor(description='Position command for y-axis', floating_point_range=[y_axis_command_range])
        z_axis_cmd_descriptor = ParameterDescriptor(description='Position command for z-axis', floating_point_range=[z_axis_command_range])
        ls_axis_cmd_descriptor = ParameterDescriptor(description='Position command for linear stage axis', floating_point_range=[ls_axis_command_range])




        # Parameters declaration
        self.declare_parameter("abort", False, abort_descriptor)
        self.declare_parameter("x_axis_toggle", False, x_axis_toggle_descriptor)
        self.declare_parameter("y_axis_toggle", False, y_axis_toggle_descriptor)
        self.declare_parameter("z_axis_toggle", False, z_axis_toggle_descriptor)
        self.declare_parameter("ls_axis_toggle", False, ls_axis_toggle_descriptor)
        self.declare_parameter("x_axis_zero", False, x_axis_zero_descriptor)
        self.declare_parameter("y_axis_zero", False, y_axis_zero_descriptor)
        self.declare_parameter("z_axis_zero", False, z_axis_zero_descriptor)
        self.declare_parameter("ls_axis_zero", False, ls_axis_zero_descriptor)
        self.declare_parameter("x_axis_abs_cmd", 0.0, x_axis_cmd_descriptor)
        self.declare_parameter("y_axis_abs_cmd", 0.0, y_axis_cmd_descriptor)
        self.declare_parameter("z_axis_abs_cmd", 0.0, z_axis_cmd_descriptor)
        self.declare_parameter("ls_axis_abs_cmd", 0.0, ls_axis_cmd_descriptor)
        self.declare_parameter("x_axis_rel_cmd", 0.0, x_axis_cmd_descriptor)
        self.declare_parameter("y_axis_rel_cmd", 0.0, y_axis_cmd_descriptor)
        self.declare_parameter("z_axis_rel_cmd", 0.0, z_axis_cmd_descriptor)
        self.declare_parameter("ls_axis_rel_cmd", 0.0, ls_axis_cmd_descriptor)



        # Save parameters in class properties
        # self.abort = self.get_parameter('abort').value
        # self.x_axis_toggle = self.get_parameter('x_axis_toggle').value
        # self.y_axis_toggle = self.get_parameter('y_axis_toggle').value
        # self.z_axis_toggle = self.get_parameter('z_axis_toggle').value
        # self.ls_axis_toggle = self.get_parameter('ls_axis_toggle').value
        # self.x_axis_zero = self.get_parameter('x_axis_zero').value
        # self.y_axis_zero = self.get_parameter('y_axis_zero').value
        # self.z_axis_zero = self.get_parameter('z_axis_zero').value
        # self.ls_axis_zero = self.get_parameter('ls_axis_zero').value
        # self.x_axis_abs_cmd = self.get_parameter('x_axis_abs_cmd').value
        # self.y_axis_abs_cmd = self.get_parameter('y_axis_abs_cmd').value
        # self.z_axis_abs_cmd = self.get_parameter('z_axis_abs_cmd').value
        # self.ls_axis_abs_cmd = self.get_parameter('ls_axis_abs_cmd').value
        # self.x_axis_rel_cmd = self.get_parameter('x_axis_rel_cmd').value
        # self.y_axis_rel_cmd = self.get_parameter('y_axis_rel_cmd').value
        # self.z_axis_rel_cmd = self.get_parameter('z_axis_rel_cmd').value
        # self.ls_axis_rel_cmd = self.get_parameter('ls_axis_rel_cmd').value

        # Assign callback function for parameters
        self.add_on_set_parameters_callback(self.parameters_callback)

    def send_request(self, axis):
        if axis=="x":
            self.future = self.x_toggle_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="y":
            self.future = self.y_toggle_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="z":
            self.future = self.z_toggle_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="ls":
            self.future = self.ls_toggle_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def send_request_zero(self, axis):
        if axis=="x":
            self.future = self.x_zero_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="y":
            self.future = self.y_zero_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="z":
            self.future = self.z_zero_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        if axis=="ls":
            self.future = self.ls_zero_cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def send_request_abort(self):
        self.future = self.abort_cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def topic_callbackAxisCommand(self, msg, axis, is_absolute):
        self.get_logger().info('TEST axis ID: %i; cmd value %f' % (axis, msg.data))

        self.m_pub_js.publish()


    def parameters_callback(self, params):
        for param in params:
            #!!! Should be a button for movement abortion, not toggle
            if param.name == "abort":
                #call service named "/axis/state/toggle/x"
                #TO:DO: get current axes state and renew position values
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request_abort()
                else:
                    return SetParametersResult(successful=False)


            if param.name == "x_axis_toggle":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request("x")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "y_axis_toggle":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request("y")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "z_axis_toggle":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request("z")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "ls_axis_toggle":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request("ls")
                else:
                    return SetParametersResult(successful=False)

            #!!! Should be a button for zeroing, not toggle
            if param.name == "x_axis_zero":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request_zero("x")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "y_axis_zero":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request_zero("y")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "z_axis_zero":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request_zero("z")
                else:
                    return SetParametersResult(successful=False)
            if param.name == "ls_axis_zero":
                #call service named "/axis/state/toggle/x"
                if param.type_ == Parameter.Type.BOOL:
                    self.send_request_zero("ls")
                else:
                    return SetParametersResult(successful=False)

            if param.name == "x_axis_abs_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.x_axis_abs_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "y_axis_abs_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.y_axis_abs_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "z_axis_abs_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.z_axis_abs_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)
            
            if param.name == "ls_axis_abs_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.ls_axis_abs_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "x_axis_rel_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.x_axis_rel_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "y_axis_rel_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.y_axis_rel_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "z_axis_rel_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.z_axis_rel_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)

            if param.name == "ls_axis_rel_cmd":
                #publish to the corresponding topic
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    msg = Float32()
                    msg.data=param.value
                    self.ls_axis_rel_cmd_pub.publish(msg)
                else:
                    return SetParametersResult(successful=False)
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