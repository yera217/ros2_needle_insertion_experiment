import numpy as np
from functools import partial

# ROS2 packages
import rclpy
from rclpy.node import Node
from rclpy import Parameter

# messages
from geometry_msgs.msg import PoseArray, Point, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class RobotSim(Node):

    def __init__(self, name='robot_sim'):
        super().__init__(name)

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

    def topic_callbackAxisCommand(self, msg, axis, is_absolute):
        self.get_logger().info('TEST axis ID: %i; cmd value %f' % (axis, msg.data))

        self.m_pub_js.publish()







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