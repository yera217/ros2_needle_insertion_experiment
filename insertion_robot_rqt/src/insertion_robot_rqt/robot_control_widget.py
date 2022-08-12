from __future__ import division
import os

from ament_index_python import get_resource, has_resource

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot, qWarning
from python_qt_binding.QtWidgets import QWidget

from qt_gui.ros_package_helper import get_package_path

from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py.utilities import get_message

from rqt_py_common.message_helpers import get_service_class, get_message_class, SRV_MODE
from rqt_py_common.topic_helpers import is_primitive_type, get_type_class

from rclpy.qos import QoSProfile


from std_srvs.srv import Trigger
from std_msgs.msg import Float32



# main class inherits from the ui window class
class RobotControlWidget(QWidget):
    publish_once = Signal(int)

    def __init__(self, node, parent=None):
        super(RobotControlWidget, self).__init__(parent)
        self._node = node

        package_path = get_package_path('insertion_robot_rqt')
        ui_file = os.path.join(package_path, 'share', 'insertion_robot_rqt', 'resource', 'RobotControl.ui')
        loadUi(ui_file, self)

        self.pushButton_toggleY.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/y"))
        self.pushButton_toggleZ.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/z"))
        self.pushButton_toggleLS.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/linear_stage"))
        self.pushButton_toggleAll.clicked.connect(self.pushButton_toggleAll_event)

        self.pushButton_zeroY.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/y"))
        self.pushButton_zeroZ.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/z"))
        self.pushButton_zeroLS.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/linear_stage"))
        self.pushButton_zeroAll.clicked.connect(self.pushButton_zeroAll_event)

        self.pushButton_abort.clicked.connect(lambda x: self.pushButton_srv_event("/stage/abort"))

        self.pushButton_relMoveY.clicked.connect(lambda x: self.pushButton_move_event("stage/axis/command/relative/y"))




    def pushButton_srv_event(self, srvName):
        current_services = dict(self._node.get_service_names_and_types())
        if srvName not in current_services:
            qWarning('Service {} is no longer available. Refresh the list of services'.format(
                     srvName))
            return

        request = Trigger.Request()

        cli = self._node.create_client(
            Trigger, srvName)

        future = cli.call_async(request)
        while rclpy.ok() and not future.done():
            pass

        self._node.destroy_client(cli)

    def pushButton_toggleAll_event(self):
        self.pushButton_srv_event("/stage/axis/state/toggle/y")
        self.pushButton_srv_event("/stage/axis/state/toggle/z")
        self.pushButton_srv_event("/stage/axis/state/toggle/linear_stage")

    def pushButton_zeroAll_event(self):
        self.pushButton_srv_event("/stage/axis/zero/y")
        self.pushButton_srv_event("/stage/axis/zero/z")
        self.pushButton_srv_event("/stage/axis/zero/linear_stage")





    def pushButton_move_event(self, topicName):
        pub = self._node.create_publisher(
            Float32, topicName, qos_profile=QoSProfile(depth=10))
        msg = Float32()
        msg.data = float(self.horizontalSlider_relMoveY.value())
        print(msg.data)
        pub.publish(msg)
