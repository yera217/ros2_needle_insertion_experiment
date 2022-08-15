from __future__ import division
import os
from posixpath import isabs

from ament_index_python import get_resource, has_resource

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot, qWarning
from python_qt_binding.QtWidgets import QWidget

from qt_gui.ros_package_helper import get_package_path

from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py.utilities import get_message

from rqt_py_common.message_helpers import get_service_class, get_message_class, SRV_MODE
from rqt_py_common.topic_helpers import is_primitive_type, get_type_class

import rclpy
from rclpy.qos import QoSProfile


from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from std_msgs.msg import Bool



# main class inherits from the ui window class
class RobotControlWidget(QWidget):
    # publish_once = Signal(int)

    def __init__(self, node, parent=None):
        super(RobotControlWidget, self).__init__(parent)
        self._node = node

        package_path = get_package_path('insertion_robot_rqt')
        ui_file = os.path.join(package_path, 'share', 'insertion_robot_rqt', 'resource', 'RobotControl.ui')
        loadUi(ui_file, self)



        # "Initialize" and "State" functionality implementation
        self.pushButton_toggleY.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/y"))
        self.pushButton_toggleZ.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/z"))
        self.pushButton_toggleLS.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/state/toggle/linear_stage"))
        self.pushButton_toggleAll.clicked.connect(self.pushButton_toggleAll_event)

        self.pushButton_zeroY.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/y"))
        self.pushButton_zeroZ.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/z"))
        self.pushButton_zeroLS.clicked.connect(lambda x: self.pushButton_srv_event("/stage/axis/zero/linear_stage"))
        self.pushButton_zeroAll.clicked.connect(self.pushButton_zeroAll_event)


        self._node.create_subscription(Float32, "/stage/axis/position/y", lambda msg: self.axis_position_callback(msg, "y"), 10)
        self._node.create_subscription(Float32, "/stage/axis/position/z", lambda msg: self.axis_position_callback(msg, "z"), 10 )
        self._node.create_subscription(Float32, "/stage/axis/position/linear_stage", lambda msg: self.axis_position_callback(msg, "linear_stage"), 10 )

        self._node.create_subscription(Bool, "/stage/axis/state/on/y", lambda msg: self.axis_state_callback(msg, "y"), 10)
        self._node.create_subscription(Bool, "/stage/axis/state/on/z", lambda msg: self.axis_state_callback(msg, "z"), 10)
        self._node.create_subscription(Bool, "/stage/axis/state/on/linear_stage", lambda msg: self.axis_state_callback(msg, "linear_stage"), 10)

        # "Move" functionality implementation
        self.label_relMoveY_min.setText(str(self.horizontalSlider_relMoveY.minimum()))
        self.label_relMoveY_max.setText(str(self.horizontalSlider_relMoveY.maximum()))
        self.label_relMoveZ_min.setText(str(self.horizontalSlider_relMoveZ.minimum()))
        self.label_relMoveZ_max.setText(str(self.horizontalSlider_relMoveZ.maximum()))
        self.label_relMoveLS_min.setText(str(self.horizontalSlider_relMoveLS.minimum()))
        self.label_relMoveLS_max.setText(str(self.horizontalSlider_relMoveLS.maximum()))
        self.label_absMoveY_min.setText(str(self.horizontalSlider_absMoveY.minimum()))
        self.label_absMoveY_max.setText(str(self.horizontalSlider_absMoveY.maximum()))
        self.label_absMoveZ_min.setText(str(self.horizontalSlider_absMoveZ.minimum()))
        self.label_absMoveZ_max.setText(str(self.horizontalSlider_absMoveZ.maximum()))
        self.label_absMoveLS_min.setText(str(self.horizontalSlider_absMoveLS.minimum()))
        self.label_absMoveLS_max.setText(str(self.horizontalSlider_absMoveLS.maximum()))

        self.lineedit_relMoveY.setInputMask("####")
        self.lineedit_relMoveZ.setInputMask("####")
        self.lineedit_relMoveLS.setInputMask("####")
        self.lineedit_absMoveY.setInputMask("####")
        self.lineedit_absMoveZ.setInputMask("####")
        self.lineedit_absMoveLS.setInputMask("####")

        self.horizontalSlider_relMoveY.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_relMoveY.value(), self.lineedit_relMoveY))
        self.horizontalSlider_relMoveZ.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_relMoveZ.value(), self.lineedit_relMoveZ))
        self.horizontalSlider_relMoveLS.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_relMoveLS.value(), self.lineedit_relMoveLS))
        self.horizontalSlider_absMoveY.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_absMoveY.value(), self.lineedit_absMoveY))
        self.horizontalSlider_absMoveZ.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_absMoveZ.value(), self.lineedit_absMoveZ))
        self.horizontalSlider_absMoveLS.valueChanged.connect(lambda x: self.lineedit_move_event(self.horizontalSlider_absMoveLS.value(), self.lineedit_absMoveLS))



        self.pushButton_relMoveY.clicked.connect(lambda x: self.pushButton_move_event("y", isAbsolute=False))
        self.pushButton_relMoveZ.clicked.connect(lambda x: self.pushButton_move_event("z", isAbsolute=False))
        self.pushButton_relMoveLS.clicked.connect(lambda x: self.pushButton_move_event("linear_stage", isAbsolute=False))

        self.pushButton_absMoveY.clicked.connect(lambda x: self.pushButton_move_event("y", isAbsolute=True))
        self.pushButton_absMoveZ.clicked.connect(lambda x: self.pushButton_move_event("z", isAbsolute=True))
        self.pushButton_absMoveLS.clicked.connect(lambda x: self.pushButton_move_event("linear_stage", isAbsolute=True))

        self.pushButton_abort.clicked.connect(lambda x: self.pushButton_srv_event("/stage/abort"))

    def axis_position_callback(self, msg, axisName):
        if (axisName=="y"):
            self.lcdNumber_Y.display(msg.data)
        if (axisName=="z"):
            self.lcdNumber_Z.display(msg.data)
        if (axisName=="linear_stage"):
            self.lcdNumber_LS.display(msg.data)

    def axis_state_callback(self, msg, axisName):
        if (axisName=="y"):
            self.label_stateY.setText("ON" if msg.data else "OFF")
            self.label_stateY.setStyleSheet("background-color: green; border: 1px solid black;" if msg.data else "background-color: red; border: 1px solid black;")
        if (axisName=="z"):
            self.label_stateZ.setText("ON" if msg.data else "OFF")
            self.label_stateZ.setStyleSheet("background-color: green; border: 1px solid black;" if msg.data else "background-color: red; border: 1px solid black;")
        if (axisName=="linear_stage"):
            self.label_stateLS.setText("ON" if msg.data else "OFF")
            self.label_stateLS.setStyleSheet("background-color: green; border: 1px solid black;" if msg.data else "background-color: red; border: 1px solid black;")

    def lineedit_move_event(self, value, lineEdit):
        lineEdit.setText(str(value))

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





    def pushButton_move_event(self, axisName, isAbsolute="false"):
        
        topicName="stage/axis/command/"
        if (isAbsolute):
            topicName+="absolute/"
        else:
            topicName+="relative/"

        pub = self._node.create_publisher(
            Float32, topicName + axisName, qos_profile=QoSProfile(depth=10))
        msg = Float32()
        
        if (axisName=="y"):
            if (isAbsolute):
                lineEdit=self.lineedit_absMoveY
            else:
                lineEdit=self.lineedit_relMoveY
        elif (axisName=="z"):
            if (isAbsolute):
                lineEdit=self.lineedit_absMoveZ
            else:
                lineEdit=self.lineedit_relMoveZ
        elif (axisName=="linear_stage"):
            if (isAbsolute):
                lineEdit=self.lineedit_absMoveLS
            else:
                lineEdit=self.lineedit_relMoveLS
        
        
        msg.data = float(lineEdit.text())
        pub.publish(msg)
