from __future__ import division
import array
import math
import random
import time

from python_qt_binding.QtCore import Slot, QSignalMapper, QTimer, qWarning

from rclpy.exceptions import InvalidTopicNameException
from rclpy.qos import QoSProfile

from rosidl_runtime_py.utilities import get_message

from rqt_gui_py.plugin import Plugin
from rqt_py_common.topic_helpers import get_slot_type

from .robot_control_widget import RobotControlWidget

_list_types = [list, tuple, array.array]
try:
    import numpy
    _list_types.append(numpy.ndarray)
except ImportError:
    pass

_numeric_types = [int, float]
try:
    import numpy
    _numeric_types += [
        numpy.int8, numpy.int16, numpy.int32, numpy.int64,
        numpy.float16, numpy.float32, numpy.float64
    ]
except ImportError:
    pass


class RobotControl(Plugin):

    def __init__(self, context):
        super(RobotControl, self).__init__(context)
        self.setObjectName('RobotControl')

        self._node = context.node

        # create widget
        self._widget = RobotControlWidget(self._node)
        # self._widget.publish_once.connect(self.publish_once)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))


        # self._timeout_mapper = QSignalMapper(self)
        # self._timeout_mapper.mapped[int].connect(self.publish_once)

        # add our self to the main window
        context.add_widget(self._widget)


    # @Slot(int)
    # def publish_once(self, publisher_id):
    #     publisher_info = self._publishers.get(publisher_id, None)
    #     if publisher_info is not None:
    #         publisher_info['counter'] += 1
    #         self._fill_message_slots(
    #             publisher_info['message_instance'],
    #             publisher_info['topic_name'],
    #             publisher_info['expressions'],
    #             publisher_info['counter'])
    #         publisher_info['publisher'].publish(publisher_info['message_instance'])
