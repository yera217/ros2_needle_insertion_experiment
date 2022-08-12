from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction

from .publisher_tree_model import PublisherTreeModel
from rqt_py_common.message_tree_widget import MessageTreeWidget
from rqt_py_common.item_delegates import SpinBoxDelegate


class PublisherTreeWidget(MessageTreeWidget):
    remove_publisher = Signal(int)
    publish_once = Signal(int)

    def __init__(self, parent=None):
        super(PublisherTreeWidget, self).__init__(parent)
        self.setModel(PublisherTreeModel(self))
        self._action_remove_publisher = QAction(
            QIcon.fromTheme('list-remove'), 'Remove Selected', self)
        self._action_remove_publisher.triggered[bool].connect(self._handle_action_remove_publisher)
        self._action_publish_once = QAction(
            QIcon.fromTheme('media-playback-start'), 'Publish Selected Once', self)
        self._action_publish_once.triggered[bool].connect(self._handle_action_publish_once)
        self.setItemDelegateForColumn(self.model()._column_index['rate'],
                                      SpinBoxDelegate(min_value=0, max_value=1000000, decimals=2))

    @Slot()
    def remove_selected_publishers(self):
        publisher_ids = self.model().get_publisher_ids(self.selectedIndexes())
        for publisher_id in publisher_ids:
            self.remove_publisher.emit(publisher_id)
        self.model().remove_items_with_parents(self.selectedIndexes())

    def _context_menu_add_actions(self, menu, pos):
        if self.selectionModel().hasSelection():
            menu.addAction(self._action_remove_publisher)
            menu.addAction(self._action_publish_once)
        # let super class add actions
        super(PublisherTreeWidget, self)._context_menu_add_actions(menu, pos)

    def _handle_action_remove_publisher(self, checked):
        self.remove_selected_publishers()

    def _handle_action_publish_once(self, checked):
        for publisher_id in self.model().get_publisher_ids(self.selectedIndexes()):
            self.publish_once.emit(publisher_id)