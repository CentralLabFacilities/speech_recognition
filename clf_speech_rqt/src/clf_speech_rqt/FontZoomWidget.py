from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QShortcut
from python_qt_binding.QtGui import QKeySequence

class FontZoomWidget(QWidget):
    """Widget allowing font zooming via mouse wheel and keyboard shortcuts."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.min_font_size = 6
        self.max_font_size = 32
        shortcut = QShortcut(QKeySequence("Ctrl++"), self)
        shortcut.activated.connect(self.increase_font_size)
        shortcut = QShortcut(QKeySequence("Ctrl+-"), self)
        shortcut.activated.connect(self.decrease_font_size)

    def wheelEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            delta = event.angleDelta().y() / 120.0  # standard step for wheel events
            self.set_font_size(self.font().pointSize() + int(delta))
            event.accept()
        else:
            super().wheelEvent(event)

    @QtCore.pyqtSlot()
    def increase_font_size(self):
        self.set_font_size(self.font().pointSize() + 1)

    @QtCore.pyqtSlot()
    def decrease_font_size(self):
        self.set_font_size(self.font().pointSize() - 1)

    @QtCore.pyqtSlot(int)
    def set_font_size(self, new_font_size):
        if new_font_size < self.min_font_size:
            new_font_size = self.min_font_size
        elif new_font_size > self.max_font_size:
            new_font_size = self.max_font_size
        if new_font_size == self.font().pointSize():
            return

        font = self.font()
        font.setPointSize(new_font_size)
        self.setFont(font)