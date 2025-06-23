from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QListView, QShortcut
from python_qt_binding.QtWidgets import QStyledItemDelegate, QStyleOptionViewItem
from python_qt_binding.QtGui import QKeySequence, QTextDocument, QTextOption

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


class ChatDelegate(QStyledItemDelegate):
    def __init__(self, parent=None):
        super(ChatDelegate, self).__init__(parent)

    def _doc(self, options):
        doc = QTextDocument()
        doc.setTextWidth(options.rect.width() - 10)
        doc.setDefaultFont(options.font)
        doc.setDefaultTextOption(QTextOption(options.displayAlignment))
        doc.setHtml(options.text)
        return doc

    def initStyleOption(self, options, index):
        super(ChatDelegate, self).initStyleOption(options, index)
        if index.data(QtCore.Qt.UserRole):
            options.displayAlignment = Qt.AlignRight
        else:
            options.displayAlignment = Qt.AlignLeft

    def sizeHint(self, option, index):
        options = QStyleOptionViewItem(option)
        self.initStyleOption(options, index)
        return self._doc(options).size().toSize()

    def paint(self, painter, option, index):
        options = QStyleOptionViewItem(option)
        self.initStyleOption(options, index)
        doc = self._doc(options)

        painter.save()

        options.text = ""
        # options.widget.style().drawControl(QStyle.CE_ItemViewItem, options, painter)
        painter.translate(
            options.rect.left(), options.rect.top()
        )  # translate to the top-left corner of item
        if options.displayAlignment == Qt.AlignLeft:
            painter.translate(10, 0)  # add padding to the left
        doc.drawContents(painter)

        painter.restore()


class ChatListView(QListView):
    def __init__(self, parent=None):
        super(ChatListView, self).__init__(parent)
        self.setItemDelegate(ChatDelegate(self))
        self.setUniformItemSizes(False)
        self.setResizeMode(QListView.Adjust)

    def resizeEvent(self, event):
        super(ChatListView, self).resizeEvent(event)
        self.model().layoutChanged.emit()
