from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from clf_speech_msgs.msg import NLU


class UserFilter(QtCore.QSortFilterProxyModel):
    def __init__(self):
        super(UserFilter, self).__init__()
        self.enabled = False

    def filter(self, enable):
        self.enabled = enable
        self.invalidate()

    def filterAcceptsRow(self, sourceRow, sourceParent):
        if self.enabled:
            index = self.sourceModel().index(sourceRow, 0, sourceParent)
            data = self.sourceModel().itemData(index)
            # print(f"{data}")
            return data[256]
        return True


class NLUTableModel(QtCore.QAbstractTableModel):
    def __init__(self):
        super(NLUTableModel, self).__init__()
        self._data = []

    def add_nlu(self, nlu: NLU):
        entities = "; ".join(map(lambda e: f"{e.key}:{e.value}", nlu.entities))
        row = [nlu.intent, entities, nlu.text]
        self.beginInsertRows(QtCore.QModelIndex(), 0, 0)
        self._data.insert(len(self._data), row)
        self.endInsertRows()

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            if col == 0:
                return "intent"
            elif col == 1:
                return "entities"
            elif col == 2:
                return "text"
        else:
            return None

    def data(self, index, role):
        if role == Qt.DisplayRole:
            if index.row() < len(self._data):
                return self._data[index.row()][index.column()]
            else:
                return ""

    def rowCount(self, index):
        return len(self._data)

    def columnCount(self, index):
        return 3

    def setRowCount(self, count):
        self.beginRemoveRows(QtCore.QModelIndex(), count, len(self._data) - 1)
        self._data = self._data[:count]
        self.endRemoveRows()
