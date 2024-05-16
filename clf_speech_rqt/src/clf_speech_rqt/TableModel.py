import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from clf_speech_msgs.msg import Entity, NLU

class TableModel(QtCore.QAbstractTableModel):
    def __init__(self):
        super(TableModel, self).__init__()
        self._data = [
          ["", "", ""],
        ]

    def add_nlu(self, nlu: NLU):
        ents = f"; ".join(map(lambda e: f"{e.key}:{e.value}", nlu.entities))
        row = [nlu.intent, ents, nlu.text]
        self._data.insert(0, row)
        self._data = self._data[:5]
        print(self._data)

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            if col == 0:
                return 'intent'
            elif col == 1:
                return 'entities'
            elif col == 2:
                return 'text'
        else:
            return None

    def data(self, index, role):
        if role == Qt.DisplayRole:
            if index.row() < len(self._data):
                return self._data[index.row()][index.column()]
            else:
                return ""
            

    def rowCount(self, index):
        return 5

    def columnCount(self, index):
        # The following takes the first sub-list, and returns
        # the length (only works if all rows are an equal length)
        return 3