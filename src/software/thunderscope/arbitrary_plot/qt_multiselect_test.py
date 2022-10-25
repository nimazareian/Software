from pyqtgraph.Qt import QtWidgets, QtCore, QtGui


# class List(QtGui.QDialog):
#
#     def __init__(self):
#         super(List, self).__init__()
#
#         self._listWidget = QtGui.QListWidget()
#         self._listWidget.addItems(['a', 'b', 'c'])
#
#         self._red = QtGui.QBrush(QtGui.QColor(255, 0, 0))
#         self._green = QtGui.QBrush(QtGui.QColor(0, 255, 0))
#
#         layout = QtGui.QVBoxLayout(self)
#         layout.addWidget(self._listWidget)
#
#         self._listWidget.itemDoubleClicked.connect(self._handleDoubleClick)
#
#     def _handleDoubleClick(self, item):
#         color = item.background()
#         item.setBackground(self._red if color == self._green else self._green)
#         item.setSelected(False)
#
#
# if __name__ == '__main__':
#     import sys
#     app = QtGui.QApplication(sys.argv)
#     d = List()
#     d.show()
#     # d.raise_()
#     app.exec_()

# class Test(QtWidgets.QDialog):
#     def __init__(self, parent=None):
#         super(Test, self).__init__(parent)
#         self.layout = QtWidgets.QVBoxLayout()
#         self.listWidget = QtWidgets.QListWidget()
#         self.listWidget.setSelectionMode(
#             QtWidgets.QAbstractItemView.SelectionMode.MultiSelection
#         )
#         self.listWidget.setGeometry(QtCore.QRect(10, 10, 211, 291))
#         for i in range(10):
#             item = QtWidgets.QListWidgetItem("Item %i" % i)
#             self.listWidget.addItem(item)
#         self.listWidget.itemClicked.connect(self.printItemText)
#         self.layout.addWidget(self.listWidget)
#         self.setLayout(self.layout)
#
#         self._red = QtGui.QBrush(QtGui.QColor(255, 0, 0))
#         self._green = QtGui.QBrush(QtGui.QColor(0, 255, 0))
#         self.listWidget.itemDoubleClicked.connect(self._handleDoubleClick)
#
#     def _handleDoubleClick(self, item):
#         color = item.background()
#         item.setBackground(self._red if color == self._green else self._green)
#         item.setSelected(False)
#
#     def printItemText(self):
#         items = self.listWidget.selectedItems()
#         x = []
#         for i in range(len(items)):
#             x.append(str(self.listWidget.selectedItems()[i].text()))
#
#         print (x)
#
# if __name__ == "__main__":
#     import sys
#     app = QtWidgets.QApplication(sys.argv)
#     form = Test()
#     form.show()
#     app.exec()

class Widget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.setLayout(QtWidgets.QVBoxLayout())
        self.GroupBox = QtWidgets.QGroupBox(self)

        self.GroupBox.setLayout(QtWidgets.QVBoxLayout())
        for i in range(6):
            checkbox = QtWidgets.QCheckBox("{}".format(i), self.GroupBox)
            self.GroupBox.layout().addWidget(checkbox)

        self.layout().addWidget(self.GroupBox)
        self.GroupBox.toggled.connect(self.onToggled)
        self.GroupBox.setCheckable(True)

    def onToggled(self, on):
        for box in self.sender().findChildren(QtWidgets.QCheckBox):
            box.setChecked(on)
            box.setEnabled(True)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    form = Widget()
    form.show()
    app.exec()
    # sys.exit(app.exec_())