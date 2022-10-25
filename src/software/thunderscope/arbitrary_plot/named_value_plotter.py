import random
import time

from vispy import scene
from vispy.color.color_array import Color
import numpy as np
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
from pyqtgraph.dockarea import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
INITIAL_Y_MIN = 0
INITIAL_Y_MAX = 100
TIME_WINDOW_TO_DISPLAY_S = 20
MAX_NUM_POINTS_IN_LINE = TIME_WINDOW_TO_DISPLAY_S * 60
BACKGROUND_COLOR = (0.1, 0.1, 0.1)


# TODO: Add button to increase or decrease time window to display
class NamedValuePlotter(object):
    """ Plot named values in real time with a scrolling plot """

    def __init__(self, buffer_size=1000):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        # TODO: Investigate (if performance is an issue, and will be better) to replace
        #       the field widget to use vispy shapes (vispy.scene.visuals)
        #       https://vispy.org/gallery/scene/polygon.html#sphx-glr-gallery-scene-polygon-py
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor=BACKGROUND_COLOR)
        # For allowing to have multiple plots in the same window
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")
        self.view.camera.set_range(x=[0, TIME_WINDOW_TO_DISPLAY_S], y=[INITIAL_Y_MIN, INITIAL_Y_MAX])

        # Visualizing Axis
        # TODO: https://vispy.org/api/vispy.scene.visuals.html#vispy.scene.visuals.Axis
        self.x_axis = scene.AxisWidget(orientation="bottom")
        self.y_axis = scene.AxisWidget(orientation="left")
        self.x_axis.stretch = (1, 0.05)  # TODO: Not sure what this does
        self.y_axis.stretch = (0.05, 1)
        self.grid.add_widget(self.x_axis, row=1, col=1)
        self.grid.add_widget(self.y_axis, row=0, col=0)
        self.x_axis.link_view(self.view)
        self.y_axis.link_view(self.view)

        # Initialize data structures
        self.time = time.time()
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

        self.plots = {}
        self.plot_colors = {}
        self.assigned_plot_colors = {}
        self.disable_tracking = False
        self.line = scene.visuals.Line(np.empty((0, 2), dtype=np.float32), parent=self.view.scene, color='white')

        plotter_dock = Dock("plotter")
        plotter_dock.addWidget(self.canvas.native)

        plot_controls_dock = Dock("plot controls")
        self.plot_controls = PlotControls()
        plot_controls_dock.addWidget(self.canvas.native)

        self.dock_area = DockArea()
        self.dock_area.addDock(plotter_dock)
        self.dock_area.addDock(plot_controls_dock, "left", plotter_dock)

        # Added for debugging
        self.total_time = 0
        self.num_calls = 0

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.
        """
        start = time.time()

        # Dump the entire buffer into a deque. This operation is fast because
        # its just consuming data from the buffer and appending it to a deque.
        new_data = {}
        for _ in range(self.named_value_buffer.queue.qsize()):
            named_value = self.named_value_buffer.get(block=False)
            # TODO: Calculate the min/max x for the camera
            # If named_value is new, create a plot and for the new value and
            # add it to necessary maps
            if named_value.name not in self.plots:
                self.plots[named_value.name] = np.empty((0, 2), dtype=np.float32)

                # Assign this line a random color
                new_color = Color(color=[random.uniform(0.4, 1.0) for _ in range(4)])
                self.assigned_plot_colors[named_value.name] = new_color
                self.plot_colors[named_value.name] = np.empty((0, 4), dtype=Color)
                # TODO: Add a drop down menu -overlay- to select which lines to show: https://stackoverflow.com/questions/49077083/how-to-overlay-widgets-in-pyqt5

            new_data_pair = np.empty((1, 2), dtype=np.float32)
            new_data_pair[0][0] = time.time() - self.time
            new_data_pair[0][1] = named_value.value
            if named_value.name not in new_data:
                new_data[named_value.name] = new_data_pair
            else:
                # TODO: Limit the arrays to TIME_WINDOW_TO_DISPLAY_S length, and then shift the array to the left
                #       Time how much time this adds, if its a lot, we can do it less often. Once every 10 sec? Gotta
                #       make sure it doesn't suddenly slow down everything though
                new_data[named_value.name] = np.append(new_data[named_value.name], new_data_pair, axis=0)

        # Add new data points to the existing data points
        for name, data in new_data.items():
            self.plots[name] = np.append(self.plots[name], data, axis=0)

            color_data = np.empty((len(data), 4), dtype=Color)
            color_data[:] = self.assigned_plot_colors[name].rgba
            self.plot_colors[name] = np.append(self.plot_colors[name], color_data, axis=0)

        # VisPy plots points from a single 2D list. We can specify which adjacent points connect
        # with each other using a boolean array where True means that adjacent points should be
        # connected, and False means they should be disconnected.
        # Create a single array of all data points:
        line_data = np.vstack([self.plots[name] for name, _ in self.plots.items()])
        color_data = np.vstack([self.plot_colors[name] for name, _ in self.plot_colors.items()])
        connections = np.ones(line_data.shape[0], dtype=bool)
        offset = 0
        for name, data in self.plots.items():
            # The last point of each line should not be connected with the first point
            # of the next line
            num_points = data.shape[0]
            connections[offset + num_points - 1] = False
            offset += num_points

        self.line.set_data(line_data, connect=connections, color=color_data)
        # TODO: Add button for disabling camera following data
        if not self.disable_tracking:
            self.view.camera.set_range(x=[max(line_data[-1, 0]-TIME_WINDOW_TO_DISPLAY_S, 0), line_data[-1, 0]], y=[0, 100], margin=0.1)

        self.total_time += time.time() - start
        self.num_calls += 1
        if self.num_calls > 150:
            print(f"avg time {self.num_calls}: {self.total_time / self.num_calls}")
            self.num_calls = 0
            self.total_time = 0

    def set_line_color(self, color):
        print(f"Changing line color to {color}")
        for name, _ in self.assigned_plot_colors.items():
            self.assigned_plot_colors[name] = Color(color=color)

    def set_disable_tracking(self, disable_tracking):
        print(f"Setting disable tracking to {disable_tracking}")
        self.disable_tracking = disable_tracking


# The Qt widget that will contain the vispy canvas and the side buttons
class MainPlotterWidget(QtWidgets.QWidget):
    def __init__(self, plotter, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # TODO: Make configurable and resizable layouts
        main_layout = QtWidgets.QHBoxLayout()

        # TODO: With buttons, performance seems much worse!?
        # The controls and dropdowns
        self.plot_controls = PlotControls()
        main_layout.addWidget(self.plot_controls)

        self.plotter = plotter
        main_layout.addWidget(self.plotter.canvas.native)

        self.setLayout(main_layout)
        self._connect_controls()

    def _connect_controls(self):
        # Use connect keyword to bind the listener for change in controls to the canvas
        self.plot_controls.named_plot_picker.currentTextChanged.connect(self.plotter.set_line_color)
        self.plot_controls.disable_tracking_checkbox.stateChanged.connect(self.plotter.set_disable_tracking)

    def refresh(self):
        self.plotter.refresh()


# The controls for changing our vispy plot
class PlotControls(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()

        self.line_color_label = QtWidgets.QLabel("Line color:")
        layout.addWidget(self.line_color_label)
        self.named_plot_picker = QtWidgets.QComboBox()
        self.named_plot_picker.addItems(["green", "red", "blue"])
        layout.addWidget(self.named_plot_picker)

        self.disable_tracking_checkbox = QtWidgets.QCheckBox("Disable Tracking")
        self.disable_tracking_checkbox.setChecked(False)
        layout.addWidget(self.disable_tracking_checkbox)

        # List
        self.listWidget = QtWidgets.QListWidget()
        self.listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.SelectionMode.MultiSelection
        )
        for i in range(10):
            # TODO: Checkout QListWidgetItem.setHidden, setIcon
            item = QtWidgets.QListWidgetItem("Item %i" % i)
            self.listWidget.addItem(item)
            item.setTextAlignment(2 ** (i % 4))  # Text alignment DOES work!
            item.setBackground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))  # Color gets set, but not reflected in GUI
            # Foreground = Textcolor (when not selected)
            # Background = background color (not the item color when selected)

        self.listWidget.itemClicked.connect(self.printItemText)
        self.listWidget.setStyleSheet("""
    QListView::item:selected {
        border: 1px solid #400404;
        background: #d41608;
    }""")
        layout.addWidget(self.listWidget)

        # Group checkbox
        self.GroupBox = QtWidgets.QGroupBox(self)
        self.GroupBox.setLayout(QtWidgets.QVBoxLayout())
        for i in range(6):
            checkbox = QtWidgets.QCheckBox("{}".format(i), self.GroupBox)
            checkbox.stateChanged.connect(lambda _, checkbox=checkbox: self.onCheckboxToggle(checkbox))
            checkbox.setStyleSheet("""QCheckBox {
                                        background: #d41608;
                                   }""")
            self.GroupBox.layout().addWidget(checkbox)

        layout.addWidget(self.GroupBox)
        self.GroupBox.toggled.connect(self.onToggled)
        self.GroupBox.setCheckable(True)

        layout.addStretch(1)
        self.setLayout(layout)

    def printItemText(self, item):
        print(item.foreground().color().name())
        items = self.listWidget.selectedItems()
        x = []
        for i in range(len(items)):
            x.append(str(self.listWidget.selectedItems()[i].text()))

        print(x)

    def onToggled(self, on):
        print(f"QGroupBox pressed {on}")
        for box in self.sender().findChildren(QtWidgets.QCheckBox):
            box.setChecked(on)
            box.setEnabled(True)

    def onCheckboxToggle(self, checkbox):
        print(f"checkbox pressed {checkbox.text()}")