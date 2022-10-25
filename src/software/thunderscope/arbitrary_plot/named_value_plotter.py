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
class NamedValuePlotter(QtWidgets.QWidget):
    """ Plot named values in real time with a scrolling plot """
    new_line_signal = QtCore.pyqtSignal(str, Color)

    def __init__(self, buffer_size=1000, *args, **kwargs):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        # TODO: Investigate (if performance is an issue, and will be better) to replace
        #       the field widget to use vispy shapes (vispy.scene.visuals)
        #       https://vispy.org/gallery/scene/polygon.html#sphx-glr-gallery-scene-polygon-py
        super().__init__(*args, **kwargs)
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

        self.line_point_lists = {}
        self.line_color_lists = {}
        self.line_visibility = {}
        self.assigned_line_colors = {}
        self.disable_tracking = False
        self.line = scene.visuals.Line(np.empty((0, 2), dtype=np.float32), parent=self.view.scene, color='white')

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
            if named_value.name not in self.line_point_lists:
                new_line_name = named_value.name
                new_line_color = Color(color=[random.uniform(0.4, 1.0) for _ in range(4)])

                self.line_point_lists[new_line_name] = np.empty((0, 2), dtype=np.float32)

                # Assign this line a random color
                self.assigned_line_colors[new_line_name] = new_line_color
                self.line_color_lists[new_line_name] = np.empty((0, 4), dtype=Color)
                self.new_line_signal.emit(new_line_name, new_line_color)
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
            self.line_point_lists[name] = np.append(self.line_point_lists[name], data, axis=0)

            color_data = np.empty((len(data), 4), dtype=Color)
            color_data[:] = self.assigned_line_colors[name].rgba
            self.line_color_lists[name] = np.append(self.line_color_lists[name], color_data, axis=0)

        # VisPy plots points from a single 2D list. We can specify which adjacent points connect
        # with each other using a boolean array where True means that adjacent points should be
        # connected, and False means they should be disconnected.
        # Create a single array of all data points:
        line_data = np.vstack([self.line_point_lists[name] for name, _ in self.line_point_lists.items()])
        color_data = np.vstack([self.line_color_lists[name] for name, _ in self.line_color_lists.items()])
        connections = np.ones(line_data.shape[0], dtype=bool)
        offset = 0
        for name, data in self.line_point_lists.items():
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

    def set_line_visibility(self, line_name: str, line_visibility: bool):
        self.line_visibility[line_name] = line_visibility

    def set_disable_tracking(self, disable_tracking: bool):
        print(f"Setting disable tracking to {disable_tracking}")
        self.disable_tracking = disable_tracking


# The Qt widget that will contain the vispy canvas and the side buttons
class MainPlotterWidget(QtWidgets.QWidget):
    def __init__(self, plotter, *args, **kwargs):

        # plotter_dock = Dock("plotter")
        # plotter_dock.hideTitleBar()
        # plotter_dock.addWidget(self.canvas.native)
        #
        # plot_controls_dock = Dock("plot controls")
        # plot_controls_dock.setStretch(x=5)
        # plot_controls_dock.hideTitleBar()
        # self.plot_controls = PlotControlsWidget()
        # plot_controls_dock.addWidget(self.plot_controls)
        #
        # self.dock_area = DockArea()
        # self.dock_area.addDock(plotter_dock)
        # self.dock_area.addDock(plot_controls_dock, "left", plotter_dock)
        #
        # self.main_layout = QtWidgets.QHBoxLayout()
        # self.main_layout.addWidget(self.dock_area)
        #
        # self.setLayout(self.main_layout)

        super().__init__(*args, **kwargs)
        # TODO: Make configurable and resizable layouts
        main_layout = QtWidgets.QHBoxLayout()

        # TODO: With buttons, performance seems much worse!?
        # The controls and dropdowns
        self.plot_controls = PlotControlsWidget()
        main_layout.addWidget(self.plot_controls)

        self.plotter = plotter
        main_layout.addWidget(self.plotter.canvas.native)

        self.setLayout(main_layout)
        self._connect_controls()

    def _connect_controls(self):
        # Use connect keyword to bind the listener for change in controls to the canvas
        self.plot_controls.disable_camera_tracking_signal.connect(self.plotter.set_disable_tracking)
        self.plot_controls.line_visibility_signal.connect(self.plotter.set_line_visibility)
        self.plotter.new_line_signal.connect(self.plot_controls.add_line_visibility_checkbox)

    def refresh(self):
        self.plotter.refresh()


# The controls for changing our vispy plot
class PlotControlsWidget(QtWidgets.QWidget):
    disable_camera_tracking_signal = QtCore.pyqtSignal(bool)
    line_visibility_signal = QtCore.pyqtSignal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtWidgets.QVBoxLayout()

        self.disable_tracking_checkbox = QtWidgets.QCheckBox("Disable Tracking")
        self.disable_tracking_checkbox.setChecked(False)
        self.disable_tracking_checkbox.stateChanged.connect(self.__on_camera_tracking_clicked)
        self.layout.addWidget(self.disable_tracking_checkbox)

        line_visibilities_label = QtWidgets.QLabel("Line Visibilities")
        self.layout.addWidget(line_visibilities_label)

        self.setLayout(self.layout)

    def __on_camera_tracking_clicked(self):
        disable_tracking = self.disable_tracking_checkbox.isChecked()
        self.disable_camera_tracking_signal.emit(disable_tracking)

    def __on_line_visibility_checkbox_pressed(self, checkbox: bool):
        self.line_visibility_signal.emit(checkbox.text(), checkbox.isChecked())

    def add_line_visibility_checkbox(self, line_name: str, line_color: Color):
        checkbox = QtWidgets.QCheckBox(line_name)
        checkbox.stateChanged.connect(lambda _, toggled_checkbox=checkbox: self.__on_line_visibility_checkbox_pressed(toggled_checkbox))
        print(f"{line_color.hex=}")
        checkbox.setStyleSheet(f"QCheckBox {{ background: {line_color.hex} ;}}")
        self.layout.addWidget(checkbox)
