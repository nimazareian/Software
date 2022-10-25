import random
import time

from vispy import scene
from vispy.color.color_array import Color
import numpy as np
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
INITIAL_Y_MIN = 0
INITIAL_Y_MAX = 100
TIME_WINDOW_TO_DISPLAY_S = 20
MAX_NUM_POINTS_IN_LINE = TIME_WINDOW_TO_DISPLAY_S * 60
BACKGROUND_COLOR = (0.1, 0.1, 0.1)


class NamedValuePlotter(QWidget):
    """ Plot named values in real time with a scrolling plot """
    new_line_signal = QtCore.pyqtSignal(str, Color)

    def __init__(self, buffer_size=1000, *args, **kwargs):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        super().__init__(*args, **kwargs)
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor=BACKGROUND_COLOR)
        # For allowing to have multiple plots in the same window
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")
        self.view.camera.set_range(x=[0, TIME_WINDOW_TO_DISPLAY_S], y=[INITIAL_Y_MIN, INITIAL_Y_MAX])

        # Visualizing Axis
        self.x_axis = scene.AxisWidget(orientation="bottom")
        self.y_axis = scene.AxisWidget(orientation="right")
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
                self.line_color_lists[new_line_name] = np.empty((0, 4), dtype=np.float32)
                self.new_line_signal.emit(new_line_name, new_line_color)
                self.line_visibility[new_line_name] = True
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

        # TODO: Time every section of the code and see if there are any bottlenecks
        # Add new data points to the existing data points
        for name, new_data_points in new_data.items():
            line_points = self.line_point_lists[name]

            line_color = self.line_color_lists[name]
            new_data_colors = np.empty((len(new_data_points), 4), dtype=np.float32)
            new_data_colors[:] = self.assigned_line_colors[name].rgba

            if len(line_points) > MAX_NUM_POINTS_IN_LINE:
                # Shift point/color values to the left
                shift = -1 * len(new_data_points)
                # TODO: See if these can be simplified to 1 line
                line_points[:, 0] = np.roll(line_points[:, 0], shift)
                line_points[:, 1] = np.roll(line_points[:, 1], shift)

                line_color[:, 0] = np.roll(line_color[:, 0], shift)
                line_color[:, 1] = np.roll(line_color[:, 1], shift)
                line_color[:, 2] = np.roll(line_color[:, 2], shift)
                line_color[:, 3] = np.roll(line_color[:, 3], shift)

                # Add new data at the end
                line_points[shift:len(line_points), :] = new_data_points[:, :]
                line_color[shift:len(line_color), :] = new_data_colors[:, :]
            else:
                self.line_point_lists[name] = np.append(self.line_point_lists[name], new_data_points, axis=0)
                self.line_color_lists[name] = np.append(line_color, new_data_colors, axis=0)

        line_data = np.empty((1, 2), dtype=np.float32)
        color_data = np.empty((1, 4), dtype=np.float32)
        connections = np.empty((1,), dtype=bool)
        if True in self.line_visibility.values():
            # VisPy plots points from a single 2D list. We can specify which adjacent points connect
            # with each other using a boolean array.
            # Create a single array of all data points:
            line_data = np.vstack([self.line_point_lists[name] for name in self.line_point_lists.keys() if self.line_visibility[name]])
            color_data = np.vstack([self.line_color_lists[name] for name in self.line_color_lists.keys() if self.line_visibility[name]])
            connections = np.ones(line_data.shape[0], dtype=bool)
            offset = 0
            for name, new_data_points in self.line_point_lists.items():
                if self.line_visibility[name]:
                    # The last point of each line should not be connected with the first point
                    # of the next line
                    num_points = new_data_points.shape[0]
                    connections[offset + num_points - 1] = False
                    offset += num_points

        # Update plot
        self.line.set_data(line_data, connect=connections, color=color_data)

        # Update camera
        if not self.disable_tracking and len(line_data) > 0:
            x_max = line_data[-1, 0]
            x_min = max(x_max-TIME_WINDOW_TO_DISPLAY_S, 0)
            self.view.camera.set_range(x=[x_min, x_max], y=[INITIAL_Y_MIN, INITIAL_Y_MAX])

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


# The controls for changing our vispy plot
class PlotControlsWidget(QWidget):
    disable_camera_tracking_signal = QtCore.pyqtSignal(bool)
    line_visibility_signal = QtCore.pyqtSignal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QFormLayout()

        self.disable_camera_tracking = False
        self.disable_tracking_title = "Disable Tracking"
        self.enable_tracking_title = "Enable Tracking"
        self.camera_tracking_button = QPushButton(self.disable_tracking_title)
        self.camera_tracking_button.clicked.connect(self.__on_camera_tracking_clicked)
        self.layout.addWidget(self.camera_tracking_button)

        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        self.layout.addWidget(divider)

        line_visibilities_label = QLabel("Line Visibilities")
        line_visibilities_label.setStyleSheet("font-weight: bold")
        line_visibilities_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(line_visibilities_label)

        self.setLayout(self.layout)

    def __on_camera_tracking_clicked(self):
        self.disable_camera_tracking = not self.disable_camera_tracking
        button_title = self.enable_tracking_title if self.disable_camera_tracking else self.disable_tracking_title
        self.camera_tracking_button.setText(button_title)

        self.disable_camera_tracking_signal.emit(self.disable_camera_tracking)

    def __on_line_visibility_checkbox_pressed(self, checkbox: bool):
        self.line_visibility_signal.emit(checkbox.text(), checkbox.isChecked())

    def add_line_visibility_checkbox(self, line_name: str, line_color: Color):
        checkbox = QCheckBox(line_name)
        checkbox.setChecked(True)
        checkbox.stateChanged.connect(lambda _, toggled_checkbox=checkbox: self.__on_line_visibility_checkbox_pressed(toggled_checkbox))
        checkbox.setStyleSheet(f"QCheckBox::indicator {{ border: 3px solid {line_color.hex};}}")
        self.layout.addWidget(checkbox)


# The Qt widget that will contain the vispy canvas and the side buttons
class MainPlotterWidget(QWidget):
    def __init__(self, plotter, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.plotter = plotter

        plotter_dock = Dock("Plotter")
        plotter_dock.hideTitleBar()
        plotter_dock.addWidget(self.plotter.canvas.native)

        # TODO: With buttons, performance seems much worse!?
        controls_dock = Dock("Plot Controls")
        controls_dock.hideTitleBar()
        controls_dock.setStretch(x=5)
        self.plot_controls = PlotControlsWidget()
        controls_dock.addWidget(self.plot_controls)

        self.dock_area = DockArea()
        self.dock_area.addDock(plotter_dock)
        self.dock_area.addDock(controls_dock, "left", plotter_dock)

        self.main_layout = QHBoxLayout()
        self.main_layout.addWidget(self.dock_area)

        self.setLayout(self.main_layout)
        self._connect_controls()

    def _connect_controls(self):
        """Connect the plotter with the controls"""
        self.plot_controls.disable_camera_tracking_signal.connect(self.plotter.set_disable_tracking)
        self.plot_controls.line_visibility_signal.connect(self.plotter.set_line_visibility)
        self.plotter.new_line_signal.connect(self.plot_controls.add_line_visibility_checkbox)

    def refresh(self):
        self.plotter.refresh()