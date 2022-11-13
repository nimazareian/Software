import random
import time
import numpy as np
from dataclasses import dataclass, field
from typing import Dict

from vispy import scene
from vispy.app import Timer
from vispy.color.color_array import Color
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


@dataclass
class NamedLine:
    name: str
    num_points: int = 0
    is_visible: bool = True
    color: Color = field(
        default_factory=lambda: Color(
            color=[random.uniform(0.4, 1.0) for _ in range(3)]
        )
    )


class ProtoPlotter(QWidget):
    refreshed_plot_signal = QtCore.pyqtSignal()

    def __init__(
        self, window_secs, min_y=0, max_y=100,
    ):
        """Initializes Plotter.

        :param window_secs: How many seconds to show in the x axis
        :param min_y: Initial minimum y value to display
        :param max_y: Initial maximum y value to display

        """
        super().__init__()

        # Set up the VisPy plot
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor=(0.1, 0.1, 0.1))
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")
        self.view.camera.set_range(x=[0, window_secs], y=[min_y, max_y])
        self.window_secs = window_secs

        # Visualizing Axis
        self.x_axis = scene.AxisWidget(orientation="top")
        self.y_axis = scene.AxisWidget(orientation="right", tick_label_margin=3)
        self.x_axis.stretch = (1, 0.05)
        self.y_axis.stretch = (0.05, 1)
        self.grid.add_widget(self.x_axis, row=1, col=1)
        self.grid.add_widget(self.y_axis, row=0, col=0)
        self.x_axis.link_view(self.view)
        self.y_axis.link_view(self.view)

        # Initialize data structures
        self.line_data = np.zeros((1, 2), dtype=np.float32)
        self.color_data = np.zeros((1, 4), dtype=np.float32)
        self.connections = np.zeros(1, dtype=bool)
        self.vispy_line = scene.visuals.Line(
            self.line_data, parent=self.view.scene, color="white"
        )

        self.live_plotting_enabled = True
        self.should_update_camera = False

        # Variables added for debugging
        self.num_calls = 0
        self.refresh_total_time = 0

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        refresh_start = time.time()

        # TODO: Make the data aggregation multi-threaded. This should make points more spread out as well
        #       Aggregate new data with the time they were received, but only update the line (emit) 1/60 sec
        # Re-render plot
        if self.live_plotting_enabled:
            self.vispy_line.set_data(
                self.line_data, connect=self.connections, color=self.color_data
            )

        # Update camera
        if self.should_update_camera:
            # Calculate min/max y based on the visible points
            plotted_data = self.line_data[self.connections]
            if len(plotted_data) > 0:
                y_min = plotted_data[:, 1].min()
                y_max = plotted_data[:, 1].max()

                self.view.camera.set_range(x=[0, self.window_secs], y=[y_min, y_max])
                self.should_update_camera = False

        self.refreshed_plot_signal.emit()

        self.refresh_total_time += time.time() - refresh_start
        self.num_calls += 1
        if self.num_calls >= 100:
            print(f"=== avg plotter time for {self.num_calls} calls: {self.refresh_total_time / self.num_calls:0.5f}")
            self.num_calls = 0
            self.refresh_total_time = 0

    def set_new_data(self, new_data: dict):
        """Sets the new data to be plotted.

        :param new_data: A dictionary with the line, point, and connection data

        """
        self.line_data = new_data["line_data"]
        self.color_data = new_data["color_data"]
        self.connections = new_data["connections"]

    def set_live_plotting(self, live_plotting_enabled: bool):
        """Update whether the plots should be updated in real-time or not.

        :param live_plotting_enabled: If true, the plot will be updated in real time. If false, the plot will not be updated.

        """
        self.live_plotting_enabled = live_plotting_enabled

    def update_camera(self):
        """If this callback is called, the camera will be updated in the next refresh call."""
        self.should_update_camera = True


class ProtoPlotDataGenerator(QtCore.QObject):
    """Plot the protobuf data in a VisPy plot

    In-order to make the plotter as flexible as possible, we need dependency
    inject a way to extract data from the incoming protobufs. This is so that
    the user can control the way the data is plotted.

    Examples:

    NamedValueProto: we can plot the value directly
    RobotStatus: the data comes in over multiple packets, with each one having
    a different robot id. We need to append the robot id to the name so that
    the values are plotted separated by the ID. We also might want to plot
    different parts of the robot status in different plots.

    So we define a configuration dictionary that looks like this:

    {
        protobuf_type_1: data_extractor_function_1,
        protobuf_type_2: data_extractor_function_2,
        ...
    }

    The plotter will observe the provided protobuf types and call the associated
    data extractor function to extract the data.

    The data_extractor_function should take a protobuf return a dictionary of
    the form:

    {
        "name_1": value_1,
        "name_2": value_2,
        ...
    }

    """

    # Signal for sending the new data to the plotter
    new_data_signal = QtCore.pyqtSignal(dict)

    # Signal for when a new line is added
    new_line_signal = QtCore.pyqtSignal(NamedLine)

    update_camera_signal = QtCore.pyqtSignal()

    def __init__(self, configuration, max_points_per_line, buffer_size=200,):
        """Initializes a ProtoPlotDataGenerator.

        :param configuration: A dictionary of protobuf types to data extractor
        :param max_points_per_line: The maximum number of points to store per line
        :param buffer_size: The size of the buffer to use for plotting.

        """
        super().__init__()

        self.configuration = configuration
        self.buffers = {
            key: ThreadSafeBuffer(buffer_size, key) for key in configuration.keys()
        }

        # Initialize data structures
        self.lines: Dict[str, NamedLine] = {}
        self.line_data = np.empty((0, 2), dtype=np.float32)

        self.last_buffer_read_time = time.time()
        self.max_points_per_line = max_points_per_line
        self.cached_new_data = {}
        self.should_emit_new_data = True
        self.should_update_camera = False

    def generate_line_data(self):
        """Generates data for the proto plotter."""
        # TODO: If we x out of the window, this thread does NOT close!!
        # Shift old data to the right by the elapsed time
        self.line_data[:, 0] += time.time() - self.last_buffer_read_time
        # print(f"Shifting data by: {time.time() - self.last_buffer_read_time}")

        # Organize all buffers into numpy arrays containing new data for each line
        for proto_class, buffer in self.buffers.items():
            for _ in range(buffer.queue.qsize()):

                data = self.configuration[proto_class](buffer.get(block=False))

                for name, value in data.items():
                    # If named_value is new, add it to the necessary maps and notify listeners
                    if name not in self.lines:
                        self.lines[name] = NamedLine(name=name)
                        self.new_line_signal.emit(self.lines[name])
                        self.should_update_camera = True

                    new_data_pair = np.zeros((2,), dtype=np.float32)
                    new_data_pair[1] = value
                    if name not in self.cached_new_data:
                        self.cached_new_data[name] = [new_data_pair]
                    else:
                        self.cached_new_data[name].append(new_data_pair)
                    # print(f"New value read: {time.time()}")

        # Update the time which we last read the buffer
        self.last_buffer_read_time = time.time()

        if True: # self.should_emit_new_data and len(self.cached_new_data) > 0:
            # Add the new data points to the existing data points
            vstack_array = []
            data_start_index = 0
            for name, line in self.lines.items():
                new_data_len = 0
                if name in self.cached_new_data:
                    vstack_array.append(np.array(self.cached_new_data[name])[::-1])
                    new_data_len = len(self.cached_new_data[name])

                old_data_len = line.num_points
                old_data_end = data_start_index + min(
                    old_data_len, self.max_points_per_line - new_data_len
                )

                # Shift old data points to the right
                vstack_array.append(self.line_data[data_start_index:old_data_end])

                # Length of remaining old data + length of new data
                line.num_points = (old_data_end - data_start_index) + new_data_len
                data_start_index += old_data_len

            if len(vstack_array) > 0:
                self.line_data = np.vstack(vstack_array)
            else:
                self.line_data = np.empty((0, 2), dtype=np.float32)

            # Prepare the color and connection data
            connections = np.ones(len(self.line_data), dtype=bool)
            line_colors = []
            line_lengths = []
            offset = 0
            for name, line in self.lines.items():
                num_points = line.num_points

                if line.is_visible:
                    # The last point of each line should not be connected with the first point
                    # of the next line
                    connections[offset + num_points - 1] = False
                else:
                    # If the line is not visible, don't connect its points
                    connections[offset : offset + num_points] = False
                offset += num_points

                line_colors.append(line.color.rgba)
                line_lengths.append(num_points)

            color_data = np.repeat(line_colors, line_lengths, axis=0)

            data_dict = {
                "line_data": self.line_data,
                "color_data": color_data,
                "connections": connections,
            }
            # Send new data to the plotter
            self.new_data_signal.emit(data_dict)
            self.should_emit_new_data = False
            self.cached_new_data = {}

            # Update the camera if a new line was added or a line visibility was toggled
            if self.should_update_camera:
                self.update_camera_signal.emit()
                self.should_update_camera = False
        else:
            pass
            # print(f"{time.time()} NOT EMITTING NEW DATA")

        # QTimer is used instead of a blocking infinite loop to allow this thread to receive signals between iterations
        QtCore.QTimer.singleShot(0, self.generate_line_data)

    def emit_new_data(self):
        """Generate and send a new set of data to the plotter"""
        print("Received emit new data request")
        self.should_emit_new_data = True

    def set_line_visibility(self, line_name: str, line_visibility: bool):
        """Update the visibility of a line. If line_visibility is True, the line will be shown. If False, the line will not be rendered in the next refresh call.

        :param line_name: Name of the line to update
        :param line_visibility: New visibility of the line

        """
        self.lines[line_name].is_visible = line_visibility
        self.should_update_camera = True


class PlotControlsWidget(QWidget):
    """Widget with the controls for the plotter"""

    # Signals for connecting the controls to the plotter
    live_plotting_signal = QtCore.pyqtSignal(bool)
    line_visibility_signal = QtCore.pyqtSignal(str, bool)

    def __init__(self, parent=None):
        """
        :param parent: Parent widget

        """
        super().__init__(parent)
        self.layout = QFormLayout()

        # Button for enabling/disabling live plotting
        self.plot_new_data = True
        self.pause_plotting_title = "Pause Plotting"
        self.resume_plotting_title = "Resume Plotting"
        self.live_plotting_button = QPushButton(self.pause_plotting_title)
        self.live_plotting_button.clicked.connect(
            self.__on_live_plotting_button_clicked
        )
        self.layout.addWidget(self.live_plotting_button)

        # Horizontal line dividing sections
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        self.layout.addWidget(divider)

        # Title above the line visibility controls
        line_visibilities_label = QLabel("Line Visibilities")
        line_visibilities_label.setStyleSheet("font-weight: bold")
        line_visibilities_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(line_visibilities_label)

        self.setLayout(self.layout)

    def __on_live_plotting_button_clicked(self):
        """Callback for when the live plot button is clicked"""
        self.plot_new_data = not self.plot_new_data
        button_title = (
            self.pause_plotting_title
            if self.plot_new_data
            else self.resume_plotting_title
        )
        self.live_plotting_button.setText(button_title)

        # Emit signal to update the plotter
        self.live_plotting_signal.emit(self.plot_new_data)

    def __on_line_visibility_checkbox_pressed(self, checkbox: QCheckBox):
        """
        Callback for when a line visibility checkbox is pressed

        :param checkbox: The checkbox that was toggled

        """
        self.line_visibility_signal.emit(checkbox.text(), checkbox.isChecked())

    def add_line_visibility_checkbox(self, new_line: NamedLine):
        """
        Callback for adding a new line visibility checkbox. Should be called when a new line is added to the plotter

        :param new_line: The new line that was added

        """
        checkbox = QCheckBox(new_line.name)
        checkbox.setChecked(True)
        checkbox.setStyleSheet(
            f"QCheckBox::indicator {{ border: 3px solid {new_line.color.hex};}}"
        )
        self.layout.addWidget(checkbox)

        # Wrap the default stateChanged callback with a lambda function so we can pass extra parameters
        checkbox.stateChanged.connect(
            lambda _, toggled_checkbox=checkbox: self.__on_line_visibility_checkbox_pressed(
                toggled_checkbox
            )
        )


class MainPlotterWidget(QWidget):
    """Widget that contains the plotter and the controls for the plotter"""

    def __init__(self, plotter: ProtoPlotter, data_generator: ProtoPlotDataGenerator):
        super().__init__()
        self.plotter = plotter
        self.data_generator = data_generator

        # Run the data generator in a separate thread
        self.data_thread = QtCore.QThread(parent=self)
        self.data_generator.moveToThread(self.data_thread)

        # Start the data generator when the thread has started
        self.data_thread.started.connect(self.data_generator.generate_line_data)
        # When the thread has ended, delete the data source from memory
        self.data_thread.finished.connect(self.data_generator.deleteLater)
        self.data_thread.start()
        # TODO: Will probably have to stop data generation somehow (win.closing.connect(data_generator.stop_data, QtCore.Qt.DirectConnection))


        # Using pyqtgraph Docks to allow for easy resizing of the plotter and its controls
        plotter_dock = Dock("Plotter")
        plotter_dock.hideTitleBar()
        plotter_dock.addWidget(self.plotter.canvas.native)

        controls_dock = Dock("Plot Controls")
        controls_dock.hideTitleBar()
        controls_dock.setStretch(x=5)
        self.plot_controls = PlotControlsWidget(self)
        controls_dock.addWidget(self.plot_controls)

        self.dock_area = DockArea()
        self.dock_area.addDock(plotter_dock)
        self.dock_area.addDock(controls_dock, "left", plotter_dock)

        self.main_layout = QHBoxLayout()
        self.main_layout.addWidget(self.dock_area)
        self.setLayout(self.main_layout)

        self._connect_controls()

        # TODO: Added for debugging
        self.widget_total_time = 0
        self.num_calls = 0

    def _connect_controls(self):
        """Connect the plotter with the controls"""
        self.plot_controls.live_plotting_signal.connect(self.plotter.set_live_plotting)
        self.plot_controls.line_visibility_signal.connect(
            self.data_generator.set_line_visibility
        )
        self.data_generator.new_line_signal.connect(
            self.plot_controls.add_line_visibility_checkbox
        )
        self.data_generator.new_data_signal.connect(self.plotter.set_new_data)
        self.data_generator.update_camera_signal.connect(self.plotter.update_camera)
        self.plotter.refreshed_plot_signal.connect(self.data_generator.emit_new_data, QtCore.Qt.ConnectionType.DirectConnection)

    def refresh(self):
        """Refresh the plotter with new data"""
        widget_start = time.time()

        # self.data_generator.generate_line_data()
        self.plotter.refresh()

        self.widget_total_time += time.time() - widget_start
        self.num_calls += 1
        if self.num_calls >= 100:
            print(f"=== avg main widget time for {self.num_calls} calls: {self.widget_total_time / self.num_calls:0.5f}")
            self.num_calls = 0
            self.widget_total_time = 0
