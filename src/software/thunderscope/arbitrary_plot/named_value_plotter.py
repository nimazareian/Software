import random
import time
import numpy as np
from dataclasses import dataclass, field
from typing import Dict

from vispy import scene
from vispy.color.color_array import Color
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *

from proto.visualization_pb2 import NamedValue
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

INITIAL_Y_MIN = 0
INITIAL_Y_MAX = 100
TIME_WINDOW_TO_DISPLAY_S = 20
MAX_NUM_POINTS_IN_LINE = TIME_WINDOW_TO_DISPLAY_S * 60
BACKGROUND_COLOR = (0.1, 0.1, 0.1)


@dataclass
class NamedLine:
    name: str
    num_points: int = 0
    is_visible: bool = True
    color: Color = field(default_factory=lambda: Color(color=[random.uniform(0.4, 1.0) for _ in range(3)]))


class NamedValuePlotter(QWidget):
    """ Plot named values in real time with a scrolling plot """
    new_line_signal = QtCore.pyqtSignal(NamedLine)

    def __init__(self, buffer_size=200, *args, **kwargs):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        super().__init__(*args, **kwargs)
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor=BACKGROUND_COLOR)
        # For allowing to have multiple plots/axis in the same window
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")
        self.view.camera.set_range(x=[0, TIME_WINDOW_TO_DISPLAY_S], y=[INITIAL_Y_MIN, INITIAL_Y_MAX])

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
        self.last_buffer_read_time = time.time()
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

        self.lines: Dict[str, NamedLine] = {}
        self.live_plotting_enabled = True
        self.should_update_camera = False
        self.line_data = np.empty((0, 2), dtype=np.float32)
        self.vispy_line = scene.visuals.Line(self.line_data, parent=self.view.scene, color='white')

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        # TODO: Make the data aggregation multi-threaded
        # Shift old data to the right by the elapsed time
        self.line_data[:, 0] += time.time() - self.last_buffer_read_time

        # Organize the entire buffer into numpy arrays for each line
        new_data = {}
        for _ in range(self.named_value_buffer.queue.qsize()):
            named_value = self.named_value_buffer.get(block=False)

            # If named_value is new, create a plot and for the new value and
            # add it to necessary maps
            if named_value.name not in self.lines:
                new_line_name = named_value.name
                self.lines[new_line_name] = NamedLine(name=new_line_name)
                self.new_line_signal.emit(self.lines[new_line_name])
                self.should_update_camera = True

            new_data_pair = np.zeros((2, ), dtype=np.float32)
            new_data_pair[1] = named_value.value
            if named_value.name not in new_data:
                new_data[named_value.name] = [new_data_pair]
            else:
                new_data[named_value.name].append(new_data_pair)  #np.append(new_data_pair, new_data[named_value.name], axis=0)

        # Update the time which we last read the buffer
        self.last_buffer_read_time = time.time()
        
        # Add new data points to the existing data points
        vstack_array = []
        data_start_index = 0
        for name, line in self.lines.items():
            new_data_len = 0
            if name in new_data:
                vstack_array.append(np.array(new_data[name])[::-1])
                new_data_len = len(new_data[name])

            old_data_len = line.num_points
            old_data_end = data_start_index + min(old_data_len,
                                                  MAX_NUM_POINTS_IN_LINE - new_data_len)

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
        any_visible_lines = False
        offset = 0
        for name, line in self.lines.items():
            num_points = line.num_points

            if line.is_visible:
                # The last point of each line should not be connected with the first point
                # of the next line
                connections[offset + num_points - 1] = False
                if line.num_points > 0:
                    any_visible_lines = True
            else:
                # If the line is not visible, don't connect its points
                connections[offset:offset + num_points] = False
            offset += num_points

            line_colors.append(line.color.rgba)
            line_lengths.append(num_points)

        color_data = np.repeat(line_colors, line_lengths, axis=0)

        # Re-render plot
        if self.live_plotting_enabled:
            self.vispy_line.set_data(self.line_data, connect=connections, color=color_data)

        # Update camera
        if self.should_update_camera and any_visible_lines:
            # Calculate min/max y based on the visible points
            plotted_data = self.line_data[connections]
            y_min = plotted_data[:, 1].min()
            y_max = plotted_data[:, 1].max()

            self.view.camera.set_range(x=[0, TIME_WINDOW_TO_DISPLAY_S], y=[y_min, y_max])
            self.should_update_camera = False

    def set_line_visibility(self, line_name: str, line_visibility: bool):
        """Update the visibility of a line. If line_visibility is True, the line will be shown. If False, the line will not be rendered in the next refresh call.

        :param line_name: Name of the line to update
        :param line_visibility: New visibility of the line

        """
        self.lines[line_name].is_visible = line_visibility

    def set_live_plotting(self, live_plotting_enabled: bool):
        """Update whether the plots should be updated in real-time or not.

        :param live_plotting_enabled: If true, the plot will be updated in real time. If false, the plot will not be updated.

        """
        self.live_plotting_enabled = live_plotting_enabled

    def update_camera(self):
        """If this callback is called, the camera will be updated in the next refresh call."""
        self.should_update_camera = True


class PlotControlsWidget(QWidget):
    """Widget with the controls for the plotter"""

    # Signals for connecting the controls to the plotter
    update_camera_signal = QtCore.pyqtSignal()
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
        self.live_plotting_button.clicked.connect(self.__on_live_plotting_button_clicked)
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
        button_title = self.pause_plotting_title if self.plot_new_data else self.resume_plotting_title
        self.live_plotting_button.setText(button_title)

        # Emit signal to update the plotter
        self.live_plotting_signal.emit(self.plot_new_data)

    def __on_line_visibility_checkbox_pressed(self, checkbox: QCheckBox):
        """
        Callback for when a line visibility checkbox is pressed

        :param checkbox: The checkbox that was toggled

        """
        self.line_visibility_signal.emit(checkbox.text(), checkbox.isChecked())

        # Update the camera if a new line is shown/hidden
        self.update_camera_signal.emit()

    def add_line_visibility_checkbox(self, new_line: NamedLine):
        """
        Callback for adding a new line visibility checkbox. Should be called when a new line is added to the plotter

        :param new_line: The new line that was added

        """
        checkbox = QCheckBox(new_line.name)
        checkbox.setChecked(True)
        checkbox.setStyleSheet(f"QCheckBox::indicator {{ border: 3px solid {new_line.color.hex};}}")
        self.layout.addWidget(checkbox)

        # Wrap the default stateChanged callback with a lambda function so we can pass extra parameters
        checkbox.stateChanged.connect(lambda _, toggled_checkbox=checkbox: self.__on_line_visibility_checkbox_pressed(toggled_checkbox))

        # Update the camera when a new line is added
        self.update_camera_signal.emit()


class MainPlotterWidget(QWidget):
    """Widget that contains the plotter and the controls for the plotter"""

    def __init__(self):
        super().__init__()
        self.plotter = NamedValuePlotter()

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

    def _connect_controls(self):
        """Connect the plotter with the controls"""
        self.plot_controls.update_camera_signal.connect(self.plotter.update_camera)
        self.plot_controls.live_plotting_signal.connect(self.plotter.set_live_plotting)
        self.plot_controls.line_visibility_signal.connect(self.plotter.set_line_visibility)
        self.plotter.new_line_signal.connect(self.plot_controls.add_line_visibility_checkbox)

    def refresh(self):
        """Refresh the plotter with new data"""
        self.plotter.refresh()
