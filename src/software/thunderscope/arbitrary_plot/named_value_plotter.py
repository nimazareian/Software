import random
import time

from vispy import scene
from vispy.color.color_array import Color
import numpy as np
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

# TODO: Move to constants
DEQUE_SIZE = 1000
INITIAL_Y_MIN = 0
INITIAL_Y_MAX = 100
TIME_WINDOW_TO_DISPLAY_S = 20
MAX_NUM_POINTS_IN_LINE = TIME_WINDOW_TO_DISPLAY_S * 60
BUFFER_RESIZE_THRESHOLD = 1.2
BACKGROUND_COLOR = (0.1, 0.1, 0.1)


class NamedValuePlotter(QWidget):
    """ Plot named values in real time with a scrolling plot """
    new_line_signal = QtCore.pyqtSignal(str, Color)

    def __init__(self, buffer_size=1000, *args, **kwargs): # TODO: Potentially lower buffer size for less powerful laptops
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
        self.last_buffer_read_time = time.time()
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

        self.line_point_lists = {}
        self.line_names = []
        self.line_data = np.empty((0, 2), dtype=np.float32)
        self.line_length = {}
        self.line_color_lists = {}
        self.line_visibility = {}
        self.assigned_line_colors = {}
        self.disable_tracking = False
        self.line = scene.visuals.Line(self.line_data, parent=self.view.scene, color='white')

        # Added for debugging
        self.refresh_total_time = 0
        self.new_data_total_time = 0
        self.vstack_total_time = 0
        self.setup_vstack_total_time = 0
        self.setup_color_total_time = 0
        self.create_final_array_total_time = 0
        self.draw_line_total_time = 0
        self.min_max_total_time = 0
        self.set_range_total_time = 0
        self.num_calls = 0

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        # TODO: Make the data aggregation multi-threaded
        refresh_start = time.time()

        # Shift old data to the right by the elapsed time
        time_since_last_read = time.time() - self.last_buffer_read_time
        self.line_data[:, 0] += time_since_last_read

        # Organize the entire buffer into numpy arrays for each line
        new_data = {}
        for _ in range(self.named_value_buffer.queue.qsize()):
            named_value = self.named_value_buffer.get(block=False)

            # If named_value is new, create a plot and for the new value and
            # add it to necessary maps
            # TODO: O(n) operation if its not hashed
            if named_value.name not in self.line_point_lists:
                new_line_name = named_value.name
                self.line_names.append(new_line_name)
                new_line_color = Color(color=[random.uniform(0.4, 1.0) for _ in range(4)])

                self.line_point_lists[new_line_name] = np.empty((0, 2), dtype=np.float32)
                self.line_length[new_line_name] = 0
                print(f"PLOT ADDED {new_line_name} => {len(self.line_point_lists.keys())} total plots")

                # Assign this line a random color
                self.assigned_line_colors[new_line_name] = new_line_color
                self.line_color_lists[new_line_name] = np.empty((0, 4), dtype=np.float32)
                self.new_line_signal.emit(new_line_name, new_line_color)
                self.line_visibility[new_line_name] = True

            # TODO: Instead of appending everytime, preallocate qsize() amount, fill, delete extra space
            #       https://betterprogramming.pub/numpy-illustrated-the-visual-guide-to-numpy-3b1d4976de1d
            new_data_pair = np.zeros((2, ), dtype=np.float32)
            new_data_pair[1] = named_value.value
            if named_value.name not in new_data:
                new_data[named_value.name] = [new_data_pair]
            else:
                new_data[named_value.name].append(new_data_pair)  #np.append(new_data_pair, new_data[named_value.name], axis=0)

        # Update the time which we last read the buffer
        self.last_buffer_read_time = time.time()

        self.new_data_total_time += time.time() - refresh_start

        create_final_array_start = time.time()
        
        # Add new data points to the existing data points
        vstack_array = []
        data_start_index = 0
        for name in self.line_names:
            # TODO: check visibility
            new_data_len = 0
            if name in new_data:
                vstack_array.append(np.array(new_data[name])[::-1])
                new_data_len = len(new_data[name])

            old_data_len = self.line_length[name]
            old_data_end = data_start_index + min(old_data_len,
                                                  MAX_NUM_POINTS_IN_LINE - new_data_len)

            # Shift old data points to the right
            vstack_array.append(self.line_data[data_start_index:old_data_end])

            # Length of remaining old data + length of new data
            self.line_length[name] = (old_data_end - data_start_index) + new_data_len

            data_start_index += old_data_len

        self.setup_vstack_total_time += time.time() - create_final_array_start
        vstack_start = time.time()
        self.line_data = np.vstack(vstack_array)
        self.vstack_total_time += time.time() - vstack_start

        create_color_connection_start = time.time()
        connections = np.ones(len(self.line_data), dtype=bool)
        visible_colors = []
        visible_line_lengths = []
        offset = 0
        for name in self.line_names:
            num_points = self.line_length[name]

            if self.line_visibility[name]:
                # The last point of each line should not be connected with the first point
                # of the next line
                connections[offset + num_points - 1] = False
            else:
                # If the line is not visible, don't connect its points
                connections[offset:offset + num_points] = False
            offset += num_points

            visible_colors.append(self.assigned_line_colors[name].rgba)
            visible_line_lengths.append(num_points)

        color_data = np.repeat(visible_colors, visible_line_lengths, axis=0)
        self.setup_color_total_time += time.time() - create_color_connection_start

        self.create_final_array_total_time += time.time() - create_final_array_start

        draw_line_start = time.time()
        # Re-render plot
        self.line.set_data(self.line_data, connect=connections, color=color_data)
        self.draw_line_total_time += time.time() - draw_line_start

        # Update camera
        # if not self.disable_tracking and len(line_data) > 0:
        #     min_max_start = time.time()
        #     y_min = line_data[:, 1].min()
        #     y_max = line_data[:, 1].max()
        #     self.min_max_total_time += time.time() - min_max_start
        #
        #     x_max = line_data[-1, 0]
        #     x_min = max(x_max-TIME_WINDOW_TO_DISPLAY_S, 0)
        #     set_range_start = time.time()
        #     # TODO: set_range is the bottleneck. one option is to shift the data rather than the camera
              # TODO: Disable tracking button ==update to==> auto_range button. auto_range when a plot is toggled
        #     self.view.camera.set_range(x=[x_min, x_max], y=[y_min, y_max])
        #     self.set_range_total_time += time.time() - set_range_start

        self.refresh_total_time += time.time() - refresh_start
        self.num_calls += 1
        if self.num_calls >= 150:
            print(f"=== avg time for {self.num_calls} calls:")
            avg_total_time = self.refresh_total_time / self.num_calls
            print(f"refresh_total_time = {avg_total_time:.5f}")
            new_data_avg = self.new_data_total_time / self.num_calls
            print(f"new_data_total_time = {new_data_avg:.5f} => {(new_data_avg / avg_total_time)*100:.5f}%")
            create_final_array_avg = self.create_final_array_total_time / self.num_calls
            print(f"create_final_array_total_time = {create_final_array_avg:.5f} => {(create_final_array_avg / avg_total_time)*100:.5f}%")
            setup_vstack_avg = self.setup_vstack_total_time / self.num_calls
            print(f"    setup_vstack_total_time = {setup_vstack_avg:.5f} => {(setup_vstack_avg / avg_total_time)*100:.5f}%")
            update_existing_datapoints_avg = self.vstack_total_time / self.num_calls
            print(f"    vstack_total_time = {update_existing_datapoints_avg:.5f} => {(update_existing_datapoints_avg / avg_total_time)*100:.5f}%")
            setup_color_avg = self.setup_color_total_time / self.num_calls
            print(f"    setup_vstack_total_time = {setup_color_avg:.5f} => {(setup_color_avg / avg_total_time)*100:.5f}%")
            draw_line_avg = self.draw_line_total_time / self.num_calls
            print(f"draw_line_total_time = {draw_line_avg:.5f} => {(draw_line_avg / avg_total_time)*100:.5f}%")

            self.num_calls = 0
            self.refresh_total_time = 0
            self.new_data_total_time = 0
            self.vstack_total_time = 0
            self.setup_vstack_total_time = 0
            self.setup_color_total_time = 0
            self.create_final_array_total_time = 0
            self.draw_line_total_time = 0

    def set_line_visibility(self, line_name: str, line_visibility: bool):
        """Update the visibility of a line. If line_visibility is True, the line will be shown. If False, the line will not be rendered in the next refresh call.

        :param line_name: Name of the line to update
        :param line_visibility: New visibility of the line

        """
        self.line_visibility[line_name] = line_visibility

    def set_disable_tracking(self, disable_tracking: bool):
        """If disable_tracking is True, the camera will not automatically move to the right

        :param disable_tracking: Whether to disable tracking

        """
        print(f"Setting disable tracking to {disable_tracking}")
        self.disable_tracking = disable_tracking


class PlotControlsWidget(QWidget):
    """Widget with the controls for the plotter"""

    # Signals for connecting the controls to the plotter
    disable_camera_tracking_signal = QtCore.pyqtSignal(bool)
    line_visibility_signal = QtCore.pyqtSignal(str, bool)

    def __init__(self, parent=None):
        """
        :param parent: Parent widget

        """
        super().__init__(parent)
        self.layout = QFormLayout()

        # Button for enabling/disabling camera tracking (auto-scrolling)
        self.disable_camera_tracking = False
        self.disable_tracking_title = "Disable Tracking"
        self.enable_tracking_title = "Enable Tracking"
        self.camera_tracking_button = QPushButton(self.disable_tracking_title)
        self.camera_tracking_button.clicked.connect(self.__on_camera_tracking_clicked)
        self.layout.addWidget(self.camera_tracking_button)

        # Horizontal line dividing sections
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        self.layout.addWidget(divider)

        # TODO: Add configurable buffer size (shouldn't hurt performance too much)
        line_visibilities_label = QLabel("Buffer Size")
        line_visibilities_label.setStyleSheet("font-weight: bold")
        line_visibilities_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(line_visibilities_label)

        self.buffer_size_input = QLineEdit()
        self.buffer_size_input.setValidator(QtGui.QIntValidator(200, 3600))
        # self.buffer_size_input.setValue(MAX_NUM_POINTS_IN_LINE)
        self.layout.addWidget(self.buffer_size_input)
        self.buffer_size_update_button = QPushButton("Update")
        self.layout.addWidget(self.buffer_size_update_button)

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

    def __on_camera_tracking_clicked(self):
        """Callback for when the camera tracking button is clicked"""
        self.disable_camera_tracking = not self.disable_camera_tracking
        button_title = self.enable_tracking_title if self.disable_camera_tracking else self.disable_tracking_title
        self.camera_tracking_button.setText(button_title)

        # Emit signal to update the plotter
        self.disable_camera_tracking_signal.emit(self.disable_camera_tracking)

    def __on_line_visibility_checkbox_pressed(self, checkbox: QCheckBox):
        """
        Callback for when a line visibility checkbox is pressed

        :param checkbox: The checkbox that was toggled

        """
        self.line_visibility_signal.emit(checkbox.text(), checkbox.isChecked())

    def add_line_visibility_checkbox(self, line_name: str, line_color: Color):
        """
        Callback for adding a new line visibility checkbox. Should be called when a new line is added to the plotter

        :param line_name: Name of the new line
        :param line_color: Color of the new line

        """
        checkbox = QCheckBox(line_name)
        checkbox.setChecked(True)
        checkbox.setStyleSheet(f"QCheckBox::indicator {{ border: 3px solid {line_color.hex};}}")
        self.layout.addWidget(checkbox)

        # Wrap the default stateChanged callback with a lambda function so we can pass the checkbox
        checkbox.stateChanged.connect(lambda _, toggled_checkbox=checkbox: self.__on_line_visibility_checkbox_pressed(toggled_checkbox))


class MainPlotterWidget(QWidget):
    """Widget that contains the plotter and the controls for the plotter"""

    def __init__(self):
        super().__init__()
        self.plotter = NamedValuePlotter()

        # Using pyqtgraph Docks to allow for easy resizing of the plotter and its controls
        plotter_dock = Dock("Plotter")
        plotter_dock.hideTitleBar()
        plotter_dock.addWidget(self.plotter.canvas.native)

        # TODO: With buttons, performance seems much worse!?
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
        self.plot_controls.disable_camera_tracking_signal.connect(self.plotter.set_disable_tracking)
        self.plot_controls.line_visibility_signal.connect(self.plotter.set_line_visibility)
        self.plotter.new_line_signal.connect(self.plot_controls.add_line_visibility_checkbox)

    def refresh(self):
        """Refresh the plotter with new data"""
        self.plotter.refresh()
