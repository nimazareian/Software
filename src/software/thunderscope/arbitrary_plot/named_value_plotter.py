import random
import time

from vispy import scene
from vispy.color.color_array import Color
import numpy as np
from proto.visualization_pb2 import NamedValue

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
        self.line = scene.visuals.Line(np.empty((0, 2), dtype=np.float32), parent=self.view.scene, color='white')

        # Added for debugging
        self.total_time = 0

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
                # TODO: Is it possible to have different colors for each plot?
                self.plots[named_value.name] = np.empty((0, 2), dtype=np.float32)

                # Assign this line a random color
                new_color = Color(color=[random.uniform(0.4, 1.0) for _ in range(4)])
                self.assigned_plot_colors[named_value.name] = new_color
                self.plot_colors[named_value.name] = np.empty((0, 4), dtype=Color)

                # TODO: Text representing which line is being shown right now. This slows down the plots ALOT. Don't need to be redrawn every tick
                # TODO: Check what else the scene class provides that we can utilize
                # Can change method to 'gpu'
                # Text doesn't have to be in the visual either... Probably better to have it with the selection drop down
                # TODO: Add a drop down menu -overlay- to select which lines to show: https://stackoverflow.com/questions/49077083/how-to-overlay-widgets-in-pyqt5

            new_data_pair = np.empty((1, 2), dtype=np.float32)
            new_data_pair[0][0] = time.time() - self.time
            new_data_pair[0][1] = named_value.value
            if named_value.name not in new_data:
                # TODO: Time should somehow be a part of the named value plot...
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

        # TODO: Update camera to follow data
        # self.view.camera.set_range(x=[line[0, 0], line[-1, 0]], y=[0, 1], margin=0.1)
