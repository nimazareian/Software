# https://www.youtube.com/watch?v=k1Z-55lHNm8&list=PL2OQ8odJIDfPU67ML2k-ldvgtISoxJE8b&index=2
# Code: https://vispy.org/gallery/scene/realtime_data/ex01_embedded_vispy.html#sphx-glr-gallery-scene-realtime-data-ex01-embedded-vispy-py

import math
import numpy as np
from PyQt6 import QtWidgets, QtCore
from vispy.app import use_app, Timer
from vispy import scene

bg_clr = (0.1, 0.1, 0.1)  # dark background color
TIME_LENGTH = 1
NUM_LINE_POINTS = TIME_LENGTH * 60


# The Qt widget that will contain the vispy canvas and the side buttons
class MyMainWindow(QtWidgets.QMainWindow):
    def __init__(self, canvas_wrapper, *args, **kwargs):
        super().__init__(*args, **kwargs)

        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout()

        # The controls and dropdowns
        self._controls = Controls()
        main_layout.addWidget(self._controls)

        # The plot canvas
        self._canvas_wrapper = canvas_wrapper
        # .native is the low-level pyqt widget that vispy uses and can be added to the layout
        main_layout.addWidget(self._canvas_wrapper.canvas.native)  # TODO: Improtant!

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self._connect_controls()

    def _connect_controls(self):
        # Use connect keyword to bind the listener for change in controls to the canvas
        self._controls.line_color_chooser.currentTextChanged.connect(self._canvas_wrapper.set_line_color)


# Must extends QObject class to access Qt signals and slots
class DataSource(QtCore.QObject):
    # Passing the data type which is passed through the signal
    new_data = QtCore.pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._count = 0

        self._line_data = None
        self._connections = None

        pos = np.empty((NUM_LINE_POINTS, 2), dtype=np.float32)
        pos[:, 0] = np.linspace(start=0, stop=TIME_LENGTH, num=NUM_LINE_POINTS)
        pos[:, 1] = np.zeros((NUM_LINE_POINTS,), dtype=np.float32)
        self._line_data1 = pos
        self._line_data2 = pos.copy()

    def run_data_creation(self, timer_event):
        """
        This is the function that will be called by the timer. It will emit a signal with the new data
        :param timer_event:
        :return:
        """
        self._count += 1
        self._line_data1, self._connections = self._update_line_data()
        # Create and emit a dictionary with the new data
        self.new_data.emit({"line": self._line_data, "connections": self._connections})

    def _update_line_data(self):
        # TODO: Using np.roll to shift the data to the left and add a new value of sin wave
        #       Only roll if we x has reached TIME_LENGTH, else add more data to the right
        # Shift x values by 1/60
        self._line_data1[:, 0] = np.roll(self._line_data1[:, 0], -1)
        self._line_data1[-1, 0] = self._line_data1[-2, 0] + 1 / 60
        # Shift y values and add a new value
        self._line_data1[:, 1] = np.roll(self._line_data1[:, 1], -1)
        self._line_data1[-1, 1] = -abs(math.sin(self._count / 50 * math.pi))

        # Shift x values by 1/60
        self._line_data2[:, 0] = np.roll(self._line_data2[:, 0], -1)
        self._line_data2[-1, 0] = self._line_data2[-2, 0] + 1 / 60
        # Shift y values and add a new value
        self._line_data2[:, 1] = np.roll(self._line_data2[:, 1], -1)
        self._line_data2[-1, 1] = abs(math.sin(self._count / 50 * math.pi))-1

        # vstack supports connecting any number of input arrays
        self._line_data = np.vstack((self._line_data1, self._line_data2))
        self._connections = np.ones(self._line_data.shape[0], dtype=bool)
        self._connections[self._line_data1.shape[0] - 1] = False
        print(f"{self._line_data1.shape[0]=}", f"{self._line_data2.shape[0]=}", f"{self._line_data.shape[0]=}")

        return self._line_data.copy(), self._connections.copy()  # Why do we return a copy


# The controls for changing our vispy plot
class Controls(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()

        self.line_color_label = QtWidgets.QLabel("Line color:")
        layout.addWidget(self.line_color_label)
        self.line_color_chooser = QtWidgets.QComboBox()
        self.line_color_chooser.addItems(["black", "red", "blue"])
        layout.addWidget(self.line_color_chooser)

        layout.addStretch(1)
        self.setLayout(layout)


# The canvas that will contain the vispy scene
class CanvasWrapper:
    def __init__(self):
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor=bg_clr)
        # For allowing to have multiple plots in the same window
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")

        # Visualizing Axis
        self.x_axis = scene.AxisWidget(orientation="bottom")
        self.y_axis = scene.AxisWidget(orientation="left")
        self.x_axis.stretch = (1, 0.05)  # Not sure what this does
        self.y_axis.stretch = (0.05, 1)
        self.grid.add_widget(self.x_axis, row=1, col=1)
        self.grid.add_widget(self.y_axis, row=0, col=0)
        self.x_axis.link_view(self.view)
        self.y_axis.link_view(self.view)

        # Add data to view
        line_data = _generate_random_line_positions(NUM_LINE_POINTS)
        self.line = scene.visuals.Line(line_data, parent=self.view.scene, color='white')
        self.view.camera.set_range(x=[0, TIME_LENGTH], y=[0, 1])

    def show(self):
        self.canvas.show()

    def set_line_color(self, color):
        print(f"Changing line color to {color}")
        self.line.set_data(color=color)

    def update_data(self, new_data_dict):
        line = new_data_dict["line"]
        connections = new_data_dict["connections"]
        self.line.set_data(line, connect=connections)  # TODO: Figure out connections. Could use array of bools for which adjacent points are connected
        # Update camera with 10% margin around the data
        # self.view.camera.set_range(x=[line[0, 0], line[-1, 0]], y=[0, 1], margin=0.1)  # TODO: Hard coding y range, to find min/max will have to search entire array


def _generate_random_line_positions(num_points, dtype=np.float32):
    rng = np.random.default_rng()
    pos = np.empty((num_points, 2), dtype=np.float32)
    pos[:, 0] = np.arange(num_points)
    pos[:, 1] = rng.random((num_points,), dtype=dtype)
    return pos


class Plot:
    def __init__(self, curves):
        pass


if __name__ == "__main__":
    app = use_app("pyqt6")
    app.create()

    data_source = DataSource()
    canvas_wrapper = CanvasWrapper()
    window = MyMainWindow(canvas_wrapper)

    data_source.new_data.connect(canvas_wrapper.update_data)
    # Vispy's wrapper around QTimer. It will call the run_data_creation function as fast it can
    # Timers can be problematic as its blocking the main thread
    timer = Timer("0.1", connect=data_source.run_data_creation, start=True)
    window.show()

    # Steps: Using VisPy Timer to call the run_data_creation function
    # 1. data_source.run_data_creation
    # 2. data_source.new_data.emit
    # 3. canvas_wrapper.update_data
    # 4. canvas_wrapper.line.set_data (Updating the plot)

    app.run()