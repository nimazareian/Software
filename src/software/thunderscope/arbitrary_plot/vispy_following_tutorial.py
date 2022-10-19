# https://www.youtube.com/watch?v=k1Z-55lHNm8&list=PL2OQ8odJIDfPU67ML2k-ldvgtISoxJE8b&index=2
# Code: https://vispy.org/gallery/scene/realtime_data/ex01_embedded_vispy.html#sphx-glr-gallery-scene-realtime-data-ex01-embedded-vispy-py

import numpy as np
from PyQt6 import QtWidgets
from vispy.app import use_app
from vispy import scene

bg_clr = (0.1, 0.1, 0.1)  # dark background color


# The Qt widget that will contain the vispy canvas and the side buttons
class MyMainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout()

        # The controls and dropdowns
        self._controls = Controls()
        main_layout.addWidget(self._controls)

        # The plot canvas
        self._canvas_wrapper = CanvasWrapper()
        # .native is the low-level pyqt widget that vispy uses and can be added to the layout
        main_layout.addWidget(self._canvas_wrapper.canvas.native)  # TODO: Improtant!

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)


# The controls for changing our vispy plot
class Controls(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()
        self.colormap_label = QtWidgets.QLabel("Image Colormap:")
        layout.addWidget(self.colormap_label)
        self.colormap_chooser = QtWidgets.QComboBox()
        self.colormap_chooser.addItems(["viridis", "reds", "blues"])
        layout.addWidget(self.colormap_chooser)

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
        self.canvas = scene.SceneCanvas(keys="interactive", show=True, bgcolor=bg_clr)
        # For allowing to have multiple plots in the same window
        self.grid = self.canvas.central_widget.add_grid()
        # Supporting panning and zooming
        self.view = self.grid.add_view(row=0, col=1, camera="panzoom")

        # Visualizing Axis
        self.x_axis = scene.AxisWidget(orientation="bottom")
        self.y_axis = scene.AxisWidget(orientation="left")
        self.x_axis.stretch = (1, 0.05)
        self.y_axis.stretch = (0.05, 1)
        self.grid.add_widget(self.x_axis, row=1, col=1)
        self.grid.add_widget(self.y_axis, row=0, col=0)
        self.x_axis.link_view(self.view)
        self.y_axis.link_view(self.view)

        self.view.camera.set_range()

        # Add data to view
        NUM_LINE_POINTS = 100
        line_data = _generate_random_line_positions(NUM_LINE_POINTS)
        self.line = scene.visuals.Line(line_data, parent=self.view.scene, color='white')
        self.view.camera.set_range(x=(0, NUM_LINE_POINTS), y=(0, 1))

    def show(self):
        self.canvas.show()


def _generate_random_image_data(shape, dtype=np.float32):
    rng = np.random.default_rng()
    data = rng.random(shape, dtype=dtype)
    return data


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

    window = MyMainWindow()
    window.show()

    app.run()