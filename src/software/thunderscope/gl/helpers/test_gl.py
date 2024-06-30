from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtGui import QSurfaceFormat
from PyQt6.QtCore import QTimer
from PyQt6.QtOpenGL import QOpenGLDebugLogger
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

class OpenGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.debug_logger = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)  # Connect the timer to the update method
        self.timer.start(16)  # Roughly 60 FPS (1000 ms / 60 frames ≈ 16 ms per frame)

    def initializeGL(self):
        # self.initializeOpenGLFunctions()

        # Check OpenGL version
        version = glGetString(GL_VERSION)
        print(f'OpenGL version: {version}')

        # Set up OpenGL debugging
        self.debug_logger = QOpenGLDebugLogger(self)
        if self.debug_logger.initialize():
            self.debug_logger.messageLogged.connect(self.onDebugMessageLogged)
            print("OpenGL debug logger initialized")

        # Example OpenGL initialization
        glClearColor(1.0, 0.0, 0.0, 1.0)

        # Check for errors
        self.checkForErrors()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Check for errors
        self.checkForErrors()

    def onDebugMessageLogged(self, message):
        print(f'Debug message: {message.message()}')

    def checkForErrors(self):
        error = glGetError()
        if error != GL_NO_ERROR:
            print(f'OpenGL error: {error}')

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.opengl_widget = OpenGLWidget(self)
        self.layout.addWidget(self.opengl_widget)

        self.gl_view_widget = gl.GLViewWidget()
        self.layout.addWidget(self.gl_view_widget)

        self.gl_view_widget.setCameraPosition(distance=40)
        self.add_plot_items()

    def add_plot_items(self):
        gx = gl.GLGridItem()
        gx.rotate(90, 0, 1, 0)
        gx.translate(-10, 0, 0)
        self.gl_view_widget.addItem(gx)

        gy = gl.GLGridItem()
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -10, 0)
        self.gl_view_widget.addItem(gy)

        gz = gl.GLGridItem()
        gz.translate(0, 0, -10)
        self.gl_view_widget.addItem(gz)

        n = 51
        y = np.linspace(-10, 10, n)
        x = np.linspace(-10, 10, 100)
        for i in range(n):
            yi = y[i]
            d = np.hypot(x, yi)
            z = 10 * np.cos(d) / (d + 1)
            pts = np.column_stack([x, np.full_like(x, yi), z])
            plt = gl.GLLinePlotItem(pos=pts, color=pg.mkColor((i, n * 1.3)), width=(i + 1) / 10., antialias=True)
            self.gl_view_widget.addItem(plt)

def main():
    import sys
    app = QApplication(sys.argv)

    # Set the OpenGL format for the application
    fmt = QSurfaceFormat()
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    # fmt.setVersion(3, 3)  # Set the required OpenGL version here
    QSurfaceFormat.setDefaultFormat(fmt)

    main_window = MainWindow()
    main_window.resize(800, 600)
    main_window.show()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()
