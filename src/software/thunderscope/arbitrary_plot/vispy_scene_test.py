# from https://vispy.org/gallery/scene/line_update.html
import random
import sys
import time

import numpy as np
from vispy import app, scene, color


canvas = scene.SceneCanvas(keys='interactive', show=True)
grid = canvas.central_widget.add_grid(spacing=0)

viewbox = grid.add_view(row=0, col=1, camera='panzoom')

# add some axes
x_axis = scene.AxisWidget(orientation='bottom')
x_axis.stretch = (1, 0.1)
grid.add_widget(x_axis, row=1, col=1)
x_axis.link_view(viewbox)
y_axis = scene.AxisWidget(orientation='left')
y_axis.stretch = (0.1, 1)
grid.add_widget(y_axis, row=0, col=0)
y_axis.link_view(viewbox)


# vertex positions of data to draw
N = 1000


pos = np.zeros((N, 2), dtype=np.float32)
x_lim = [50., 750.]
y_lim = [-2., 2.]

NUM_LINES = 2
lines = []

i = 0
pos[:, 0] = np.linspace(x_lim[0], x_lim[1], N)
pos[:, 1] = np.sin(np.linspace(-np.pi, np.pi, N) + i / float(NUM_LINES))
# add a line plot inside the viewbox
scene.Line(pos, parent=viewbox.scene)

pos[:, 0] = np.linspace(x_lim[0], x_lim[1], N)
pos[:, 1] = np.ones(N)
# add a line plot inside the viewbox
scene.Line(pos, parent=viewbox.scene)

# color array
# colors = []
# for i in range(NUM_LINES):
#     color = np.ones((N, 4), dtype=np.float32)
#     color[:, 0] = random.random()  # np.linspace(0, 1, N)
#     color[:, 1] = color[::-1, 0]
#     colors.append(color)
#
#     pos[:, 0] = np.linspace(x_lim[0], x_lim[1], N)
#     pos[:, 1] = np.sin(np.linspace(-np.pi, np.pi, N) + i / float(NUM_LINES))
#     # add a line plot inside the viewbox
#     scene.Line(pos, color=color, parent=viewbox.scene)

# auto-scale to see the whole line.
viewbox.camera.set_range()

offset = 0
start = time.time()
def update(ev):
    return
    global pos, colors, lines, offset, start
    print(time.time() - start)
    start = time.time()
    for i, line in enumerate(lines):
        pos[:, 1] = np.sin(np.linspace(-np.pi, np.pi, N) + (i + offset / 10) / float(NUM_LINES))
        # color = np.roll(color, 1, axis=0)
        line.set_data(pos=pos, color=colors[i])

    offset += 1


# timer = app.Timer()
# timer.connect(update)
# timer.start(0)

if __name__ == '__main__' and sys.flags.interactive == 0:
    app.Timer(connect=update)
    app.run()