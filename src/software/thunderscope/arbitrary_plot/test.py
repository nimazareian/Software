import sys
import numpy as np
from vispy import app, scene, gloo
from vispy.color import Colormap, ColorArray

MAX_X = 1000
MAX_PLOTS = 6
INTERVAL=0
class Canvas(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self, keys='interactive', size=(800, 800), bgcolor="white", show=True, vsync=True)
        self.unfreeze()
        self.measure_fps()

        self.pos = np.zeros((MAX_X * MAX_PLOTS, 2), dtype=np.float32)
        self.x = np.linspace(0, MAX_X, num=MAX_X, endpoint=True)

        self.y_list = []
        self.y_list.append(np.sin(self.x * 8 * np.pi / MAX_X) * 20 + 200)
        self.y_list.append(np.sin(self.x * 7 * np.pi / MAX_X) * 50 + 200)
        self.y_list.append(np.sin(self.x * 5 * np.pi / MAX_X) * 100 + 200)

        self.y_list.append(np.sin(self.x * 11 * np.pi / MAX_X) * 20 + 400)
        self.y_list.append(np.sin(self.x * 13 * np.pi / MAX_X) * 50 + 400)
        self.y_list.append(np.sin(self.x * 17 * np.pi / MAX_X) * 100 + 400)
        self.x = np.tile(self.x, MAX_PLOTS)
        self.pos[:, 0] = self.x

        cm = Colormap(['r', 'g', 'b'])

        color_indices = np.concatenate(
            ([0.0] * MAX_X, [0.5] * MAX_X, [1.0] * MAX_X, [0.0] * MAX_X, [0.5] * MAX_X, [1.0] * MAX_X))
        colors = cm[color_indices]

        #create connection array separating lines from each other
        connect = []
        for i in range(MAX_PLOTS):
            start = i * MAX_X
            temp = np.empty((MAX_X - 1, 2), np.float32)
            temp[:, 0] = np.arange(start=start,stop=start+MAX_X - 1)
            temp[:, 1] = temp[:, 0] + 1
            connect.extend(temp.tolist())
        connect=np.array(connect)

        self.line = scene.Line(connect=connect,width=2, color=colors,parent=self.scene)

        self._timer = app.Timer(INTERVAL, connect=self.on_timer, start=True)

    def on_timer(self, event):
        data=np.concatenate(self.y_list)
        for i in range(MAX_PLOTS):
            self.y_list[i] = np.roll(self.y_list[i], -1)

        self.pos[:, 1] = data
        self.line.set_data(pos=self.pos)
        self.update()

if __name__ == '__main__' and sys.flags.interactive == 0:
    win = Canvas()
    app.run()