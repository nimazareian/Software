# With help from https://github.com/vispy/vispy/blob/master/examples/basics/visuals/line.py
# and https://stackoverflow.com/questions/34884302/efficiently-plotting-many-lines-in-vispy 

import numpy as np
from vispy import scene

bg_clr = (0.1, 0.1, 0.1)  # dark background color

class plot:

    def __init__(self, curves):
        self.canvas = scene.SceneCanvas(keys='interactive', bgcolor=bg_clr )

        self.grid = self.canvas.central_widget.add_grid(spacing=0)
        self.view = self.grid.add_view(row=0, col=1, camera='panzoom')

        N,S = curves.shape

        # the Line visual requires a vector of X,Y coordinates 
        xy_curves = np.dstack( (np.tile(np.arange(S), (N,1)), curves) )

        # Specify which points are connected
        # Start by connecting each point to its successor
        connect = np.empty((N*S-1,2), np.int32)
        # [0, 0] = 0, [1, 0] = 1, [2, 0] = 2, ..., [NS-2, 0] = NS-2
        connect[:, 0] = np.arange(N*S-1)
        # [0, 1] = 1, [1, 1] = 2, [2, 1] = 3, ..., [NS-2, 1] = NS-1
        connect[:, 1] = connect[:, 0] + 1

        # Prevent vispy from drawing a line between the last point 
        # of a curve and the first point of the next curve 
        for i in range(S, N*S, S):
            # The S-1th point will be connected only to itself
            # connect[S-1, 0] == connect[S-1, 1] == S-1
            connect[i-1, 1] = i-1

        scene.Line(pos=xy_curves, parent=self.view.scene, connect=connect)

        self.x_axis = scene.AxisWidget(orientation='bottom')
        self.y_axis = scene.AxisWidget(orientation='left')
        self.x_axis.stretch = (1, 0.05)
        self.y_axis.stretch = (0.05, 1)
        self.grid.add_widget(self.x_axis, row=1, col=1)
        self.grid.add_widget(self.y_axis, row=0, col=0)
        self.x_axis.link_view(self.view)
        self.y_axis.link_view(self.view)

        self.view.camera.set_range()

        self.canvas.show()
        self.canvas.app.run()

if __name__ == "__main__":
    a = np.random.normal(0.,0.5,(256,20000))
    p = plot(np.array([c-4*i for i,c in enumerate(a)]))