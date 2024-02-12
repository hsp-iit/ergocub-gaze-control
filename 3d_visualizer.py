import copy
from multiprocessing import Process
import os
import sys
import time

# sys.path.append('/robotology-superbuild/build/install/lib/python3.8/site-packages')
import yarp
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
from vispy import scene, app
from vispy.scene import ViewBox, Markers, SurfacePlot

class ObjectDetection3DVisualizer(Process):
    def __init__(self, local, remote):
        super().__init__()
        self.vis_R1 = Rotation.from_euler('x', -90, degrees=True).as_matrix()
        self.vis_R2 = Rotation.from_euler('x', 90, degrees=True).as_matrix()

        self.vb1, self.vb2 = None, None
        self.local, self.remote = local, remote
        
        
    def run(self):
        canvas = scene.SceneCanvas(keys='interactive')
        canvas.size = 1200, 600
        canvas.show()
        
        yarp.Network.init()
        self.input_port = yarp.BufferedPortBottle()
        self.input_port.open(self.local)
        while not yarp.Network.connect(self.remote, self.local):
            time.sleep(0.1)
        print('Connected')

        grid = canvas.central_widget.add_grid()

        #  Left Viewbox
        vb1 = ViewBox()
        self.vb1 = grid.add_widget(vb1)

        #  Creating and linking the cameras
        vb1.camera = scene.TurntableCamera(elevation=0, azimuth=0, distance=1)

        self.point1 = Markers(size=10, parent=vb1.scene)
        self.point2 = Markers(size=10, parent=vb1.scene)
        self.r_hand = scene.XYZAxis(parent=vb1.scene, width=10)
        
        
        while True:
            bottle = self.input_port.read(False)
            if bottle is not None:
                desired = bottle.get(0).asList()
                actual = bottle.get(1).asList()

                if desired is not None:
                    desired_point = np.array([[desired.get(i).asFloat64() for i in range(3)]])
                    self.point1.set_data(desired_point, edge_color='orange', face_color='orange', size=50)
                    
                if actual is not None:
                    actual_point = np.array([[actual.get(i).asFloat64() for i in range(3)]])
                    self.point2.set_data(actual_point, edge_color='blue', face_color='blue', size=50)

            time.sleep(0.01) 
            app.process_events()


if __name__ == '__main__':
    source = ObjectDetection3DVisualizer(local="/GazeController/Visualizer/deb:i", 
                                         remote="/GazeController/debug:o")
    
    source.start()
    source.join()
