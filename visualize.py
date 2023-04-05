import yarp

yarp.Network.init()

input_port = yarp.BufferedPortBottle()
input_port.open("/GazeController/Visualizer/debug:i")

yarp.Network.connect("/GazeController/debug:o", "/GazeController/Visualizer/debug:i")

while True:
    bottle = input_port.read(False)
    if bottle is not None:
        point = bottle.get(0).asList()
        vector = bottle.get(1).asList()

        print("Point:", point.toString())
        print("Vector:", vector.toString())