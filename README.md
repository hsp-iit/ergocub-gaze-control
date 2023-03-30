# icub-bimanual
Control classes for 2-handed control of the iCub robot

# Prerequisites
This repository utilizes:
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page),
- [Robotology Superbuild](https://github.com/robotology/robotology-superbuild),
- [iCub Models](https://github.com/robotology/icub-models), and
- [Gazebo](https://gazebosim.org/home) for simulation.

Follow the links to install each of these.

# Installation

1. Clone the repository to your workspace:
```
git clone https://github.com/Woolfrey/icub-bimanual.git
```

2. Navigate to the directory:
```
cd icub-bimanual/
```

3. Create a new `build` directory and navigate to it:
```
mkdir build && cd build
```

4. Then build the repository:
```
cmake ../ && make
```

# Running a demo in Gazebo

1. Open a new terminal in Ubuntu with `ctrl + alt + t` and start the YARP server:
```
yarpserver
```

  If this fails to launch, try using:
```
yarpserver --write
```

2. Open *another* terminal and run:
```
gazebo ~/your_workspace_directory/icub-bimanual/gazebo/words/grasp-demo.sdf
```

  A model of the iCub2 should launch in Gazebo.

3. In a *third* terminal, navigate to `~/icub-bimanual/build` and run:
```
./bin//grasp-demo ~/directory/to/urdf
```

If you have installed the `icub-models` repository, then you can use:
```
./bin/grasp-demo ~/your_workspace_directory/icub-models/iCub/robots/iCubGazeboV2_7/model.urdf
```

If successful, then a yarp port `/command` should be open for communication.

4. In a *fourth* terminal, run:
```
yarp rpc /command
```

You can now type commands for the robot to execute.

## List of Commands
In the terminal where you can `yarp rpc /command`, you can type the following:
- **stop**: stops any current action so that the robot holds its current joint configuration
- **home**: lowers the arms to a resting configuration
- **wave**: makes the robot wave with its right hand,
- **shake**: makes the robot extend its right hand,
- **ready**: moves the arms to a position ready for grasping
- **grasp**: moves the hands together in a grasp pose
- **release**: moves the hands apart and back to the home positionm
- **in**: moves the hands from their current positionsinward
- **out**: moves the hands from their current position outward
- **up**: moves the hands from their current position upward
- **down**: moves the hands from their current position downward
- **fore**: moves the hands from their current position forward
- **aft**: moves the hands from their current positions backward
