# ur5robotiq

This package enables an UR5 arm with a Robotiq 85 Gripper to be used with MoveIt!.

It depends from the packages from the following repositories:

https://github.com/ros-industrial/robotiq

https://github.com/ros-industrial/universal_robot

## Building

The packages is available for ROS Melodic and you have to build the packages from source.

### Building the packages

The following instructions assume that a [Catkin workspace][] has been created at `$HOME/catkin_ws` and that the *source space* is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

These instructions build the package on a ROS Melodic system:

```bash
$ cd $HOME/catkin_ws/src

# retrieve the sources
$ git clone git@github.com:VictorWilvert/ur5robotiq.git

$ cd $HOME/catkin_ws

# checking dependencies
$ rosdep update
$ rosdep install --rosdistro melodic --ignore-src --from-paths src

# building
$ catkin_make
```

### Activating the workspace

Finally, activate the workspace to get access to the packages just built:

```bash
# activate this workspace
$ source $HOME/catkin_ws/devel/setup.bash
```

At this point all packages should be usable.

## Running The Simulation

Open two terminals. In the first terminal start RViz and wait for everything to finish loading:

```bash
$ roslaunch ur5robotiq_moveit_config demo.launch
```

In the second terminal run the pick and place simulation:

```ba/sh
$ rosrun ur5robotiq_manipulation pick_and_place
```