# Install the `turtlesim` package 

The `turtlesim` package is a simple demonstration package that provides a turtle that can be controlled using ROS 2 commands.
This package is useful for learning the basics of ROS 2, as it illustrates what ROS 2 does at the most basic level and it gives an idea of what you can do with ROS 2 when working with real or simulated robots.

After sourcing the ROS 2 environment, you can install the `turtlesim` package using the following command:

```bash
sudo apt update
sudo apt install ros-iron-turtlesim
```

To check if the `turtlesim` package is installed correctly, run the following command:

```bash
ros2 pkg executables turtlesim
```

This command will display the executables provided by the `turtlesim` package.
Specifically,

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

## Starting the `turtlesim` node

To start the `turtlesim` node, you need to launch the `turtlesim_node` executable.

### Launch an Executable

The command `ros2 run` launches an executable in a ROS 2 package.
An executable is a file that can be run as a program.
In ROS 2, executables are used to run nodes and other programs (e.g., tools, GUIs) that are part of a package.

The syntax is as follows:

```bash
ros2 run <package_name> <executable_name>
```

### Start the `turtlesim` Node

To start the `turtlesim` node, run the following executable:

```bash
ros2 run turtlesim turtlesim_node
```

The simulator window will open, displaying a turtle in the center of the window.
In the terminal, you will see the output of the `turtlesim` node.

## Control the turtle using the keyboard

To control the turtle using the keyboard, you can run the `turtle_teleop_key` executable.
In a new terminal window, run the following command:

```bash
ros2 run turtlesim turtle_teleop_key
```

You can now use the keyboard to control the turtle in the simulator window.

💡 **Tip:** Arrange the terminal windows and the simulator window so that you can see and interact with both at the same time.
