# Launch Files

A launch file is a file that allows you to start multiple nodes with a single command, without having to start each node in an individual terminal window.

The launch file is greatly useful to configure complex systems, as it allows you to start multiple nodes with specific parameters, monitor the output of the nodes, and stop/react to the nodes' behavior.

Launch files can be written in Python, XML, or YAML.
We will focus on Python launch files, as they are more flexible and easier to read than XML and YAML launch files.

## Create a Launch File

For example, let's create a launch file for the `talker` and `listener` nodes.

Start by creating a new folder where to store the launch files:

```bash
mkdir launch
```

Then, create a new Python file inside the `launch` folder named `launch.py` with the following content:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_sub',              # package name
            executable='pub_sub_listener',  # executable name
            name='listener'                 # node name
        ),
        Node(
            package='pub_sub',
            executable='pub_sub_talker',
            name='talker'
        )
    ])
```

Note that the first lines import the necessary modules, namely `LaunchDescription` and `Node`.
The `generate_launch_description` function returns a `LaunchDescription` object that contains the nodes to be launched.

The `Node` class is used to create a node, and it requires the package name, the executable name, and the node name as arguments.
The executable name is the name given to the `entry point` in the `setup.py` file.
The node name is the name that will be displayed in the ROS 2 graph.
Multiple instances of the same executable node can be created by changing the `name` argument (e.g., `name='talker1'`, `name='talker2'`, etc.).

## Integrate a Launch File in a Package

To integrate the launch file into the package, you must update the `setup.py` file of the package.
Specifically, you must include the launch file in the `data_files` list.
The `data_files` list specifies the files that should be installed with the package.
By default, `data_files` includes the resource files and the `package.xml` file.
To include the launch file, you must add the following line to the `data_files` list:

```python
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
```

This line specifies that all files containing the word `launch` and ending with `.py`, `.xml`, or `.yaml` in the `launch` directory should be included in the package.
Since the Python libraries `os` and `glob` are used in the line, you must import them at the beginning of the `setup.py` file:

```python
import os
from glob import glob
# Other imports ...
```

Note that the new import statements are placed *before* the other import statements.

## Run the Launch File

To run the launch file, move to the `launch` directory and run the following command:

```bash
ros2 launch <launch_file_name>.py
```

If we want to run a launch file provided by a package, we must specify the package name:

```bash
ros2 launch <package_name> <launch_file_name>.py
```
