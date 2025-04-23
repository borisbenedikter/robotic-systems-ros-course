# ROS and Python

ROS 2 is designed to be used with Python, and many of the tools and libraries provided by ROS 2 are written in Python.
In theory, you can use any Python library with ROS 2, as long as the library is compatible with the version of Python used by ROS 2.

## Python Version Used by ROS 2

To find out which version of Python is used by ROS 2, you can check in the folder `/opt/ros/<distro>/lib/`.
There, you will find a folder named `python3.x`, where `x` is the version of Python used by ROS 2.
For example, if you are using ROS 2 Iron, you can enter

```bash
ls /opt/ros/iron/lib
```

and you will see a folder named `python3.10`.

## Conda Environments

A very useful tool for managing Python packages is `conda`.
`conda` is a package manager that allows you to create isolated environments with specific versions of Python and packages.
This is useful for testing different versions of packages or for creating a clean environment for a specific project.

For a ROS 2 project, you should create a conda environment with the same version of Python used by ROS 2.
To create a conda environment with Python 3.10, you can use the following command:

```bash
conda create --name myenv python=3.10
```

This will create a new conda environment named `myenv` with Python 3.10.
To activate the environment, you can use the following command:

```bash
conda activate myenv
```

This will activate the `myenv` environment in your terminal.

However, by default, when launching a ROS 2 node via launch files or `ros2 run`, the Python interpreter used is the one installed in the system, not the one in the conda environment.
If the nodes depend on packages installed only in the conda environment and that we do not want to install in the system, we need to be sure that the Python interpreter used is the one in the conda environment.

A workaround is to create a wrapper launch script that activates the conda environment and then launches the ROS 2 node.

## Wrapper Launch Scripts

Let us consider a generic python file `node.py` that defines a ROS 2 executable node and depends on some packages installed in the conda environment `myenv` and not in the system.
To be sure that the conda environment is activated when launching the node, we can create a wrapper launch script that activates the conda environment and then launches the node.
For example,

```bash
#!/bin/bash
# Activate the conda environment
conda activate myenv

# Move to the workspace directory
cd /path/to/your/workspace

# Launch the ROS 2 node
python src/package/package/node.py
```

This script activates the conda environment `myenv`, moves to the workspace directory, and then launches the ROS 2 node defined in the file `node.py` located in the folder `src/package/package/`.
The script can be saved in the `launch` folder of the package or in another folder (e.g., `launch_wrappers`).
The script can be named `launch_node.sh` and made executable with the command:

```bash
chmod +x path/to/launch_node.sh
```

Then, you must create a ROS 2 launch file that launches the wrapper script instead of the node directly.
To do this, instead of using the `Node` class to launch the node, you can use the `ExecuteProcess` class to execute the wrapper script.
For example, the launch file `launch.py` that launches the node defined in `node.py` would look like this:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package',
            executable='node',
            name='node'         
        ),])
```

By replacing the `Node` class with the `ExecuteProcess` class, the launch file would look like this:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['/path/to/launch_wrappers/launch_node.sh'],
            shell=True,
            output='screen'
        )])
```

This will execute the `launch_node.sh` script, which activates the conda environment and then launches the ROS 2 node defined in `node.py`.
Note that the `cmd` argument specifies the command to be executed, the `shell=True` argument indicates that the command should be executed in a shell, and the `output='screen'` argument specifies that the output should be displayed on the screen.
Note that `ExecuteProcess` is imported from `launch.actions` instead of `launch_ros.actions`.

## Activation Scripts

When activating a conda environment, you may need to set some environment variables or run some commands to configure the environment properly.
This can be done by creating an activation script that is executed every time the conda environment is activated via the `conda activate` command.

Activation scripts are shell scripts that are written by the user and saved in the `etc/conda/activate.d` folder of the conda environment.
To locate the folder of a conda environment, you can use the command:

```bash
ls $(conda info --base)/envs/
```

The `(conda info --base)` command returns the base directory of the conda installation (e.g., `/home/user/miniconda3`).
That directory contains a folder named `envs` that contains one folder for each conda environment ever created.
The `ls` command lists the folders in the `envs` directory, so you can see the folders of all the conda environments.

Activation scripts must be saved in the `etc/conda/activate.d` folder of the conda environment.
This folder is not created by default, so you must create it manually.
To create the folder, you can use the following command:

```bash
mkdir -p $(conda info --base)/envs/myenv/etc/conda/activate.d
```

where `myenv` is the name of the conda environment.
Then, you can create a shell script in the `activate.d` folder with the commands to be executed when the conda environment is activated.

For example, ROS libraries may be incompatible with the version of the C++ compiler installed in the conda environment.
Instead, you know that ROS libraries are compatible with the version of the C++ compiler installed in your system.
Thus, you may want to use the version of the C++ compiler installed in your system instead of the one in the conda environment.
To do this, you need to modify the `LD_PRELOAD` environment variable to point to the version of the C++ compiler installed in your system.
The `LD_PRELOAD` environment variable is used to specify shared libraries that should be loaded before any other libraries when a program is executed.
To change its value, you can create a shell script in the `activate.d` folder with the following content:

```bash
#!/bin/bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
```

This script sets the `LD_PRELOAD` environment variable to point to the version of the C++ compiler installed in your system.
When the conda environment is activated, this script will be executed, and the `LD_PRELOAD` environment variable will be set to the specified value.
The script must be made executable with the command:

```bash
chmod +x $(conda info --base)/envs/myenv/etc/conda/activate.d/script.sh
```

where `script.sh` is the name of the script. This ensures that the activation script is executable and will run correctly when the conda environment is activated.
