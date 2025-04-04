# Moving Gazebo Robots Using ROS

In this tutorial, we will learn how to move a robot in Gazebo using ROS.
First, we will see how to move a robot in Gazebo using Gazebo's built-in tools.
Second, we will explore how to get the position and orientation of a robot in Gazebo using the `PosePublisher` plugin.
Finally, we will discuss the ROS-Gazebo bridge, which allows ROS nodes to publish and subscribe to Gazebo topics and messages. This bridge enables seamless communication between ROS and Gazebo, allowing you to control Gazebo simulations using ROS messages.

## Moving a Robot in Gazebo

To move a robot in Gazebo, the `diff_drive` plugin can be used.
Plugins are pieces of code that enable the user to control many aspects of a Gazebo simulation.
The `diff_drive` plugin is a plugin that allows you to control a robot with differential drive (i.e., a robot whose movement is controlled by varying the speeds of two independently driven wheels mounted on the same axis).

Plugins are added to the SDF file as `<plugin>` tags.
For instance, the following lines add the `diff_drive` plugin to the `rover_blue` model we introduced in [the previous example](4_visualization_gazebo.md#rover-robot-chassis):

These lines should be added within the `<model>` tag:

```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation> <!-- (-0.6 to 0.6) -->
    <wheel_radius>0.4</wheel_radius>       <!-- Cylinder radius -->
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

The `<plugin>` tag has two attributes: `filename` and `name`.
The `filename` attribute specifies the name of the plugin library, which is a compiled shared library (`.so` file).
The `name` attribute specifies the name of the plugin.
The `diff_drive` plugin has several parameters that must be set:

- `<left_joint>`: The name of the left wheel joint.
- `<right_joint>`: The name of the right wheel joint.
- `<wheel_separation>`: The distance between the left and right wheels.
- `<wheel_radius>`: The radius of the wheels.
- `<odom_publish_frequency>`: The frequency (in Hz) at which the odometry data is published.
- `<topic>`: The name of the input topic that controls the robot's movement.

Thus, the `diff_drive` plugin creates a topic called `cmd_vel`, where odometry data is published at a frequency of 1 Hz.
The model is also a subscriber to the `cmd_vel` topic, so that it can receive velocity commands and move accordingly.
The topics are very similar to ROS topics, but they are specific to Gazebo.

In one terminal window, the robot world can be started by running the following command:

```bash
ign gazebo --render-engine ogre <path/to>/rover_world_with_diff_drive.sdf
```

where `rover_world_with_diff_drive.sdf` is the SDF file that contains the `rover_blue` model with the `diff_drive` plugin, which can be found [here](../examples/gazebo/rover_world_with_diff_drive.sdf).
In another terminal window, the robot can be moved by publishing velocity commands to the `cmd_vel` topic:

```bash
ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

The command above uses several flags:

- `-t`: Specifies the topic to publish the message to.
- `-m`: Specifies the message type to publish.
- `-p`: Specifies the content of the message.

The `ign topic` command allows you to list all available Gazebo topics by using the `-l` flag:

```bash
ign topic -l
```

To know the meaning of all available flags, you can run the following command:

```bash
ign topic --h
```

After running the command to publish velocity commands, the robot should move in Gazebo as soon as the `Play` button is pressed.

The `diff_drive` plugin can control a robot with multiple pairs of wheels, not just two wheels.
For example, the plugin for a six-wheeled robot would look like this:

```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_front_wheel_joint</left_joint>     <!-- 1 Left -->
    <right_joint>right_front_wheel_joint</right_joint>  <!-- 1 Right -->
    <left_joint>left_middle_wheel_joint</left_joint>    <!-- 2 Left -->
    <right_joint>right_middle_wheel_joint</right_joint> <!-- 2 Right -->
    <left_joint>left_back_wheel_joint</left_joint>      <!-- 3 Left -->
    <right_joint>right_back_wheel_joint</right_joint>   <!-- 3 Right -->
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

## Getting Robot Position and Orientation in Gazebo

To get the position and orientation of a robot in Gazebo, you can use the `pose-publisher` plugin provided by Gazebo.
The `pose-publisher` plugin creates a topic where the position and orientation of the robot are published at a specified frequency.
The `pose-publisher` plugin can be added to any model in the SDF file as follows:

```xml
<model name="rover_blue">
    <!-- Other tags... -->
    <plugin 
        filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>        <update_frequency>1</update_frequency> 
    </plugin>
    <!-- Other tags... -->
</model>
```

In this example, the `PosePublisher` plugin is added to the `rover_blue` model.
The `<plugin>` tag has several parameters that can be set:
- `filename`: The name of the plugin library, which is `libignition-gazebo-pose-publisher-system.so`.
- `name`: The name of the plugin, which is `ignition::gazebo::systems::PosePublisher`.
- `<publish_link_pose>`: If set to `true`, it will publish the pose of every link in the model. In this case, it is set to `false` to publish only the pose of the model itself.
- `<publish_nested_model_pose>`: If set to `true`, it will publish the pose of nested models (models included in the parent model). In this case, it is set to `true` to publish the pose of the included models.
- `<update_frequency>`: The frequency (in Hz) at which the pose is published. In this case, it is set to `1 Hz`.

Once the `PosePublisher` plugin is added to the SDF file, you can start Gazebo with the updated SDF file:

```bash
ign gazebo --render-engine ogre <path/to>/car_world_with_pose_publisher.sdf
```

Then, you can list the available topics in Gazebo to find the pose topic:

```bash
ign topic -l
```

The output will show a list of all the topics being published by Gazebo, including the pose topic, for example:

```plaintext
/clock
/gazebo/resource_paths
/gui/camera/pose
/model/rover_blue/odometry
/model/rover_blue/pose
/model/rover_blue/tf
/stats
/world/car_world/clock
/world/car_world/dynamic_pose/info
/world/car_world/pose/info
/world/car_world/scene/deletion
/world/car_world/scene/info
/world/car_world/state
/world/car_world/stats
```

Note that the name of the topic is `/model/<model_name>/pose`, where `<model_name>` is the name of the model you added the `PosePublisher` plugin to (in this case, `rover_blue`).

To subscribe to the pose topic and see the messages being published, you can use the `ign topic` command as follows:

```bash
ign topic -t /model/rover_blue/pose -e
```

where the `-e` flag echoes the messages to the terminal as they are received.

To get some info on the message type being published on the `/model/rover_blue/pose` topic, you can use the following command:

```bash
ign topic -t /model/rover_blue/pose -i
```

where the `-i` flag stands for "info" and will display the message type being published on the specified topic, which in this case will typically be `ignition.msgs.Pose` or a similar message type depending on the version of Gazebo you are using.

## The ROS-Gazebo Bridge

`ros_gz_bridge` provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo Transport, which is the communication component used by Gazebo.
Its support is limited to only certain message types, meaning that not all messages can be exchanged between ROS 2 and Gazebo Transport.
To check if a message type is supported, you can refer to this [README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) file from the `ros_gz_bridge` official repository.

### Installing `ros_gz_bridge`

To install `ros_gz_bridge`, you can use the following command:

```bash
sudo apt install ros-<ros2-distro>-ros-gz-bridge
```

Make sure to replace `<ros2-distro>` with the name of your ROS 2 distribution (e.g., `iron`, `humble`, etc.). For example, if you are using ROS 2 Iron, the command would be:

```bash
sudo apt install ros-iron-ros-gz-bridge
```

### Launching a ROS-Gazebo Bridge from the Command Line

To launch a bridge, the `parameter_bridge` executable from ROS2's `ros_gz_bridge` package can be used.
Therefore, after sourcing your ROS 2 installation (e.g., `source /opt/ros/iron/setup.bash`), you can use the following syntax to start the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge <GZ_TOPIC>@<ROS_MSG_TYPE>@<GZ_MSG_TYPE>
```

where `<GZ_TOPIC>` is the name of the Gazebo topic to bridge, `<ROS_MSG_TYPE>` is the ROS2 message type, and `<GZ_MSG_TYPE>` is the Gazebo message type.

The first `@` symbol is used simply to separate the Gazebo topic from the ROS2 message type, while the second `@` symbol is used to define the direction of the bridge.
Indeed, the bridge can be unidirectional or bidirectional, according to the symbol used after the `<ROS_MSG_TYPE>`.
The following symbols can be used to define the direction of the bridge:

- `@`: Bidirectional bridge between ROS and Gazebo.
- `[`: Unidirectional bridge from Gazebo to ROS.
- `]`: Unidirectional bridge from ROS to Gazebo.

The ROS and Gazebo message types must be specified in specific formats:

- ROS message types are specified in the format `<package_name>/msg/<msg_type>`, where `<package_name>` is the name of the package that contains the message type, and `<msg_type>` is the name of the message type. For instance, the ROS message type for a `Twist` message is `geometry_msgs/msg/Twist`.
- Gazebo message types are specified in the format `<package_name>.msgs/<msg_type>`, where `<package_name>` is the name of the package that contains the message type, and `<msg_type>` is the name of the message type. For instance, the Gazebo message type for a `Twist` message is `gz.msgs.Twist`.

The name and format of the ROS and Gazebo message types can be found in the [README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) previously mentioned.

### Launching a ROS-Gazebo Bridge Using a Launch File

A bridge can be launched also within a ROS2 launch file, which is a convenient way to start multiple bridges at once.
Here is an example of a `parameter_bridge` node that can be included in a ROS2 launch file:

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/rover_blue_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
    ],
    output='screen',  # Optional: to see the output in the terminal
    name='bridge_cmd_vel'  # Name of the parameter bridge node
),
```

This example shows how to create a `Node` in a ROS2 launch file that runs the `parameter_bridge` executable to bidirectionally bridge the `/rover_blue_cmd_vel` Gazebo topic which receives `gz.msgs.Twist` Gazebo messages that are mapped to the `geometry_msgs/msg/Twist` ROS2 message type.
