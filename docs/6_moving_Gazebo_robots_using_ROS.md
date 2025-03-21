# Moving Gazebo Robots Using ROS

In this tutorial, we will learn how to move a robot in Gazebo using ROS.
First, we will see how to move a robot in Gazebo using Gazebo's built-in tools.
Then, we will see how to interface Gazebo with ROS to move a robot using ROS topics and messages.

## Moving a Robot in Gazebo

To move a robot in Gazebo, the `diff_drive` plugin can be used.
Plugins are pieces of code that enable the user to control many aspects of a Gazebo simulation.
The `diff_drive` plugin is a plugin that allows you to control a robot with differential drive (i.e., a robot whose movement is controlled by varying the speeds of two independently driven wheels mounted on the same axis).

Plugins are added to the SDF file as `<plugin>` tags.
For instance, the following lines add the `diff_drive` plugin to the `vehicle_blue` model we introduced earlier.
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

where `rover_world_with_diff_drive.sdf` is the SDF file that contains the `rover_blue` model with the `diff_drive` plugin.
In another terminal window, the robot can be moved by publishing velocity commands to the `cmd_vel` topic:

```bash
ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

The command above uses several flags:

- `-t`: Specifies the topic to publish the message to.
- `-m`: Specifies the message type to publish.
- `-p`: Specifies the content of the message.

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
