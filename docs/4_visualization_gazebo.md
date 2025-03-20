# Visualization: Gazebo

## RViz and Gazebo

The ROS default visualization tool is RViz.
RViz is a 3D visualization tool for ROS that allows you to visualize the robot model, sensor data, and other information in a 3D environment.

However, RViz is not the only visualization tool available in ROS.
In fact, the most popular visualization tool is Gazebo, which is a robot simulation tool that is owned by Open Robotics, the same organization that owns ROS.
Gazebo is more popular because it is a full-fledged physics-based simulation environment that offers a more comprehensive environment for developing, testing, and validating robotic applications before deploying them in the real world.

## Installing Gazebo

To install Gazebo, you can follow the instructions provided in this repository's [installation guide](installation_gazebo.md).

## Unified Robot Description Format (URDF)

The Unified Robot Description Format (URDF) is an XML-based format used to describe the physical and visual properties of a robot: its links, joints, inertial properties (mass, inertia, etc.), visual (geometry, material, etc.), collision models, and other attributes.

## Simulation Description Format (SDF)

The Simulation Description Format (SDF) is an XML-based format used primarily in Gazebo to define robots, environments, and simulations.
Unlike URDF, which is mainly used for robot modeling and kinematics in ROS, SDF provides a more comprehensive specification that includes physics, sensors, and world descriptions.

For instance, the additional available fields in SDF include `friction`, `damping`, `gravity`, and other physical properties.
Also, the field `sensor` allows you to model sensors such as cameras, LiDARs, and IMUs.
The SDF format is more suitable for defining the complete world environment, providing fields like `light`, `ground_plane`, and other world properties.

If an URDF file is available for a robot, it can be included in an SDF file without having to convert the URDF to SDF.

In the following sections, we will build a simple SDF file step by step.
The example will include a simple rover robot in a plain world environment.

### World

The most general element in an SDF file is the `<world>` element.
The `<world>` element contains:

- `<physics>`: The properties of the physics engine used in the simulation.
- `<light>`: The properties of the light sources in the world.
- `<model>`: The models in the world (e.g., robots, objects, the ground plane, obstacles).
- `<plugin>`: The plugins that extend the functionality of Gazebo (e.g., physics engines, sensors, user interfaces).

### Physics

The `<physics>` element defines the properties of the physics engine used in the simulation.
An example of a simple physics element is shown below:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <!-- Define the world environment -->
    <world name="rover_world">
        
        <!-- Physics settings for the simulation -->
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size> <!-- Simulation step size -->
            <real_time_factor>1.0</real_time_factor> <!-- Speed relative to real-time -->
        </physics>

    </world>
</sdf>
```

In this example, the `<physics>` element specifies the simulation step size and the real-time factor.
Other properties that can be defined in the `<physics>` element (e.g., gravity) are inherited from Gazebos's default physics engine, which is added to the world as a plugin.

### Plugins

Plugins are used to extend the functionality of Gazebo.
For instance, plugins can be used to add sensors, controllers, or custom physics engines to the simulation.
Plugins are defined in the `<plugin>` element, which is a child of the `<world>` element.
Useful plugins include:

- **libignition-gazebo-physics-system.so**: For physics simulation.
- **libignition-gazebo-user-commands-system.so**: For user commands.
- **libignition-gazebo-scene-broadcaster-system.so**: For broadcasting the scene to GUI clients.

The following lines show how to add the aforementioned plugins to the world:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <!-- Define the world environment -->
    <world name="rover_world">

        <!-- Plugins to extend Gazebo functionalities -->
        <!-- Default physics engine -->
        <plugin
            filename="libignition-gazebo-physics-system.so" 
            name="ignition::gazebo::systems::Physics">
        </plugin>
        <!-- User commands for controlling the simulation -->
        <plugin 
            filename="libignition-gazebo-user-commands-system.so" 
            name="ignition::gazebo::systems::UserCommands">
        </plugin>        
        <!-- Broadcasts the scene to GUI clients -->
        <plugin 
            filename="libignition-gazebo-scene-broadcaster-system.so" 
            name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>

    </world>
</sdf>
```

### Light

The `<light>` element defines the properties of the light sources in the world.
The following example shows how to define a light source that models the sun:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <!-- Define the world environment -->
    <world name="rover_world">

        <!-- Define a directional light (e.g., sun) -->
        <light type="directional" name="sun">
            <!-- Enable shadow casting -->
            <cast_shadows>true</cast_shadows> 
            <!-- Position in the world -->
            <!-- <pose> x y z roll pitch yaw </pose> -->
            <pose>0 0 10 0 0 0</pose> 
            <!-- Light direction (mostly downward) -->
            <direction>-0.5 0.1 -0.9</direction>
            <!-- Diffuse (i.e., scattered) light -->
            <!-- <diffuse> R G B alpha </diffuse> -->
            <!-- Fully opaque light gray -->
            <diffuse>0.8 0.8 0.8 1</diffuse> 
            <!-- Shiny highlights on reflective surfaces -->
            <!-- Fully opaque dark gray -->
            <specular>0.2 0.2 0.2 1</specular>
            <!-- Attenuation over distance -->
            <attenuation>
                <!-- Max distance light travels -->
                <range>1000</range>
                <!-- Quadratic fun of distance -->
                <!-- Light = 1 / (C + L * d + Q * d^2) -->
                <constant>0.9</constant>    <!-- C -->
                <linear>0.01</linear>       <!-- L -->
                <quadratic>0.001</quadratic> <!-- Q -->
            </attenuation>
        </light>

    </world>
</sdf>
```

In this example, the `<light>` element defines a directional light source named `sun`.
A directional light source is a light source that emits light in a specific direction, similar to the sun.

The light source is position and orientation in the world is defined by the `<pose>` element.
The `<pose>` element contains the position `(x, y, z)` and the orientation `(roll, pitch, yaw)` of the light source.
The `<direction>` element defines the direction in which the light is emitted.
In this example, the light is directed mostly downward (it would be fully downward if we want to simulate the sun at the zenith).

The light emitted by the light source is defined by the `<diffuse>` and `<specular>` elements.
The `<diffuse>` element defines the scattered light, which is the light that is reflected in all directions.
The `<specular>` element defines the light that is reflected in a specific direction, creating shiny highlights on reflective surfaces.
Both elements contain the RGBA (Red, Green, Blue, Alpha) values that determine the RGB color and transparency of the light.

The `<attenuation>` element defines how the light intensity decreases over distance.
The `<range>` element defines the maximum distance the light travels.
The `<constant>`, `<linear>`, and `<quadratic>` elements define the attenuation function of the light over distance.
The attenuation function is defined as `1 / (C + L * d + Q * d^2)`, where `d` is the distance from the light source, and `C`, `L`, and `Q` are the constant, linear, and quadratic attenuation coefficients, respectively.

### Model

The `<model>` element defines the models in the world.
A model can be a robot, an object, the ground plane, an obstacle, or any other entity in the world.
The `<model>` element is made up of `<link>` elements, connected by `<joint>` elements.
For each `<link>` element, there is a `<collision>` element and a `<visual>` element that define the collision and visual properties of the link, respectively.

#### Ground Plane

For example, the following lines show how to define a simple ground plane model:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <!-- Define the world environment -->
    <world name="rover_world">

        <!-- Define a ground plane -->
        <model name="ground_plane">
            <static>true</static> <!-- The ground does not move -->
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <!-- Flat horizontal surface -->
                            <normal>0 0 1</normal> 
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <!-- Flat horizontal surface -->
                            <normal>0 0 1</normal>
                            <!-- Ground size -->
                            <size>100 100</size> 
                        </plane>
                    </geometry>
                    <material> <!-- Ground color -->
                        <ambient>0.8 0.8 0.8 1</ambient> <!-- Base color -->
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                </visual>
            </link>
        </model>

    </world>
</sdf>
```

In this example, the `<model>` element defines a ground plane model named `ground_plane`.
The ground plane is defined by a single `<link>` element, which contains a `<collision>` element and a `<visual>` element.
The `<collision>` element defines the collision model of the ground plane, while the `<visual>` element defines the visual model of the ground plane.

The collision model is defined by the `<geometry>` element, which contains a `<plane>` element.
The `<plane>` consists of a `<normal>` element that defines the normal vector of the plane, which is `(0, 0, 1)` in this case (a flat horizontal surface).

The visual model is also defined by the `<geometry>` element, which contains a `<plane>` element.
The visual geometry can be different from the collision geometry, as it is used to render the model in the simulation.
The collision geometry is used for physics calculations and collision detection, and it is usually simpler to reduce computational costs of the simulation.
In this example, the visual geometry is the same as the collision geometry, as it is a simple flat horizontal surface.
The `<size>` element defines the size of the ground plane in the x and y directions.

The `<visual>` also contains a `<material>` element that defines the color of the ground plane.
The `<ambient>`, `<diffuse>`, and `<specular>` elements define the RGBA values of the ambient, diffuse, and specular colors of the ground plane.
The ambient color is the base color of the ground plane (i.e., when there is no light shining on it), the diffuse color is the color of the scattered light, and the specular color is the color of the shiny highlights on the ground plane.

#### Rover Robot (Chassis)

The following lines show how to define a simple rover robot model:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <!-- Define the world environment -->
    <world name="rover_world">

        <!-- Define a simple rover model -->
        <model name='rover_blue' canonical_link='chassis'>
            <!-- Position of the rover in the world -->
            <pose relative_to='world'>0 0 0 0 0 0</pose>
            
            <!-- Chassis (main body) of the rover -->
            <link name='chassis'>
                <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
                
                <!-- Inertial properties for physics simulation -->
                <inertial>
                    <mass>1.14395</mass>
                    <inertia>
                        <ixx>0.095329</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.381317</iyy>
                        <iyz>0</iyz>
                        <izz>0.476646</izz>
                    </inertia>
                </inertial>
                
                <!-- Visual representation of the chassis -->
                <visual name='visual'>
                    <geometry>
                        <box>
                            <!-- Chassis dimensions -->
                            <size>2.0 1.0 0.5</size> 
                        </box>
                    </geometry>
                    <material> <!-- Color of the rover -->
                        <ambient>0.0 0.0 1.0 1</ambient>
                        <diffuse>0.0 0.0 1.0 1</diffuse>
                        <specular>0.0 0.0 1.0 1</specular>
                    </material>
                </visual>
                
                <!-- Collision properties for physics simulation -->
                <collision name='collision'>
                    <geometry>
                        <box>
                            <!-- Same size as visual -->
                            <size>2.0 1.0 0.5</size> 
                        </box>
                    </geometry>
                </collision>
            </link>
        </model>

    </world>
</sdf>
```

In this example, the `<model>` element defines a simple rover robot model named `rover_blue`.
In the `<model>` element, the `canonical_link` attribute specifies the link where the model's reference frame is located.
If no `canonical_link` is specified, the first link in the model is used as the reference frame.
The rover is placed at the origin of the world, with orientation `(0, 0, 0)`.

The rover model consists of a single `<link>` element named `chassis`, which represents the main body of the rover.
The `chassis` link is positioned relative to the model at `(0.5, 0, 0.4)` with orientation `(0, 0, 0)`.
The position and orientation of the `chassis` link are relative to the model, which allows for easy positioning of the link within the model.

The mass and inertia tensor of the `chassis` link are defined in the `<inertial>` element.
Note that only six elements of the inertia tensor are required, as the inertia tensor is symmetric.

The visual representation of the `chassis` link is defined in the `<visual>` element.
The chassis is represented as a box with dimensions `(2.0, 1.0, 0.5)`.
The color of the rover is defined by the `<material>` element, which contains the ambient, diffuse, and specular colors of the rover, all set to blue (RGB: 0, 0, 1).

The collision properties of the `chassis` link are analogous to the visual properties, as the collision geometry is a box with the same dimensions as the visual geometry.

#### Rover Robot (Chassis and Wheels)

We can add to the vehicle model a left, right, and caster wheel.
The following links are added to the model:

```xml
<!-- The left wheel link: -->
<link name='left_wheel'>
    <!-- Left-back position -->
    <!-- The wheel is a cylinder rotated 90 degrees around the Roll axis -->
    <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <!-- Red -->
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
    <!-- Same as visual -->
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

```xml
<!--The right wheel is the same as the left wheel but with different position-->
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose> <!--angles are in radian-->
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

```xml
<!-- For convenience, we define a new frame for the caster wheel -->
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
<!--caster wheel-->
<link name='caster'>
    <pose relative_to='caster_frame'/>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016</iyy>
            <iyz>0</iyz>
            <izz>0.016</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <!-- Note: the caster wheel is a sphere, not a cylinder -->
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
        <material>
            <ambient>0.0 1 0.0 1</ambient>
            <diffuse>0.0 1 0.0 1</diffuse>
            <specular>0.0 1 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
    </collision>
</link>
```

We need to connect these links together; here comes the job of the `<joint>` tag.
The joint tag connects two links together and defines how they will move with respect to each other.
Inside the `<joint>` tag we need to define the two links to connect and their relations (way of movement).

The left wheel joint must connect the `chassis` link to the `left_wheel` link.
The following lines define the left wheel joint:

```xml
<joint name='left_wheel_joint' type='revolute'>
    <pose relative_to='left_wheel'/>
    <parent>chassis</parent>
    <child>left_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz> <!--can be defined as any frame or even arbitrary frames-->
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

The left wheel joint is a revolute joint, which means that the two links can rotate around an axis.
The `<parent>` tag specifies the parent link of the joint (in this case, the `chassis` link), while the `<child>` tag specifies the child link of the joint (in this case, the `left_wheel` link).
The `<axis>` tag specifies the axis of rotation of the joint, which is the y-axis in this case.
The `<limit>` tag specifies the lower and upper limits of the joint angle, which are set to negative and positive infinity, respectively.

The right wheel joint is defined in the same way as the left wheel joint, but with different names and positions:

```xml
<joint name='right_wheel_joint' type='revolute'>
    <pose relative_to='right_wheel'/>
    <parent>chassis</parent>
    <child>right_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

For the caster wheel, we need a different type of joint, called a `ball` joint, which gives the caster wheel three rotational degrees of freedom:

```xml
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
```

After adding all these links and joints to the SDF file, the vehicle model will be loaded in Gazebo with the specified links and joints.

![Gazebo with a vehicle model with joints](figures/gazebo_rover_world.png)

