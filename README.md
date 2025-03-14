# Robotic Systems Course: ROS Module

Welcome to the Robotic Systems Course GitHub repository! This repository is designed to accompany the ROS module of the course SIE 496/596: Robotic Systems at the University of Arizona. 
The repo contains a collection of resources, examples, and documentation to support your learning experience with ROS.

## Note

⚠️ This repository is in development and will be updated regularly. 
Some of the content may be incomplete or subject to change.

## Repository Structure

- **`docs/`**: Contains markdown documentation to guide you through various topics in ROS.
  - `installation_guide.md`: Step-by-step instructions to set up ROS.
  <!-- - `ros_basics.md`: Basic concepts and usage of ROS.
  - `robot_simulation.md`: Guide on robot simulation using ROS.
  - `advanced_topics.md`: Advanced topics in ROS for further exploration. -->

- **`examples/`**: A collection of example projects demonstrated during the course.
  - `pub_sub_ws`: Example workspace containing a package with a publisher and subscriber node (like `talker` and `listener`).
  <!-- - `basic_ros_example/`: Basic example to get started with ROS.
  - `robot_arm_control/`: Example for controlling a robotic arm.
  - `rover_navigation/`: Example for navigating a rover.
  - `quadcopter_simulation/`: Example for simulating a quadcopter. -->

- **`resources/`**: Additional resources such as lecture slides and recommended readings.
  - `Dockerfile`: Dockerfile for setting up a Ubuntu 22.04 environment with ROS Iron installed. The image can be used to run the course examples and projects. Also you can use it to import a WSL2 distro with ROS Iron installed.
  <!-- - `slides/`: Lecture slides used in the course.
  - `papers/`: Research papers and articles related to robotics.
  - `additional_reading.md`: Additional reading materials and references. -->

## Getting Started

**Follow the instructions in the `docs/installation_guide.md` to set up ROS on your system**.

## Importing a Docker Image into WSL2
To import a Docker image into WSL2, follow these steps:

**Download the Docker image**:
  
The Docker image is available in the `resources/` directory as `ubuntu22rosiron.tar`. Download it to your local Windows machine and save it in the directory `C:\temp\`.

**Open PowerShell**:

  Open PowerShell on your Windows machine and navigate to the directory where you saved the Docker image. For example:

```powershell
  cd C:\temp\
```

**Create a New Directory for the WSL2 Distro**:

Create a new directory for the WSL2 distro. For example,

```powershell
mkdir C:\wslDistroStorage\ubuntu22rosiron
```

**Import the Docker Image**:

Use the `wsl --import` command to import the Docker image into WSL2. Replace `<DistroName>` with the name you want to give to your WSL2 distro (e.g., `ubuntu22rosiron`):

```powershell
wsl --import ubuntu22rosiron C:\wslDistroStorage\ubuntu22rosiron C:\temp\ubuntu22rosiron.tar 
```

**Run the WSL2 Distro**:

After importing the image, you can run the WSL2 distro using the following command in PowerShell:

```powershell
wsl -d ubuntu22rosiron
```

You can also verify if ROS is installed correctly by sourcing the ROS environment and launching the `talker` node:

```bash
source /opt/ros/iron/setup.bash 
ros2 run demo_nodes_cpp talker
```

The rosuser's password is `rosuser`.
   
## Contributing

Contributions are welcome! If you find any issues or you'd like to improve the examples or documentation, please send an email to `boris@arizona.edu`.

## License

This repository is licensed under the MIT License. See `LICENSE` for more information.

---

Happy learning and coding! 🤖🚀