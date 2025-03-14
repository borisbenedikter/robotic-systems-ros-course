# Running ROS 2 Iron in WSL2 via Docker Image

The [installation guide](../../docs/installation_guide.md#installation-guide) provided in this repository is designed to help you set up ROS 2 Iron on Ubuntu 22.04.
However, setting up ROS 2 Iron in your WSL can sometimes be challenging, especially for those unfamiliar with Linux environments. 
To simplify this process, there is a Docker image with Ubuntu 22.04 and ROS 2 Iron pre-installed. 
By following these instructions, you can import this image into WSL2, allowing you to run ROS seamlessly without needing to manually install and configure it.

This approach is particularly useful if you encountered issues during installation or if you prefer a ready-to-use environment. 
If you are already comfortable with setting up ROS manually, you may not need to follow this method. 
However, for those looking for a quick and reliable setup, importing the Docker image into WSL2 provides a convenient alternative.

## Importing a Docker Image into WSL2

To import a Docker image into WSL2, follow these steps:

**Download the Docker image**:
  
The Docker image with Ubuntu 22.04 and ROS 2 Iron pre-installed is available for download at the following link: [Docker Image](https://data.cyverse.org/dav-anon/iplant/home/borisbenedikter/robotic-systems-ros-course/ubuntu22rosiron.tar).
Download it to your local Windows machine and save it in the directory `C:\temp\`.
If you are using a different directory, make sure to adjust the paths accordingly in the following steps.

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

At the first run, the working directory may be set to `/mnt/c/temp/`, which is the location where you launched the PowerShell command. 
You can change the working directory to your home directory (`/home/rosuser`) by running:

```bash
cd
```

**Verify the Installation**:

You can also verify if ROS is installed correctly by sourcing the ROS environment and launching the `talker` node:

```bash
source /opt/ros/iron/setup.bash 
ros2 run demo_nodes_cpp talker
```

**User and Password**:

For `sudo` commands, the rosuser's password is `rosuser`.
