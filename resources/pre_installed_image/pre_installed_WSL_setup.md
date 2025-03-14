# Importing a Docker Image into WSL2

To import a Docker image into WSL2, follow these steps:

**Download the Docker image**:
  
The Docker image is available at this link: [Docker Image](https://data.cyverse.org/dav-anon/iplant/home/borisbenedikter/robotic-systems-ros-course/ubuntu22rosiron.tar). 
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

You can also verify if ROS is installed correctly by sourcing the ROS environment and launching the `talker` node:

```bash
source /opt/ros/iron/setup.bash 
ros2 run demo_nodes_cpp talker
```

For `sudo` commands, the rosuser's password is `rosuser`.
