# Python Tutorial

## Python Installation

### Ubuntu

Python is installed by default in Ubuntu. To check the version of Python installed, run the following command in the terminal:

```bash
python3 --version
```

If Python is not installed, run the following command to install it:

```bash
sudo apt update
sudo apt install python3
```

## Run a Python Script

To run a Python script, run the following command in the terminal:

```bash
python3 hello_world.py
```

You will see the output `Hello, World!` printed to the terminal.

## Conda

Python's default package manager is `pip`. However, `conda` is another package manager that is widely used in the Python community, especially in the data science and scientific computing fields.

Conda is an open-source package management system and environment management system that runs on Windows, macOS, and Linux. 
Conda quickly installs, runs, and updates packages and their dependencies. 
Conda easily creates, saves, loads, and switches between environments on your local computer. It was created for Python programs, but it can package and distribute software for other languages as well.

### Installation

Conda is not installed by default on Ubuntu 22.04 or any other version of Ubuntu. Conda is part of the Anaconda or Miniconda distributions, which need to be installed separately.

If you're just starting out with data science or scientific computing and prefer a hassle-free setup, Anaconda is a good choice. It provides a complete environment with everything you need to get started immediately.

If you're an experienced user who prefers a lightweight environment and wants more control over package installations, Miniconda is the way to go. You can install only the packages you need, keeping your environment clean and efficient.

#### Miniconda

To install Miniconda, run the following commands in the terminal:

```bash
mkdir ~/miniconda3
cd ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
mv Miniconda3-latest-Linux-x86_64.sh miniconda.sh
bash miniconda.sh -b -u -p ~/miniconda3
```
In the last command, `-b` is for batch mode, which accepts the license agreement and does not ask for confirmation, `-u` is for updating an existing installation if present, and `-p` is for specifying the installation path.

<!-- Accept the license agreement, select the installation location, and follow the prompts to complete the installation. -->

After installation, you can delete the installation file:

```bash
rm -rf ~/miniconda3/miniconda.sh
```

You might also need to initialize Conda:

```bash
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

Close and reopen your terminal.

#### Anaconda (Never tried!)

To install Anaconda, run the following commands in the terminal:

```bash
wget https://repo.anaconda.com/archive/Anaconda3-latest-Linux-x86_64.sh
bash Anaconda3-latest-Linux-x86_64.sh
```

Accept the license agreement, select the installation location, and follow the prompts to complete the installation.

Initialize Conda:

```bash
source ~/anaconda3/bin/activate
conda init
```

Close and reopen your terminal.

#### Verify Installation

To verify that Conda is installed, run the following command in the terminal:

```bash
conda --version
```

You should see the version of Conda installed.

### Basic Conda Commands

Here are some basic Conda commands:

- Create a new (minimal) environment with just Python (not all the packages that come with Anaconda base environment):

```bash
conda create --name myenv
```

- Activate the environment:

```bash
conda activate myenv
```

- Install a package in the environment:

```bash
conda install numpy
```

- Deactivate the environment:

```bash
conda deactivate
```

- Export the environment to a file (for reproducibility):

```bash
conda env export > environment.yml
```

- Recreate the environment from the file:

```bash
conda env create -f environment.yml
```

- List all environments:

```bash
conda env list
```

- Remove an environment:

```bash
conda env remove --name myenv
```

- Update Conda:

```bash
conda update -n base -c defaults conda
```
