# Organizing Large Gazebo SDF Files

This document provides a recommended structure for organizing Simulation Description Format (SDF) files when working with Gazebo. This approach simplifies the management of large simulation environments by separating models and world definitions into modular components.

## Recommended Directory Structure

Organize your simulation files as follows:

```plaintext
my_simulation/
├── models/
│ ├── robot1/
│ │ ├── model.sdf
│ │ └── model.config
│ ├── obstacle/
│ │ ├── model.sdf
│ │ └── model.config
├── worlds/
│ └── mars_world.sdf
```

- Each **model** (e.g., robot, obstacle) lives in its own subdirectory under `models/`.
- Each model directory contains:
  - `model.sdf`: The model's SDF file.
  - `model.config`: The configuration file describing metadata.
- The **world file** (e.g., `mars_world.sdf`) references models using `<include>` tags.

## Referencing Models in World SDF

Inside your world SDF file, you can include models as shown below:

```xml
<include>
  <uri>model://robot1</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

URIs are resolved by replacing `model://` with the paths in the `GZ_SIM_RESOURCE_PATH` environment variable.

## Setting GZ_SIM_RESOURCE_PATH

To make Gazebo locate your models, set the GZ_SIM_RESOURCE_PATH environment variable:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/my_simulation/models
```

You can add this line to your .bashrc or .zshrc file for persistence across sessions.
