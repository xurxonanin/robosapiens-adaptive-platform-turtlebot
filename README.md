# RoboSapiens Adaptive Platform (Turtlebot Simulator)

This project supports running a complete self-adaptive system for handling LiDAR occulusion anomolies for a Turtlebot4. This includes a self-adaptive MAPLE-K loop implemented using the RoboSapiens Adaptive Platform, Gazebo simulation of the TurtleBot 4, and .

The repository provides a devcontainer providing all of the services required to run the system.
Opening one of the devcontainers launches the entire project along with all its dependencies (ROS, MQTT, etc.).

## Getting Started

### Launching the Full Project

1. **Open the Full DevContainer:**  
   Use one of the full devcontainer configurations (e.g., [`.devcontainer/nvidia-full-tb3/devcontainer.json`](.devcontainer/nvidia-full-tb3/devcontainer.json) or [`.devcontainer/mesa-full-tb3/devcontainer.json`](.devcontainer/mesa-full-tb3/devcontainer.json)) in Visual Studio Code.  
   Use the **"Rebuild and Reopen in Container"** command to make sure the devcontainer is launched.

2. **Services in the Container:**  
   The devcontainer setup automatically starts all the necessary services for:
   - Simulation (Gazebo)
   - ROS integration
   - MQTT communication

### Running the Self-Adaptive Loop

To launch the self-adaptive loop:

```bash
cd maple-loops/HelloWorld
python3 main.py
```

This will log its progress in `maple-loops/HelloWorld/MAPE_test.log`.

You can also launch a live dashboard showing the current loop stage and LiDAR mask using:

```bash
cd maple-loops/HelloWorld
python3 LiveDashboard.py
```

### Launching the Trustworthiness Checker
The Trustworthiness Checker is available as a separate Docker Compose service.
You can run it independently in a host terminal by running e.g.

```bash
cd docker/
docker compose run --rm trustworthiness-checker /mnt/host_models/maple_seq.lola --input-mqtt-topics stage --output-stdout
```

This example runs the trustworthiness checker on the model `trustworthiness-specs/maple_seq.lola` from the input MQTT topic `stage` (provided by the self-adaptive loop) and outputs verdicts to STDOUT.
You can also change this to output to MQTT topics via the command:

```bash
cd docker/
docker compose run --rm trustworthiness-checker /mnt/host_models/maple_seq.lola --input-mqtt-topics stage --output-stdout
```

## Devcontainer Variants
This repository includes several devcontainer variants to support different hardware and simulation configurations:

### RoboSapiens Adaptive Platform -- Full TB4 Gazebo (NVIDIA GPU):

Provides a full development environment for running the TurtleBot 4 simulation using Gazebo on systems with NVIDIA GPUs. It automatically builds and starts all necessary services and installs the required dependencies after container start-up. You must have [Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed to use this option.

### RoboSapiens Adaptive Platform -- Full TB3 Gazebo (NVIDIA GPU):

Tailored for running TB3 (TurtleBot 3) simulations with NVIDIA GPU support. Features similar configurations as the TB4 variant but adapted for TB3 simulation parameters.

### RoboSapiens Adaptive Platform -- Full Gazebo TB4 (MESA):

Designed for systems without NVIDIA GPUs. This variant uses the MESA graphics stack to enable full Gazebo simulation for TurtleBot 4. It still provides all the integrated services to run the adaptive platform.

### RoboSapiens Adaptive Platform -- Full TB3 Gazebo (MESA):

Tailored for running TB3 (TurtleBot 3) simulations with MESA graphics. Features similar configurations as the TB4 variant but adapted for TB3 simulation parameters.

## RoboSapiens Adaptive Platform -- Full Gazebo TB4 (No GPU):

Provides a complete Gazebo simulation environment for TurtleBot 4 for machines without GPU acceleration. It ensures that users without any graphics hardware can still run the full self-adaptive system, but may be painfully slow.

### Non-full variants

These variants just run the main devcontainer without any dependencicies. They are useful when separately developing new versions of these dependencies, or for running Gazebo locally.
