# Kobuki fork from the Intelligent Robotics Lab using ROS 2

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)

This repository provides a streamlined setup for working with the Kobuki robot using ROS 2. In this guide, installation scripts for ROS 2 Jazzy and the Kobuki package have been added to simplify the process.

# Installation on Your Computer

## Important Considerations

This guide is tailored for Ubuntu 24.04 LTS. It’s essential to note the following for optimal performance:

- **Hardware Requirements:** A system with an NVIDIA GPU and a powerful CPU is strongly recommended. The Kobuki robot's performance can be heavily influenced by your hardware. On systems without a robust GPU, real-time applications like AI-driven object detection might suffer from significant lag or may not function correctly. A powerful GPU ensures that the robot can execute complex tasks, such as running the YOLO object detection algorithm, alongside other processes.

- **WSL2/Advanced Users:** If you prefer to work within a Windows environment, you can use WSL2 (Windows Subsystem for Linux). However, setting up ROS 2 and Kobuki on WSL2 involves additional steps and can be more complex. For detailed instructions on using WSL2, refer to this complementary guide specifically designed for EIF courses:
    ```
    https://github.com/jaruizra/EIF-WSL2-Course-Scripts/tree/netgui-wireshark-setup
    ```

## First, Clone the Repository to Your Computer
To get started, clone the repository and set up the scripts:
```bash
git clone https://github.com/jaruizra/kobuki-autoinstall-ros2-kobuki.git
cd kobuki-autoinstall-ros2-kobuki
cd scripts
chmod +x ./*
```

## Installing ROS 2 Jazzy
ROS 2 Jazzy only needs to be installed once. To install ROS 2 Jazzy on your system, run the following script:
```bash
./install_ros2.sh
```

## Clone, Prepare, and Install the Kobuki Package

This fork simplifies the setup and installation of the Kobuki package for use with ROS 2. The provided script automates the following:

- **Preparation and Compilation:** The script clones the necessary repositories, compiles the Kobuki package, and integrates it with your ROS 2 environment.

- **Environment Setup:** It modifies your .bashrc file to include Gazebo, ensuring it runs every time you open a new shell.

- **NVIDIA GPU Configuration:**`(READ NVIDIA EXPLANATIONS BELOW)` If you have an NVIDIA GPU, the script will automatically set up the necessary drivers for optimal performance. If you have a different GPU, the script will try to adapt, although performance may vary. CPU rendering is the fallback option.


To run the installer, execute:
```bash
./install_kobuki.sh
```

## Running a real Kobuki
To start using the Kobuki robot, launch the drivers with:
```bash
ros2 launch kobuki kobuki.launch.py
``` 

If you want to use a lidar or camera, you have to set the following parameters to `True`:
```bash
ros2 launch kobuki kobuki.launch.py lidar:=True
ros2 launch kobuki kobuki.launch.py lidar_s2:=True
ros2 launch kobuki kobuki.launch.py xtion:=True
ros2 launch kobuki kobuki.launch.py astra:=True
``` 

# Explanations
## Cameras and Lidar Sensors
In the lab, you might encounter different types of cameras and lidar sensors. Below are the available options:

Cameras:
- xtion: This parameter enables the Xtion camera.
- astra: This parameter enables the Astra camera.

Lidar Sensors:
- lida: This parameter enables the standard lidar sensor.
- lidar_s2: This parameter enables the S2 lidar sensor.

Ask the lab technician or professor which camera and lidar you have and use the options you need for your Kobuki.

## NVIDIA GPU Setup
### What Does This Script Do?
At the start, this script checks if your system has an NVIDIA GPU and if the appropriate NVIDIA driver is installed correctly. If the script doesn’t detect the NVIDIA driver, it will notify you and attempt to install it later in the process.

### Why NVIDIA?
This script is designed specifically for NVIDIA GPUs because the YOLO package, which you’ll use later in the course, is optimized for NVIDIA hardware. While it might be possible to adapt it for AMD GPUs, doing so would require significant effort.

- **CUDA Support:** YOLO and other deep learning models are optimized to run on CUDA, NVIDIA’s parallel computing platform.

- **Performance:** NVIDIA GPUs provide the necessary computational power to handle the high frame rates required for real-time object detection.

Running YOLO without an NVIDIA GPU may lead to significant delays, reduced accuracy, and suboptimal performance, which can impair the Kobuki robot’s ability to function effectively.

### Running on Laptops with Dual-Boot Systems
For laptop users with dual-boot setups (Windows and Ubuntu), there is a known issue where the GPU might be disabled in Ubuntu. This happens because certain settings or power-saving features deactivate the GPU when not in use.

To check if the GPU is enabled, run the script. If the script fails to detect the GPU, you’ll need to enable it in Windows using the appropriate utility:

- Asus: Armory Crate
- Acer: PredatorSense (or the relevant BIOS settings)
- MSI: Dragon Center or MSI Center
- Dell/Alienware: Alienware Command Center
- HP Omen: OMEN Gaming Hub

Once enabled, shut down your laptop, boot into Ubuntu, and re-run the script. It should now detect the NVIDIA GPU. If it doesn’t, you may need to update your BIOS or adjust additional settings.

## Compilation Issues and System Freezes

### Problem Description
Compilation issues, such as script failures or system freezes, are common on systems with insufficient RAM and no SWAP partition. The Kobuki and Gazebo packages require significant memory to compile—around `16 GB of combined RAM and SWAP memory`. If your system doesn’t meet this requirement, you may encounter problems during the installation process.

The step that fails is the `colcon-build` that compiles the kobuki package.

### Script Behavior
The script is designed to detect your system’s available memory. If it finds that the total RAM and SWAP memory is insufficient, it will alert you before proceeding. This can help prevent issues related to memory shortages during compilation

### Troubleshooting Steps
If you still experience problems despite the memory check, follow these steps:

1. **Re-run the Script:** Sometimes, temporary issues can cause a failure. Running the script again might resolve the problem.

2. **Manual Installation:** If the script continues to fail, consider manually installing the Kobuki package. You can find detailed instructions in the official repository from Intelligent Robotics Labs:
    ```
    https://github.com/IntelligentRoboticsLabs/kobuki
    ```
3. **Seek Assistance:** If manual installation also fails, consult your lab technician or professor. They can provide guidance or help troubleshoot issues that arise during the colcon build process.

### Recommendation
To avoid these issues, ensure that your system has at least 16 GB of combined RAM and SWAP memory. If your system lacks sufficient memory, consider adding a `15~30GB SWAP` partition or file to increase the available memory for compiling the resource-intensive kobuki and gazebo package.

# About

This is a project made by the [Intelligent Robotics Lab], a research group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2024.

Maintainers:

* [Juan Carlos Manzanares]

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[Intelligent Robotics Lab]: https://intelligentroboticslab.gsyc.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
[Juan Carlos Manzanares]: https://github.com/Juancams
[Francisco Martín]: https://github.com/fmrico
[Nav2]: https://navigation.ros.org/
[Keepout Zones]: https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html?highlight=keep
[SLAM Toolbox]: https://vimeo.com/378682207
[Navigate While Mapping]: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
