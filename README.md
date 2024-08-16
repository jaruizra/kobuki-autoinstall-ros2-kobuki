# Kobuki fork from the Intelligent Robotics Lab using ROS 2

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)

In this project, I have added installation scripts for ROS 2 Jazzy and Kobuki.

# Installation on Your Own Computer

## First, Clone the Repository to Your Computer
Run the following commands:
```bash
git clone https://github.com/jaruizra/kobuki-autoinstall-ros2-kobuki.git
cd kobuki-autoinstall-ros2-kobuki
cd scripts
chmod +x ./*
```

## Installing ROS 2 Jazzy
Once you have installed ROS 2 Jazzy, you don't need to do it again unless you remove it from your computer. To install ROS 2 Jazzy, run:
```bash
./install_ros2.sh
```


## Clone, Prepare, and Install the Kobuki Package

This fork of the project was created to simplify the installation and setup process for the Kobuki package. The provided script prepares everything, compiles the Kobuki package, and adds Gazebo to your `.bashrc` so that it runs every time you open a new shell.

The script also sets up the NVIDIA driver if you have an NVIDIA GPU. If you have a different GPU, the script will attempt to make it work; otherwise, it will use CPU rendering. This decision was made to improve the performance of the project.

To run the installer, execute:
```bash
./install_kobuki.sh
```

## Running a real Kobuki
Run the kobuki drivers:

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

### Explanation
There are two types of cameras and two types of lidar sensors that you can find in the laboratory:

**Cameras:**
- **xtion**: This parameter enables the Xtion camera.
- **astra**: This parameter enables the Astra camera.

**Lidar Sensors:**
- **lidar**: This parameter enables the standard lidar sensor.
- **lidar_s2**: This parameter enables the S2 lidar sensor.

Ask the lab technician or professor which camera and lidar you have and use the options you need for your Kobuki.

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
