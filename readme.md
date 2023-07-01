# GoCar Project
The scope of this project is to develop from zero an environment simulation where to integrate and develop algorithms for autonomous driving.

## Dependencies
* Standard installation of ROS Noetic (Ubuntu 20.04 or similar flavours).
```
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
* Setup script execution: Binaries sourcing + Third-party ROS-package installation + Virtual keyboard event file read permission.
```
cd GoCar
./src/go_car_bringup/scripts/setup_env.sh
```
* Git submodules cloning:
```
git submodule update --init --recursive
```

## Execution
```
roslaunch go_car_bringup start_simulation.launch control_type:=keyboard world_name:=city_v2 details:=quality rviz_visualization:=true
```
Params:
+ control_type: keyboard/gamepad
+ world_name: empty/city_v1/city_v2
+ details: quality/performance
+ rviz_visualization: true/false


*Author:* Sergio León Doncel
