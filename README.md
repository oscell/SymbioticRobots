
# SymbioticRobots: Mixed Reality & Digital Twin for Remote Operations
## Abstract

This project investigates the use of Mixed Reality (MR) and Digital Twin (DT) technologies to enable remote operations, inspections, and training for maintenance tasks in access-challenged or hazardous environments such as offshore wind farms or nuclear facilities. By integrating a ground robot equipped with sensors, a HoloLens device, and digital twin models, we've conceived a proof-of-concept system that facilitates the visualization and control of the robotic platform. The methodology describes a framework for deploying mixed reality applications at different levels of the Reality-Virtuality (RV) continuum. This is implemented through the fusion of Clearpath’s Jackal robot, HoloLens 2, ROS middleware, and the Unity platform. 

For a detailed overview, refer to the [dissertation document](https://github.com/oscell/MSc_HoloLens_Jackal_2391076).

## Overview

This project outlines the steps to building an application for the [HoloLens 2](https://learn.microsoft.com/en-us/hololens/hololens-commercial-features) that is used to interact with the DT in the ROS workspace. This is constructed using the [realsense camera](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) and the [clearpath jackal](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/navigation.html).
## Repository Contents

- **Jackal_cam_lasers**: The ROS workspace. [Link to repository](https://github.com/oscell/Jackal_cam_lasers)
- **Unity Package**: Contains the Unity package with all scenes and the development environment.
- **Built App**: The application built for deployment on HoloLens 2.
- **Poster**: A visual representation of the project.
- **Dissertation**: The main document detailing the research, methodology, and findings.

## Setup & Installation

### 1. Prepare the Workspace

```bash
git clone https://github.com/oscell/SymbioticRobots.git
ubuntu1804
cd SymbioticRobots/jackal_ws
catkin_init_workspace src/
catkin_make
```

### 2. Launch Simulation

```bash
source devel/setup.bash
roslaunch aws_robomaker_small_warehouse_world view_small_warehouse.launch
```

... [other commands]

### 3. Linux Setup Installation

#### Requirements

- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) or its [wsl](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-10#1-overview)
- [ROS melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) 

... [other installation steps]

### 4. Unity Installation

... [Unity installation steps]

## Additional Repositories

- **Jackal_cam_lasers**: Contains the ROS workspace for the project. [Link to repository](https://github.com/oscell/Jackal_cam_lasers)
- **Dissertation TEX Document**: The main document detailing the research, methodology, and findings. [Link to repository](https://github.com/oscell/MSc_HoloLens_Jackal_2391076) (private)

## Acknowledgements

This project was developed as part of a dissertation. Special thanks to all contributors and advisors.

---

This enhanced README provides a clearer structure and a brief overview of the project right at the beginning. It's always a good idea to keep the README concise and to the point, directing users to other documents or links for detailed information.


# SymbioticRobots


## Overview

This project uses the [HoloLens 2](https://learn.microsoft.com/en-us/hololens/hololens-commercial-features), the [realsense camera](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) and built-in [clearpath jackal](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/navigation.html) laser to build an app for teleoperation. Start with the [Installation](https://github.com/oscell/SymbioticRobots#installation), then go to Launch simulation.

## Prepare the workspace

Clone: 

```bash
git clone https://github.com/oscell/SymbioticRobots.git
```

Open the subsystem:

```bash
ubuntu1804
```

Go to workspace

```bash
cd SymbioticRobots/jackal_ws
```

Initiate the workspace:

```bash
catkin_init_workspace src/
catkin_make
```
## Launch simulation

```bash
source devel/setup.bash
roslaunch aws_robomaker_small_warehouse_world view_small_warehouse.launch
```

```bash
source devel/setup.bash
roslaunch jackal_gazebo spawn_jackal.launch config:=cam_laser
```

```bash
rviz rviz
=======
roslaunch jackal_navigation amcl_demo.launch
```

```bash
roslaunch jackal_navigation odom_navigation_demo.launch
```

```bash
ifconfig
```

#### VM using hotspot
```bash
source ~/Desktop/SymbioticRobots/Jackal_ws/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=192.168.86.39 tcp_port:=10000
```


```bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

# Installation

## Linux setup Installation

### Requirements

- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) or its [wsl](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-10#1-overview)
- [ROS melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) 

> Note you might want to source your workspace from `.bashrc`

```bash

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install jackal packages
source ~/Desktop/SymbioticRobots/Jackal_ws/devel/setup.bash
rosrun utils base_link_listener.py
```


```bash
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation -y
```

### Install Realsense packages

```bash
sudo apt-get install ros-melodic-realsense2-camera
```

## Unity Installation

### Unity Hub

To [Install Unity](https://unity.com/download) we will first install [Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux):

1. To add the public signing key, run the following command:
    
    ```
    wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
    
    ```
    
2. To add the Unity Hub repository, you need an entry in `/etc/apt/sources.list.d`. Run the following command to add the Unity Hub repository:
    
    ```
    sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
    
    ```
    
3. Update the package cache and install the package:
    
    ```
    sudo apt update
    sudo apt-get install unityhub
    ```
    
    **Uninstall with: `sudo apt-get remove unityhub` .**
    

### Unity editor

We will install Unity 2020.3.2f1, although it is not explicitly mentioned, it [can work on Ubuntu 16.04](https://docs.unity3d.com/2020.1/Documentation/Manual/system-requirements.html).

Once logged into Unity Hub open `Install editor > 2020.3.48f1 LTS.`


### Installing the Unity Robotics packages

This page provides brief instructions on installing the Unity Robotics packages. Head over to the [Pick-and-Place Tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md) for more detailed instructions and steps for building a sample project.

1. Create or open a Unity project.
    
    > Note: If you are adding the URDF-Importer, ensure you are using a 2020.2.0+ version of Unity Editor.
    > 
2. Open `Window` -> `Package Manager`.
3. In the Package Manager window, find and click the `+` button in the upper lefthand corner of the window. Select `Add package from git URL...`.
    
    ![package manager](https://github.com/Unity-Technologies/Unity-Robotics-Hub/raw/main/images/packman.png)
    
4. Enter the git URL for the desired package. Note: you can append a version tag to the end of the git url, like `#v0.4.0` or `#v0.5.0`, to declare a specific package version, or exclude the tag to get the latest from the package's `main` branch.
    1. For the [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector), enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`.
    2. For the [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer), enter `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`.
5. Click `Add`.

To install from a local clone of the repository, see [installing a local package](https://docs.unity3d.com/Manual/upm-ui-local.html) in the Unity manual.

## Install Jackal
### Kinetic

```bash
sudo apt-get install ros-kinetic-jackal-simulator ros-kinetic-jackal-desktop ros-kinetic-jackal-navigation
```
### Melodic
```bash
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation
```



<!-- # SymbioticRobots

### SimulationSteps

```bash
cd SymbioticRobots/Jackal_ws
```

```bash
catkin_make
```

```bash
source devel/setup.bash
```

Launch rviz

```bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

#### Mapping
```bash
roslaunch jackal_navigation gmapping_demo.launch
```


```bash
roslaunch jackal_viz view_robot.launch config:=gmapping
```
#### With a map
```bash
roslaunch jackal_navigation amcl_demo.launch map_file:=/home/oscar/Desktop/SymbioticRobots/assets/full_map.yml
```

```bash
roslaunch jackal_viz view_robot.launch config:=localization
```
#### Ros TCP connection

```bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

Open Robotics/ROS Settings from the Unity menu bar, and set the ROS IP Address variable to the IP you set


```bash
cd ~/Desktop/Github/SymbioticRobots/assets/
rosrun testing map_maker.py
```

************************Save the map************************

```bash
cd ~/Desktop/Github/SymbioticRobots/assets
rosrun map_server map_saver -f mymap1
```

****************************Convert to png****************************

```jsx
cd ../usefulscripits
python pgm_to_png.py
```



## Linux setup Installation
```
sudo apt install git-all -y
```

```bash
git clone https://github.com/oscell/SymbioticRobots.git
cd SymbioticRobots
```

```bash
chmod +x requirements.txt
./requirements.txt
``` -->
