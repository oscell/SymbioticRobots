# SymbioticRobots

**LAUNCH GAZEBO**

### Steps

```bash
cd Jackal_ws
```

```bash
catkin_make
```

```bash
source Jackal_ws/devel/setup.bash
```

Launch rviz

```bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

```bash
roslaunch jackal_navigation gmapping_demo.launch
```

```bash
roslaunch jackal_viz view_robot.launch config:=gmapping
```

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
```
