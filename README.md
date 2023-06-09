# SymbioticRobots

Install [Docker](https://docs.docker.com/desktop/install/ubuntu/)

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

**LAUNCH GAZEBO**

```bash
roslaunch jackal_gazebo jackal_world.launch
```

You can also launch it with specific configs:

```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```
