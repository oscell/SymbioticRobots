# SymbioticRobots

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