# Urban Delivery Robot

## wheeled bipedal robot to wheeled quadruped robot

### Startup

1. install ROS2 humble
2. clone the repositories

```bash
git clone https://github.com/anson-liumaote/crazydog.git
```

3. download Unitree actuator sdk

```bash
cd crazydog_ws/src/robot_interfaces/robot_interfaces/
git clone https://github.com/unitreerobotics/unitree_actuator_sdk.git
```

4. follow Unitree actuator sdk README to build the sdk
5. build ros2 packages

```bash
cd crazydog_ws
colcon build --packages-select robot_interfaces unitree_msgs --symlink-install
```

6. setup CAN interfaces

```bash
cd crazydog_ws/src/robot_interfaces/
sudo chmod +x setup_can.sh
./setup_can.sh
```

7. source

```bash
source install/setup.bash
```

8. run the python files
