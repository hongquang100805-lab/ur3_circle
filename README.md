# UR3 Circle Drawing with MoveIt 2

Mô phỏng robot UR3 vẽ quỹ đạo hình tròn bằng ROS 2 Jazzy, Gazebo Sim và MoveIt 2.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim
- MoveIt 2

## Run

### Terminal 1

```bash
ccd ~/ur_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_llm_control llm_robot.launch.py

```

### Terminal 2

```bash
cd ~/ur_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run ur3_llm_control skill_executor
