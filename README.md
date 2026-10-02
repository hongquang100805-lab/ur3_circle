# UR3 semantic control with an LLM

ROS 2 Jazzy project for student **Lê Hồng Quang — 23020757**. With
`XX=57`, `P=57 mod 6=3`, the personalized mapping is:

- Zone A: Yellow
- Zone B: Blue
- Zone C: Red

The runtime pipeline is:

`Vietnamese/English command -> OpenAI-compatible LLM -> JSON plan -> strict validator -> skills -> MoveIt 2 -> UR3 in Gazebo Sim`

There is no mock LLM fallback. API, JSON, validation, planning, controller, or
simulator-grasp failures stop the task, and `TASK SUCCESS` is printed only when
every requested step succeeds.

## LLM configuration

Create or update `.env` (this file is ignored by Git):

```bash
OPENAI_BASE_URL=http://localhost:20128/v1
OPENAI_API_KEY=replace-with-your-key
LLM_MODEL=replace-with-a-model-supported-by-your-router
LLM_TIMEOUT_SECONDS=45
```

`OPENAI_BASE_URL` may be either the API root ending in `/v1` or the complete
`/chat/completions` endpoint. A 401 is treated as a fatal configuration error;
the key is never printed and the robot does not run a substitute plan.

## Build and run

```bash
cd ~/ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

Terminal 1 (simulation, MoveIt and scene):

```bash
cd ~/ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

Wait until Gazebo contains `table`, all three cubes, and the UR robot. Terminal
2 (load secrets into the process and start the interactive executor):

```bash
cd ~/ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -a
source .env
set +a
ros2 run ur3_llm_control skill_executor
```

The executor refuses motion unless it can verify this Gazebo scene, which
prevents accidentally using the CLI against a MoveIt instance connected only
to a physical robot.

## Demo commands

Basic:

```text
Đưa khối màu đỏ vào vùng B.
Move the blue cube to zone C.
```

Personalized multi-object task:

```text
Sắp xếp tất cả vật theo mã sinh viên của tôi.
Arrange all objects according to my student ID.
```

The multi-object result must be Yellow -> A, Blue -> B, Red -> C.

## Grasp behavior

The simple parallel gripper closes physically in Gazebo, but the reliable hold
mechanism in this project is explicit simulator pose-follow through
`/world/empty/set_pose`. MoveIt's attached collision object is maintained
separately for collision planning. A pick succeeds only after both the Gazebo
pose-follow operation and the MoveIt attachment succeed. Place stops
pose-follow, moves the model to the table, removes the MoveIt attachment, and
re-adds the world collision object before retreating.

The validator rejects occupied destination zones. It supports a plan that first
moves the occupant away; it never stacks objects or invents a temporary zone.

## Tests

```bash
cd ~/ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/ur3_llm_control/test
```
# ur3_llm
