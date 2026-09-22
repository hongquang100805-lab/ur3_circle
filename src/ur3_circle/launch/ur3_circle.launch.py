from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ==============================
    # 1. Launch UR3 chính thức
    # ==============================
    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"),
                "launch",
                "ur_sim_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": "ur3",
        }.items(),
    )

    # ==============================
    # 2. Node vẽ
    # ==============================
    drawing_node = Node(
        package="ur3_circle",
        executable="drawing_node.py",
        name="draw_circle",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,

                "frame_id": "base_link",
                "ee_link": "tool0",
                "group_name": "ur_manipulator",

                "x": 0.30,
                "y": -0.08,
                "z": 0.40,

                "radius": 0.08,
                "step": 0.004,

                "speed_scale": 0.25,
                "execute": True,
            }
        ],
    )

    # ==============================
    # 3. RViz riêng của bài vẽ
    # ==============================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_draw_p",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("ur3_circle"),
                "config",
                "draw_p.rviz",
            ])
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        ur_sim_moveit,
        rviz_node,
        drawing_node,
    ])