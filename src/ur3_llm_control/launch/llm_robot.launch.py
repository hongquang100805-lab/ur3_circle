import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ur_simulation = get_package_share_directory('ur_simulation_gz')
    pkg_ur3_llm = get_package_share_directory('ur3_llm_control')
    
    # Load scene.yaml
    with open(os.path.join(pkg_ur3_llm, 'config', 'scene.yaml'), 'r') as f:
        scene = yaml.safe_load(f)
        
    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_simulation, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
            'launch_rviz': 'true',
          'description_file': os.path.join(
            pkg_ur3_llm, 'urdf', 'ur3_table_mount.urdf.xacro'
          ),
            'controllers_file': os.path.join(
              pkg_ur3_llm, 'config', 'ur3_gripper_controllers.yaml'
            ),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )
    
    nodes = [
      ur_sim_moveit,
      Node(
        package='ur3_llm_control',
        executable='scene_publisher',
        output='screen',
      ),
      Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '-c', '/controller_manager'],
        output='screen',
      ),
    ]
    
    # Function to create an SDF string for a colored box
    def get_box_sdf(name, color_rgba, size="0.04 0.04 0.04"):
        return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx> <iyy>0.0001</iyy> <izz>0.0001</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{color_rgba}</ambient>
          <diffuse>{color_rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def get_table_sdf():
        table_center = scene['table']['center']
        table_size = scene['table']['size']
        leg_height = table_center[2] - table_size[2] / 2
        leg_center_z = leg_height / 2
        leg_x_offset = table_size[0] / 2 - 0.02
        leg_y_offset = table_size[1] / 2 - 0.045
        leg1_x = table_center[0] - leg_x_offset
        leg2_x = table_center[0] + leg_x_offset
        leg1_y = table_center[1] - leg_y_offset
        leg2_y = table_center[1] + leg_y_offset
        return """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="table">
    <static>true</static>
    <link name="link">
      <!-- Table geometry is expressed in the Gazebo world frame. -->
      <visual name="top_visual">
        <pose>{table_center[0]} {table_center[1]} {table_center[2]} 0 0 0</pose>
        <geometry><box><size>{table_size[0]} {table_size[1]} {table_size[2]}</size></box></geometry>
        <material>
          <ambient>0.35 0.35 0.38 1</ambient>
          <diffuse>0.60 0.60 0.65 1</diffuse>
        </material>
      </visual>
      <collision name="top_collision">
        <pose>{table_center[0]} {table_center[1]} {table_center[2]} 0 0 0</pose>
        <geometry><box><size>{table_size[0]} {table_size[1]} {table_size[2]}</size></box></geometry>
      </collision>
      <!-- 4 Legs -->
      <visual name="leg1">
        <pose>{leg1_x} {leg1_y} {leg_center_z} 0 0 0</pose>
        <geometry><box><size>0.04 0.04 {leg_height}</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="leg2">
        <pose>{leg1_x} {leg2_y} {leg_center_z} 0 0 0</pose>
        <geometry><box><size>0.04 0.04 {leg_height}</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="leg3">
        <pose>{leg2_x} {leg1_y} {leg_center_z} 0 0 0</pose>
        <geometry><box><size>0.04 0.04 {leg_height}</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="leg4">
        <pose>{leg2_x} {leg2_y} {leg_center_z} 0 0 0</pose>
        <geometry><box><size>0.04 0.04 {leg_height}</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>""".format(
            table_center=table_center,
            table_size=table_size,
            leg1_x=leg1_x,
            leg2_x=leg2_x,
            leg1_y=leg1_y,
            leg2_y=leg2_y,
            leg_center_z=leg_center_z,
            leg_height=leg_height,
        )

    def get_zone_sdf(name, color_rgba):
        return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="border">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.09 0.09 0.001</size></box></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
      <visual name="pad">
        <pose>0 0 0.0005 0 0 0</pose>
        <geometry><box><size>0.075 0.075 0.001</size></box></geometry>
        <material>
          <ambient>{color_rgba}</ambient>
          <diffuse>{color_rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    # Spawn table
    nodes.append(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'table', '-string', get_table_sdf(), '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    ))

    # Spawn zones (zone_a, zone_b, zone_c)
    zone_colors = {
        'zone_a': '1.0 0.85 0.0 0.9',  # Yellow (matches student mapping)
        'zone_b': '0.1 0.45 1.0 0.9',  # Blue
        'zone_c': '1.0 0.2 0.2 0.9'    # Red
    }
    for zone_name, zone_data in scene.get('zones', {}).items():
        pos = zone_data['position']
        color = zone_colors.get(zone_name, '0.5 0.5 0.5 0.9')
        nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', zone_name,
                '-string', get_zone_sdf(zone_name, color),
                '-x', str(pos[0]),
                '-y', str(pos[1]),
                '-z', str(pos[2])
            ],
            output='screen'
        ))

    # Spawn cubes (red_cube, yellow_cube, blue_cube)
    cube_colors = {
        'red_cube': '1.0 0.1 0.1 1.0',
        'yellow_cube': '1.0 0.9 0.0 1.0',
        'blue_cube': '0.1 0.4 1.0 1.0'
    }
    for obj_name, obj_data in scene.get('objects', {}).items():
        pos = obj_data['position']
        color = cube_colors.get(obj_name, '1 1 1 1')
        nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', obj_name,
                '-string', get_box_sdf(obj_name, color),
                '-x', str(pos[0]),
                '-y', str(pos[1]),
                '-z', str(pos[2])
            ],
            output='screen'
        ))

    launch_arguments = [
      DeclareLaunchArgument('launch_rviz', default_value='true'),
      DeclareLaunchArgument('gazebo_gui', default_value='true'),
    ]
    return LaunchDescription(launch_arguments + nodes)
