from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    # -------------------------
    # Package path
    # -------------------------
    pkg_share = FindPackageShare('six_dof_arm')

    # -------------------------
    # World file (SDF) from package
    # -------------------------
    world_sdf_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'my_world.sdf'
    ])

    # -------------------------
    # Robot description (Xacro → URDF)
    # -------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                pkg_share,
                'urdf',
                'arm.xacro'
            ])
        ]),
        value_type=str
    )

    # -------------------------
    # Gazebo resource path (meshes, models)
    # -------------------------
    gz_resource_path = os.path.expanduser("~/ros2_ws/src")

    # -------------------------
    # Start Gazebo with world
    # -------------------------
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            world_sdf_file
        ],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_resource_path
        }
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )

    # -------------------------
    # Joint State Publisher GUI
    # -------------------------
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # -------------------------
    # Spawn robot into Gazebo
    # -------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'six_dof_arm'
        ],
        output='screen'
    )
    
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )
        # -------------------------
    # RViz
    # -------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    #Ros to GZ bridge
    #Clock is added
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"bridge_names": ["clock_bridge"]},
            {"bridges.clock_bridge.ros_topic_name": "/clock"},
            {"bridges.clock_bridge.gz_topic_name": "/clock"},
            {"bridges.clock_bridge.ros_type_name": "rosgraph_msgs/msg/Clock"},
            {"bridges.clock_bridge.gz_type_name": "gz.msgs.Clock"},
            {"bridges.clock_bridge.direction": "GZ_TO_ROS"},
            {"bridges.clock_bridge.lazy": "False"},
            {"bridges.clock_bridge.qos_profile": "CLOCK"},
        ],
    )

    # -------------------------
    # Launch description
    # -------------------------
    return LaunchDescription([
       # gazebo,
        robot_state_publisher,
        joint_state_gui,
        #spawn_entity,
        rviz
    ])