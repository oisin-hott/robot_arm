from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():

    # -------------------------
    # Package paths
    # -------------------------
    pkg_share = FindPackageShare('six_dof_arm')
    bringup_share = FindPackageShare('six_dof_arm_bringup')

    # -------------------------
    # URDF (xacro -> robot_description)
    # -------------------------
    xacro_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'arm.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # -------------------------
    # Controllers config
    # -------------------------
    controllers_file = PathJoinSubstitution([
        bringup_share,
        'config',
        'controllers.yaml'
    ])

    # -------------------------
    # Pose config
    # -------------------------
    poses_file = PathJoinSubstitution([
        bringup_share,
        'config',
        'poses.yaml'
    ])

    # -------------------------
    # MoveIt
    # -------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            "six_dof_arm",
            package_name="six_dof_arm_moveit_config"
        )
        .robot_description(file_path="config/six_dof_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/six_dof_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["stomp"], default_planning_pipeline="stomp")
        .to_moveit_configs()
    )

    import yaml
    print("=== PLANNING PIPELINES ===")
    print(yaml.dump(moveit_config.planning_pipelines))
    print("=== END ===")

    # -------------------------
    # Robot State Publisher
    # -------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {"use_sim_time": False}     # ← real time, no Gazebo clock
        ],
        output='screen'
    )

    # -------------------------
    # ros2_control node (controller manager — no Gazebo hosting it)
    # -------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file,
            {"use_sim_time": False}
            # removed robot_description here — gets it from /robot_description topic
        ],
        output='screen'
    )
    # -------------------------
    # Static TF: world -> base_link
    # -------------------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # -------------------------
    # Spawners
    # -------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{"use_sim_time": False}],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        parameters=[{"use_sim_time": False}],
        output='screen'
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # -------------------------
    # Arm motion node
    # -------------------------
    arm_motion_node = Node(
        package='six_dof_arm_bringup',
        executable='arm_motion_node',
        output='screen',
        parameters=[
            poses_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}     # ← real time
        ]
    )

    # -------------------------
    # Differential mixer
    # -------------------------
    differential_mixer = Node(
        package="six_dof_arm_bringup",
        executable="differential_mixer",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # -------------------------
    # MoveGroup
    # -------------------------
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # -------------------------
    # RViz
    # -------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},    # ← real time
        ],
    )

    # -------------------------
    # Event-driven ordering
    # -------------------------
    spawn_jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    spawn_gripper_after_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gripper_spawner],
        )
    )

    start_arm_motion_after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[arm_motion_node],
        )
    )

    # -------------------------
    # Launch description
    # -------------------------
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        static_tf,
        run_move_group_node,
        rviz,
        differential_mixer,
        joint_state_broadcaster_spawner,
        spawn_jtc_after_jsb,
        spawn_gripper_after_jtc,
        start_arm_motion_after_gripper,
    ])