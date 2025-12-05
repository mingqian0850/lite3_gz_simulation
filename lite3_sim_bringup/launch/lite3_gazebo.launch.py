import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    description_share = get_package_share_directory("lite3_sim_description")
    bringup_share = get_package_share_directory("lite3_sim_bringup")
    gazebo_share = get_package_share_directory("gazebo_ros")

    xacro_file = PathJoinSubstitution([description_share, "urdf", "lite3.xacro"])
    default_rviz_config = os.path.join(bringup_share, "rviz", "lite3_sim.rviz")

    robot_description = ParameterValue(Command(["xacro", " ", xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=robot_namespace,
        arguments=[
            "-entity",
            "lite3",
            "-topic",
            "robot_description",
            "-robot_namespace",
            robot_namespace,
        ],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=robot_namespace,
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock provided by Gazebo."
            ),
            DeclareLaunchArgument(
                "robot_namespace", default_value="lite3", description="Namespace applied to spawned robot."
            ),
            DeclareLaunchArgument(
                "start_rviz", default_value="true", description="Set false to skip RViz2 when launching."
            ),
            DeclareLaunchArgument(
                "rviz_config", default_value=default_rviz_config, description="RViz configuration file."
            ),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            rviz,
        ]
    )

