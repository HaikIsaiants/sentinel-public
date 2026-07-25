from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trace = LaunchConfiguration("trace")
    playback_rate = LaunchConfiguration("playback_rate")
    gz_args = LaunchConfiguration("gz_args")
    # TODO: make the world name a launch arg when there is a second scene
    world = PathJoinSubstitution([FindPackageShare("sentinel_gazebo"), "worlds", "demo.sdf"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [gz_args, " ", world]}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/sentinel/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            "/world/sentinel/remove@ros_gz_interfaces/srv/DeleteEntity",
        ],
        output="screen",
    )
    adapter = Node(
        package="sentinel_gazebo",
        executable="sentinel_gazebo_adapter",
        parameters=[{"trace": trace, "playback_rate": playback_rate, "world": "sentinel"}],
        output="screen",
    )
    shutdown = RegisterEventHandler(OnProcessExit(target_action=adapter, on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription([
        DeclareLaunchArgument("trace"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("gz_args", default_value="-r -v 2"),
        gazebo,
        bridge,
        adapter,
        shutdown,
    ])
