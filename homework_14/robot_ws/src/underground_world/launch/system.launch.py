from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Сценарій можна змінити без редагування launch-файлу:
    # ros2 launch underground_world system.launch.py scenario:=small_rooms.yaml
    scenario = LaunchConfiguration("scenario")
    move_commit_period_ms = LaunchConfiguration("move_commit_period_ms")
    scenario_path = PathJoinSubstitution(
        [FindPackageShare("underground_world"), "config", scenario]
    )

    world_node = Node(
        package="underground_world",
        executable="underground_world_node",
        name="underground_world_node",
        output="screen",
        parameters=[
            {
                "scenario_path": scenario_path,
                "move_commit_period_ms": ParameterValue(
                    move_commit_period_ms, value_type=int
                ),
            }
        ],
    )

    # Ноди рішення стартують першими, щоб гарантовано підписатися на
    # /robot/local_scan ДО одноразової стартової публікації світу.
    payload_action_node = Node(
        package="mission_control",
        executable="payload_action",
        name="payload_action",
        output="screen",
    )

    mission_explorer_node = Node(
        package="mission_control",
        executable="mission_explorer",
        name="mission_explorer",
        output="screen",
    )

    # underground_world_node публікує стартовий стан один раз одразу після
    # запуску; затримка 2 c усуває гонку discovery на старті системи.
    delayed_world_node = TimerAction(period=2.0, actions=[world_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="training_corridor.yaml",
                description="Scenario YAML file from underground_world/config",
            ),
            DeclareLaunchArgument(
                "move_commit_period_ms",
                default_value="50",
                description="Delay before applying queued move commands",
            ),
            payload_action_node,
            mission_explorer_node,
            delayed_world_node,
        ]
    )
