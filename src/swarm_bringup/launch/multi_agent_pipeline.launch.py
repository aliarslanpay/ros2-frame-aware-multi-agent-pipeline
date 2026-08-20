import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_agent_node(config, dropout_after_s=-1.0):
    agent_id = config["agent_id"]
    return Node(
        package="agent_simulator",
        executable="agent_state_publisher",
        namespace=agent_id,
        name="state_publisher",
        output="screen",
        parameters=[
            {
                "agent_id": agent_id,
                "map_frame_id": "map",
                "odom_frame_id": f"{agent_id}/odom",
                "base_frame_id": f"{agent_id}/base_link",
                "publish_rate_hz": config["publish_rate_hz"],
                "initial_x_m": config["initial_x_m"],
                "initial_y_m": config["initial_y_m"],
                "initial_z_m": config["initial_z_m"],
                "initial_yaw_rad": config["initial_yaw_rad"],
                "velocity_x_mps": config["velocity_x_mps"],
                "velocity_y_mps": config["velocity_y_mps"],
                "velocity_z_mps": config["velocity_z_mps"],
                "yaw_rate_radps": config["yaw_rate_radps"],
                "map_origin_x_m": config["map_origin_x_m"],
                "map_origin_y_m": config["map_origin_y_m"],
                "map_origin_z_m": config["map_origin_z_m"],
                "map_origin_yaw_rad": config["map_origin_yaw_rad"],
                "simulate_dropout_after_s": dropout_after_s,
            }
        ],
    )


def generate_launch_description():
    agent_2_dropout_after_s = LaunchConfiguration("agent_2_dropout_after_s")
    coordinator_executor_threads = LaunchConfiguration(
        "coordinator_executor_threads"
    )
    summary_period_ms = LaunchConfiguration("summary_period_ms")
    stale_timeout_ms = LaunchConfiguration("stale_timeout_ms")
    instrument_callbacks = LaunchConfiguration("instrument_callbacks")
    processing_delay_ms = LaunchConfiguration("processing_delay_ms")

    # Each agent moves in its own odometry frame. map_origin_* defines the
    # static map -> odom transform; the publisher supplies odom -> base_link.
    agent_configs = [
        {
            "agent_id": "agent_1",
            "publish_rate_hz": 10.0,
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_z_m": 0.0,
            "initial_yaw_rad": 0.0,
            "velocity_x_mps": 0.50,
            "velocity_y_mps": 0.00,
            "velocity_z_mps": 0.00,
            "yaw_rate_radps": 0.10,
            "map_origin_x_m": 0.0,
            "map_origin_y_m": 0.0,
            "map_origin_z_m": 0.0,
            "map_origin_yaw_rad": 0.0,
        },
        {
            "agent_id": "agent_2",
            "publish_rate_hz": 7.0,
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_z_m": 0.0,
            "initial_yaw_rad": 0.0,
            "velocity_x_mps": 0.35,
            "velocity_y_mps": 0.00,
            "velocity_z_mps": 0.00,
            "yaw_rate_radps": -0.08,
            "map_origin_x_m": 5.0,
            "map_origin_y_m": 0.0,
            "map_origin_z_m": 0.0,
            "map_origin_yaw_rad": math.pi / 2.0,
        },
        {
            "agent_id": "agent_3",
            "publish_rate_hz": 12.0,
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_z_m": 0.0,
            "initial_yaw_rad": 0.25,
            "velocity_x_mps": 0.35 * math.cos(0.25),
            "velocity_y_mps": 0.35 * math.sin(0.25),
            "velocity_z_mps": 0.00,
            "yaw_rate_radps": 0.12,
            "map_origin_x_m": -3.0,
            "map_origin_y_m": -2.0,
            "map_origin_z_m": 0.0,
            "map_origin_yaw_rad": -math.pi / 4.0,
        },
    ]

    agent_nodes = [
        make_agent_node(agent_configs[0]),
        make_agent_node(
            agent_configs[1],
            ParameterValue(agent_2_dropout_after_s, value_type=float),
        ),
        make_agent_node(agent_configs[2]),
    ]

    coordinator = Node(
        package="swarm_coordinator",
        executable="swarm_coordinator_node",
        name="swarm_coordinator",
        output="screen",
        parameters=[
            {
                "agent_ids": [config["agent_id"] for config in agent_configs],
                "target_frame": "map",
                "summary_period_ms": ParameterValue(
                    summary_period_ms, value_type=int
                ),
                "stale_timeout_ms": ParameterValue(
                    stale_timeout_ms, value_type=int
                ),
                "executor_threads": ParameterValue(
                    coordinator_executor_threads, value_type=int
                ),
                "instrument_callbacks": ParameterValue(
                    instrument_callbacks, value_type=bool
                ),
                "processing_delay_ms": ParameterValue(
                    processing_delay_ms, value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_2_dropout_after_s",
                default_value="-1.0",
                description=(
                    "Stop agent_2 state/TF publication after this many seconds; "
                    "negative disables the dropout simulation."
                ),
            ),
            DeclareLaunchArgument(
                "coordinator_executor_threads",
                default_value="4",
                description="Worker-thread count for the coordinator executor.",
            ),
            DeclareLaunchArgument(
                "summary_period_ms",
                default_value="1000",
                description=(
                    "Period for structured snapshots and log summaries in "
                    "milliseconds."
                ),
            ),
            DeclareLaunchArgument(
                "stale_timeout_ms",
                default_value="1500",
                description=(
                    "Receipt-age threshold for classifying accepted state as "
                    "stale, in milliseconds."
                ),
            ),
            DeclareLaunchArgument(
                "instrument_callbacks",
                default_value="false",
                description="Log callback thread IDs and concurrent activity.",
            ),
            DeclareLaunchArgument(
                "processing_delay_ms",
                default_value="0",
                description=(
                    "Artificial agent-callback delay used only when instrumentation "
                    "is enabled."
                ),
            ),
            *agent_nodes,
            coordinator,
        ]
    )
