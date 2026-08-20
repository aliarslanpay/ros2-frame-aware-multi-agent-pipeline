import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from swarm_interfaces.msg import AgentSnapshot
from swarm_interfaces.msg import SwarmSnapshot


EXPECTED_AGENT_IDS = ["agent_1", "agent_2", "agent_3"]
EXPECTED_NODES = {
    ("state_publisher", "/agent_1"),
    ("state_publisher", "/agent_2"),
    ("state_publisher", "/agent_3"),
    ("swarm_coordinator", "/"),
}
SNAPSHOT_TOPIC = "/swarm/snapshot"


@launch_testing.markers.keep_alive
def generate_test_description():
    production_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("swarm_bringup"),
                    "launch",
                    "multi_agent_pipeline.launch.py",
                ]
            )
        ),
        launch_arguments={
            "agent_2_dropout_after_s": "8.0",
            "summary_period_ms": "250",
            "stale_timeout_ms": "750",
        }.items(),
    )

    return LaunchDescription(
        [
            production_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMultiAgentPipeline(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("swarm_snapshot_integration_test")
        self.snapshots = []
        self.sequence_error = None

        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.node.create_subscription(
            SwarmSnapshot,
            SNAPSHOT_TOPIC,
            self._snapshot_callback,
            snapshot_qos,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()
        rclpy.shutdown()

    def _snapshot_callback(self, message):
        if self.snapshots and message.sequence <= self.snapshots[-1].sequence:
            self.sequence_error = (
                "snapshot sequence did not increase: "
                f"previous={self.snapshots[-1].sequence}, "
                f"current={message.sequence}"
            )
        self.snapshots.append(message)

    def _diagnostic_summary(self):
        if not self.snapshots:
            return "no snapshots received"

        latest = self.snapshots[-1]
        agents = ", ".join(
            (
                f"{agent.agent_id}:has_state={agent.has_state},"
                f"freshness={agent.freshness},state_seq={agent.state.sequence}"
            )
            for agent in latest.agents
        )
        return (
            f"received={len(self.snapshots)}, latest_snapshot_seq={latest.sequence}, "
            f"latest_frame={latest.header.frame_id}, agents=[{agents}], "
            f"sequence_error={self.sequence_error}"
        )

    def _wait_for(self, probe, description, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.sequence_error is not None:
                self.fail(self.sequence_error)

            result = probe()
            if result is not None and result is not False:
                return result

            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=max(0.0, min(0.1, remaining)),
            )

        self.fail(
            f"Timed out after {timeout_sec:.1f}s waiting for {description}; "
            f"{self._diagnostic_summary()}"
        )

    @staticmethod
    def _agent_by_id(snapshot, agent_id):
        return next(
            (agent for agent in snapshot.agents if agent.agent_id == agent_id),
            None,
        )

    @staticmethod
    def _source_timestamp_is_nonzero(agent):
        stamp = agent.state.header.stamp
        return stamp.sec != 0 or stamp.nanosec != 0

    @classmethod
    def _is_complete_fresh_snapshot(cls, snapshot):
        if snapshot.header.frame_id != "map":
            return False
        if [agent.agent_id for agent in snapshot.agents] != EXPECTED_AGENT_IDS:
            return False

        return all(
            agent.has_state
            and agent.freshness == AgentSnapshot.FRESH
            and agent.state.header.frame_id == "map"
            and agent.agent_id == agent.state.agent_id
            and cls._source_timestamp_is_nonzero(agent)
            for agent in snapshot.agents
        )

    def _latest_complete_fresh_snapshot(self):
        return next(
            (
                snapshot
                for snapshot in reversed(self.snapshots)
                if self._is_complete_fresh_snapshot(snapshot)
            ),
            None,
        )

    def _find_stale_transition(self):
        for stale_index, snapshot in enumerate(self.snapshots):
            agent_2 = self._agent_by_id(snapshot, "agent_2")
            agent_1 = self._agent_by_id(snapshot, "agent_1")
            agent_3 = self._agent_by_id(snapshot, "agent_3")
            if any(agent is None for agent in (agent_1, agent_2, agent_3)):
                continue

            if not (
                agent_2.has_state
                and agent_2.freshness == AgentSnapshot.STALE
                and agent_1.has_state
                and agent_1.freshness == AgentSnapshot.FRESH
                and agent_3.has_state
                and agent_3.freshness == AgentSnapshot.FRESH
            ):
                continue

            stale_state = agent_2.state
            for previous in reversed(self.snapshots[:stale_index]):
                previous_agent_2 = self._agent_by_id(previous, "agent_2")
                if (
                    previous_agent_2 is not None
                    and previous_agent_2.has_state
                    and previous_agent_2.freshness == AgentSnapshot.FRESH
                    and previous_agent_2.state == stale_state
                ):
                    return snapshot, previous

        return None

    def test_typed_snapshot_normal_and_dropout_flow(self):
        self._wait_for(
            lambda: EXPECTED_NODES.issubset(
                set(self.node.get_node_names_and_namespaces())
            ),
            "three agent publishers and the coordinator to appear in the graph",
            10.0,
        )
        self._wait_for(
            lambda: self.node.count_publishers(SNAPSHOT_TOPIC) > 0,
            "a publisher on /swarm/snapshot",
            10.0,
        )

        normal_snapshot = self._wait_for(
            self._latest_complete_fresh_snapshot,
            "a complete three-agent FRESH snapshot in map",
            10.0,
        )

        self.assertEqual(
            [agent.agent_id for agent in normal_snapshot.agents],
            EXPECTED_AGENT_IDS,
        )
        self.assertEqual(normal_snapshot.header.frame_id, "map")
        for agent in normal_snapshot.agents:
            self.assertTrue(agent.has_state)
            self.assertEqual(agent.freshness, AgentSnapshot.FRESH)
            self.assertEqual(agent.state.header.frame_id, "map")
            self.assertEqual(agent.agent_id, agent.state.agent_id)
            self.assertTrue(self._source_timestamp_is_nonzero(agent))

        later_normal_snapshot = self._wait_for(
            lambda: next(
                (
                    snapshot
                    for snapshot in reversed(self.snapshots)
                    if snapshot.sequence >= normal_snapshot.sequence + 2
                ),
                None,
            ),
            "at least two later monotonically sequenced snapshots",
            3.0,
        )
        self.assertGreater(later_normal_snapshot.sequence, normal_snapshot.sequence)
        self.assertIsNone(self.sequence_error)

        stale_snapshot, last_fresh_snapshot = self._wait_for(
            self._find_stale_transition,
            "agent_2 to retain its last accepted state while becoming STALE",
            12.0,
        )
        stale_agent_2 = self._agent_by_id(stale_snapshot, "agent_2")
        last_fresh_agent_2 = self._agent_by_id(last_fresh_snapshot, "agent_2")
        self.assertTrue(stale_agent_2.has_state)
        self.assertEqual(stale_agent_2.freshness, AgentSnapshot.STALE)
        self.assertEqual(stale_agent_2.state.header.frame_id, "map")
        self.assertEqual(stale_agent_2.agent_id, stale_agent_2.state.agent_id)
        self.assertEqual(stale_agent_2.state, last_fresh_agent_2.state)
        self.assertEqual(
            self._agent_by_id(stale_snapshot, "agent_1").freshness,
            AgentSnapshot.FRESH,
        )
        self.assertEqual(
            self._agent_by_id(stale_snapshot, "agent_3").freshness,
            AgentSnapshot.FRESH,
        )

        post_stale_snapshot = self._wait_for(
            lambda: next(
                (
                    snapshot
                    for snapshot in reversed(self.snapshots)
                    if snapshot.sequence > stale_snapshot.sequence
                    and self._agent_by_id(snapshot, "agent_2") is not None
                    and self._agent_by_id(snapshot, "agent_2").has_state
                    and self._agent_by_id(
                        snapshot, "agent_2"
                    ).freshness == AgentSnapshot.STALE
                ),
                None,
            ),
            "snapshot sequence to continue after agent_2 becomes STALE",
            3.0,
        )
        self.assertGreater(post_stale_snapshot.sequence, stale_snapshot.sequence)
        self.assertIsNone(self.sequence_error)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
