# ROS 2 Frame-Aware Multi-Agent State Pipeline

A C++17/ROS 2 Jazzy pipeline that normalizes timestamped state from multiple
agents into a common coordinate frame and maintains a concurrency-safe
latest-state cache with a typed consumer-facing snapshot topic.

## Project overview

Three parameterized instances of one simulator publish `AgentState` messages
and TF for independent odometry/body frame pairs. A coordinator validates each
stream, performs an exact-source-time TF2 lookup, normalizes pose and velocity
into `map`, and stores only accepted state. Periodic summaries classify each
configured agent as `NEVER_SEEN`, `FRESH`, or `STALE`. Each timer callback
publishes that coherent view as `swarm_interfaces/msg/SwarmSnapshot` on
`/swarm/snapshot` and retains the existing human-readable log summary.

This repository focuses on the data-processing boundary between per-agent
state producers and a common-frame consumer. It does not implement estimation,
navigation, task allocation, or vehicle control.

## Key engineering features

- C++17 nodes and reusable processing components on ROS 2 Jazzy
- Custom `AgentState`, `AgentSnapshot`, and `SwarmSnapshot` contracts
- Three namespaced agent publishers built from one executable
- Deterministic straight-line and coordinated-turn motion
- Per-agent `odom` and `base_link` frames with TF2 publication
- Source-timestamp frame lookup and pose/vector normalization
- `SensorDataQoS` for freshness-oriented state streams
- Configurable `MultiThreadedExecutor` with per-agent callback groups
- Mutex-protected immutable cache records and coherent snapshots
- Structured `/swarm/snapshot` output with reliable, transient-local QoS
- Identity, frame, quaternion, timestamp, and sequence validation
- Sequence-gap detection, controlled dropout, and stale-state classification
- Unit, concurrency, stress, and frame-processing tests
- Launch-based typed-output integration testing of normal and dropout flows
- Optional ThreadSanitizer instrumentation

## Architecture

```mermaid
flowchart TD
    A["Agent publishers"] --> B["Namespaced state topics"]
    B --> C["Per-agent callback groups"]
    C --> D["TF2-backed frame normalizer"]
    T["TF2 buffer"] --> D
    D --> E["Validation and latest-state cache"]
    E --> F["One coherent cache view"]
    F --> G["Typed snapshot topic"]
    F --> H["Log summary"]
```

Normalization is deliberately upstream of cache mutation. A sample with a
missing transform, invalid frame, timestamp, or quaternion cannot replace the
last accepted state or refresh its receipt time.

See [Architecture](docs/architecture.md),
[Concurrency design](docs/concurrency-design.md), and
[Frame semantics](docs/frame-semantics.md) for the design details.

## Package layout

| Package | Responsibility |
| --- | --- |
| `swarm_interfaces` | Frame-aware state and structured snapshot contracts |
| `swarm_core` | Reusable source-timestamp ordering policy |
| `agent_simulator` | Deterministic state and TF publisher |
| `swarm_coordinator` | Frame normalization, validation, cache, executor policy, typed snapshots, and log summaries |
| `swarm_bringup` | Multi-agent launch configuration and end-to-end launch test |

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon` and `rosdep`
- `tf2_tools` for optional frame-tree inspection

Install the ROS-specific tooling if it is not already present:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

## Build

Run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

All C++ targets use C++17 with `-Wall -Wextra -Wpedantic` under GCC and Clang.

## Test

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

Test counts are reported by category because GTest cases, Python unittest
methods, CTest targets, and `colcon test-result` entries are different units:

- 37 GTest cases across six C++ GTest executables cover source ordering, cache
  and freshness invariants, concurrent readers/writers, frame validation and
  transformation, the normalizer-to-cache boundary, and structured snapshot
  conversion.
- One `launch_testing` target starts the installed production launch and runs
  one active end-to-end scenario plus one post-shutdown process-exit check.

Run only the multi-process integration test with live console output:

```bash
colcon test \
  --packages-select swarm_bringup \
  --ctest-args -R test_multi_agent_pipeline --output-on-failure \
  --event-handlers console_direct+

colcon test-result --verbose
```

After building and sourcing the workspace, the launch test can also be invoked
directly from the repository root:

```bash
ROS_DOMAIN_ID=73 \
  launch_test src/swarm_bringup/test/test_multi_agent_pipeline.py
```

For failure diagnosis, inspect both the package-level colcon output and the
CTest detail:

```bash
sed -n '1,260p' log/latest_test/swarm_bringup/stdout_stderr.log
sed -n '1,260p' build/swarm_bringup/Testing/Temporary/LastTest.log
```

The test includes `multi_agent_pipeline.launch.py` rather than duplicating the
graph definition. It subscribes to `/swarm/snapshot` with matching reliable,
transient-local QoS and conditionally waits for a complete FRESH snapshot,
agent 2 dropout, retained-state STALE classification, and continued snapshot
sequencing. The isolated test runner prevents dependence on the caller's
default ROS graph.

## Run

```bash
ros2 launch swarm_bringup multi_agent_pipeline.launch.py
```

The summary reports source states after normalization into `map`, for example:

```text
Swarm summary: agent_1=[FRESH seq=... frame=map p=(...)] agent_2=[FRESH ...]
```

A small number of transform rejections may occur during startup before static
TF data reaches the coordinator. Later samples should be accepted.

In a second terminal, source ROS 2 and this workspace, then inspect the typed
consumer-facing output:

```bash
ros2 topic info /swarm/snapshot --verbose
ros2 topic echo /swarm/snapshot
```

The topic uses keep-last depth 1, reliable delivery, and transient-local
durability. Unlike the high-rate best-effort input streams, it is a low-rate
latest-state view intended for downstream consumers and late joiners.

## Controlled dropout demonstration

```bash
ros2 launch swarm_bringup multi_agent_pipeline.launch.py \
  agent_2_dropout_after_s:=4.0
```

Agent 2 stops publishing state and dynamic TF after four seconds. Its last
accepted record remains available and becomes `STALE` after the configured
1.5-second timeout.

## Frame-tree inspection

In a second terminal, source ROS 2 and this workspace, then run:

```bash
ros2 topic echo /agent_2/state --once
ros2 run tf2_ros tf2_echo map agent_2/odom
ros2 run tf2_ros tf2_echo map agent_2/base_link
ros2 run tf2_tools view_frames
```

The source message uses `agent_2/odom` as `header.frame_id` and
`agent_2/base_link` as `child_frame_id`; the coordinator summary reports the
normalized record in `map`.

## Concurrency and TSan verification

Callback instrumentation is disabled by default. To demonstrate permitted
cross-agent overlap:

```bash
ros2 launch swarm_bringup multi_agent_pipeline.launch.py \
  coordinator_executor_threads:=4 \
  instrument_callbacks:=true \
  processing_delay_ms:=200
```

An `active_agent_callbacks` value above one shows callbacks from different
agent groups overlapping. Each agent subscription remains in its own
`MutuallyExclusive` group.

For an isolated ThreadSanitizer run of the cache concurrency suite:

```bash
source /opt/ros/jazzy/setup.bash

colcon --log-base log-tsan build \
  --build-base build-tsan \
  --install-base install-tsan \
  --packages-up-to swarm_coordinator \
  --cmake-args \
    -DSWARM_COORDINATOR_ENABLE_TSAN=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install-tsan/setup.bash
TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1 \
  ./build-tsan/swarm_coordinator/test_agent_state_cache_concurrency
```

## Design decisions

- `SensorDataQoS` favors recent state over retransmission of old samples.
- The snapshot output uses reliable, transient-local keep-last-1 QoS because
  it is a low-rate aggregate view rather than high-rate sensor telemetry.
- One `MutuallyExclusive` callback group per agent permits cross-agent
  parallelism without same-agent callback overlap.
- Immutable cache records keep post-lock reads safe and lock hold times short.
- One snapshot API binds state, receipt time, age, and freshness to a coherent
  cache view; the typed topic and log summary are generated from that same view.
- TF lookup uses the message source timestamp; zero/latest timestamps are
  rejected.
- Pose receives translation and rotation, while the project-defined velocity
  vectors receive rotation only.
- Transform rejection drops the current sample rather than queueing stale work.
- Staleness uses local steady-clock receipt time, independent of source clocks.

## Scope and limitations

- Membership is configured at startup through an explicit agent list.
- Sequence validation assumes one stable publisher run; restart/session
  identity is not yet represented in `AgentState`.
- Unavailable transforms cause immediate sample rejection; there is no delayed
  message queue.
- Health is a coarse simulated field, not a structured diagnostics system.
- The motion model is deterministic test input, not vehicle dynamics.
- The pipeline is not a state estimator, navigation stack, swarm controller,
  sensor-fusion system, or hard real-time executor.
