# Architecture

## System boundary

The project processes timestamped state from a fixed set of simulated agents.
Each agent publishes in its own odometry frame. The coordinator converts valid
samples into a configured common frame and retains the latest accepted state
for coherent multi-agent summaries.

The graph contains three instances of `agent_state_publisher` and one
`swarm_coordinator_node`. A relative `state` topic in the publisher resolves
through each namespace to `/agent_N/state`. The coordinator derives the same
fully qualified topics from its configured `agent_ids` list.

## Package responsibilities

- `swarm_interfaces` defines the producer/consumer message contract.
- `swarm_core` owns source-timestamp ordering logic without ROS node behavior.
- `agent_simulator` produces deterministic motion, state, and TF.
- `swarm_coordinator` owns normalization, validation, caching, freshness, and
  summary behavior.
- `swarm_bringup` supplies one reproducible three-agent deployment.

This split keeps the message contract and pure ordering rule independent from
the simulator and coordinator applications.

## Processing path

For every received sample, the coordinator:

1. Associates the subscription with an expected agent identity.
2. Validates source and child frames, source timestamp, and pose quaternion.
3. Looks up the target-to-source transform at the source timestamp.
4. Transforms the pose and rotates the linear/angular velocity vectors.
5. Validates payload identity, timestamp order, and sequence order.
6. Stores an immutable replacement record in the latest-state cache.

The `FrameAwareStateProcessor` makes this ordering structural: cache update is
not called unless normalization succeeds. Rejected traffic therefore cannot
replace state or refresh freshness.

## State communication

Publishers and coordinator subscriptions use `rclcpp::SensorDataQoS`. State is
a high-rate, latest-value stream, so best-effort delivery with a small
keep-last history avoids spending resources retransmitting samples that have
already been superseded. Sequence gaps remain observable in the payload.

This profile is specific to state telemetry. It would not be appropriate for a
critical command or acknowledgement path.

## Cache and freshness

The cache is keyed by configured agent identity and retains the full normalized
message, its source timestamp, and a local steady-clock receipt time.
Summaries request all agents through one snapshot operation and classify them:

- `NEVER_SEEN`: no valid state has been accepted;
- `FRESH`: accepted-state age is at most the configured timeout;
- `STALE`: accepted-state age exceeds the timeout.

Receipt time uses `std::chrono::steady_clock` because staleness asks how long
this coordinator has gone without usable input. It is intentionally distinct
from source time, which is used for transformation and ordering.

## Failure handling

The pipeline rejects identity mismatches, invalid frames or quaternions,
zero/invalid source timestamps, unavailable transforms, duplicate or
out-of-order timestamps, and non-increasing sequences. A sequence jump is
accepted when timestamp and sequence remain newer, while the missing count is
reported.

Transform lookup is immediate. The pipeline does not retain a pending queue;
the next high-rate state sample retries with its own timestamp. This keeps
memory and delayed ordering behavior bounded for a latest-state use case.

## Known limitations

Membership is static, publisher sessions are not represented, and a restarted
publisher whose sequence resets may be rejected against the cached run. The
project has no structured diagnostics, persistence, authentication, clock
offset estimation, or coordinator failover.

The summary is log output rather than a control API. No claim is made about
state estimation, navigation, distributed coordination, or hard real-time
execution.
