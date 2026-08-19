# Architecture

## System boundary

The project processes timestamped state from a fixed set of simulated agents.
Each agent publishes in its own odometry frame. The coordinator converts valid
samples into a configured common frame and retains the latest accepted state
for coherent multi-agent snapshots and summaries.

The graph contains three instances of `agent_state_publisher` and one
`swarm_coordinator_node`. A relative `state` topic in the publisher resolves
through each namespace to `/agent_N/state`. The coordinator derives the same
fully qualified topics from its configured `agent_ids` list.

## Package responsibilities

- `swarm_interfaces` defines the producer state contract and consumer snapshot
  contracts.
- `swarm_core` owns source-timestamp ordering logic without ROS node behavior.
- `agent_simulator` produces deterministic motion, state, and TF.
- `swarm_coordinator` owns normalization, validation, caching, freshness,
  structured snapshot publication, and log summary behavior.
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

## State communication and QoS

Publishers and coordinator subscriptions use `rclcpp::SensorDataQoS`. State is
a high-rate, latest-value stream, so best-effort delivery with a small
keep-last history avoids spending resources retransmitting samples that have
already been superseded. Sequence gaps remain observable in the payload.

This profile is specific to state telemetry. It would not be appropriate for a
critical command or acknowledgement path.

The coordinator publishes `swarm_interfaces/msg/SwarmSnapshot` on the relative
topic `swarm/snapshot`, which resolves to `/swarm/snapshot` in the supplied
root namespace. This low-rate latest-state view uses keep-last depth 1,
reliable delivery, and transient-local durability. Reliable delivery is
appropriate because summaries are not superseded at sensor rate, while
transient local lets a late-joining consumer immediately receive the newest
available view. The input and output profiles intentionally serve different
traffic semantics.

## Cache and freshness

The cache is keyed by configured agent identity and retains the full normalized
message, its source timestamp, and a local steady-clock receipt time.
The timer requests all agents through one snapshot operation and classifies them:

- `NEVER_SEEN`: no valid state has been accepted;
- `FRESH`: accepted-state age is at most the configured timeout;
- `STALE`: accepted-state age exceeds the timeout.

Receipt time uses `std::chrono::steady_clock` because staleness asks how long
this coordinator has gone without usable input. It is intentionally distinct
from source time, which is used for transformation and ordering.

After that single coherent cache view has been copied, the cache mutex is no
longer held. The coordinator converts the view to a typed ROS message,
publishes it, and formats the existing log summary from the same immutable
records. Topic and log cannot accidentally describe cache states captured at
different times.

## Snapshot contract and timestamps

`SwarmSnapshot.header.stamp` is the ROS time when the coordinator creates the
output message, and `header.frame_id` is the configured target frame. Its
process-local sequence increases once per timer callback. Agent entries remain
in configured `agent_ids` order.

For an accepted state, `AgentSnapshot.has_state` is true and the nested
`AgentState` remains normalized into the target frame. Its nested
`header.stamp` is deliberately unchanged: it is the producer's original source
timestamp, not the snapshot creation time. `age` is derived from coordinator
steady-clock receipt time and is represented as `builtin_interfaces/Duration`.
Negative ages clamp to zero and values beyond the message range saturate.

`NEVER_SEEN` entries set `has_state=false`, use zero age, and leave the nested
state at its default invalid value. Consumers must inspect `has_state` before
using that nested state. `FRESH` and `STALE` entries have `has_state=true`.

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

The snapshot topic is a latest-state observation API, not a command, control,
or diagnostics interface. No claim is made about state estimation, navigation,
distributed coordination, or hard real-time execution. Launch-based
publisher/subscriber integration testing is reserved for a later stage.
