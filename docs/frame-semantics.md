# Frame semantics

## Frame contract

Each agent publishes the following tree:

```text
map -> agent_N/odom -> agent_N/base_link
```

- `map` is the common comparison frame used by the coordinator.
- `agent_N/odom` is the agent's local motion frame.
- `agent_N/base_link` is the moving body frame.

`AgentState` defines:

| Field | Meaning |
| --- | --- |
| `header.frame_id` | Reference frame for pose and velocity |
| `child_frame_id` | Body frame described by the pose |
| `header.stamp` | Source time of the state sample |
| `pose` | `child_frame_id` pose expressed in `header.frame_id` |
| `velocity` | Linear and angular vectors expressed in `header.frame_id` at the agent origin |

The simulator publishes `map -> odom` as static TF and `odom -> base_link` as
dynamic TF. The dynamic transform and `AgentState` share one kinematic sample
and ROS timestamp.

## Timestamp-aware lookup

`StateFrameNormalizer` derives the expected source and child frames from the
configured agent identity and requests:

```cpp
lookupTransform(target_frame, source_frame, rclcpp::Time{source_stamp});
```

The lookup uses the message's source timestamp. Applying the latest available
transform could mix data from different times and introduce motion-dependent
spatial error. Timestamp zero is rejected because TF2 interprets it as a
request for the latest transform.

## Pose and velocity transformation

A pose is tied to a coordinate origin. Transforming a pose from `odom` to
`map` therefore applies both the frame rotation and translation.

This message defines linear and angular velocity as vectors at the agent
origin. Changing only their coordinate frame rotates the vectors; it does not
translate them. This is narrower than a full spatial-adjoint twist transform
between different reference points, where an additional cross-product term may
be required.

Source and transform quaternions must be finite and have non-negligible norm.
They are normalized before transformation.

## Missing transforms

If the exact-time transform is unavailable, the sample is rejected before the
cache is touched. This guarantees that unusable state cannot refresh an
agent's liveness classification.

Queueing through `tf2_ros::MessageFilter` would be appropriate if every sample
had to wait for delayed TF. This pipeline instead maintains only latest state
over a high-rate best-effort stream. Immediate rejection avoids pending-memory,
timeout, overflow, and delayed-order policies; a newer sample retries on its
next callback.

The trade-off is that a sample arriving just before its transform is dropped.
That policy should be revisited for measurement pipelines where every sample
must contribute to estimation or fusion.
