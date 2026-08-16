# Coordinate frame and source timestamp of this state estimate.
std_msgs/Header header

# Stable logical identity of the agent that produced this message.
string agent_id

# Agent body frame whose pose is expressed in header.frame_id.
string child_frame_id

# Monotonically increasing counter for this publisher run.
uint64 sequence

# Pose of child_frame_id expressed in header.frame_id.
geometry_msgs/Pose pose

# Linear and angular velocity vectors expressed in header.frame_id at the
# agent origin. Changing the reference frame rotates these vectors; the
# transform translation is intentionally not applied to them.
geometry_msgs/Twist velocity

# Coarse health state used by coordinators and health monitors.
uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_NOMINAL=1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_FAILED=3
uint8 health
