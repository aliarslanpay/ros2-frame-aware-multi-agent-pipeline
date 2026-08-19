# Creation time and configured target frame of this coordinator snapshot.
std_msgs/Header header

# Monotonically increasing sequence for this coordinator process.
uint64 sequence

# One entry per configured agent, in configured agent_ids order.
AgentSnapshot[] agents
