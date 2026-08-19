# Stable logical identity from the coordinator's configured agent list.
string agent_id

# True only when state contains the last accepted, target-frame-normalized state.
# Consumers must ignore state when this field is false.
bool has_state

# Freshness relative to coordinator receipt time.
uint8 NEVER_SEEN=0
uint8 FRESH=1
uint8 STALE=2
uint8 freshness

# Default/invalid when has_state is false. Its header stamp remains the
# original source timestamp when has_state is true.
AgentState state

# Age of the last accepted state from coordinator receipt time. Zero for
# NEVER_SEEN; conversion clamps negative values and saturates overflow.
builtin_interfaces/Duration age
