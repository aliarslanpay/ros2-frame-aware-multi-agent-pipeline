# Concurrency design

## Execution policy

The coordinator runs in a configurable `MultiThreadedExecutor`, with four
workers by default. Each agent subscription owns a dedicated
`MutuallyExclusive` callback group, and the summary timer owns a separate
group.

This topology permits callbacks for different agents to overlap while
preventing two callbacks for the same agent from executing concurrently.
Timestamp and sequence validation remain the source-of-truth ordering checks;
callback groups limit scheduler-dependent same-agent behavior.

The node retains strong references to callback groups and subscriptions for as
long as their callbacks may execute.

## Shared-state synchronization

Different callback groups can access one `std::unordered_map` concurrently, so
callback-group isolation alone is not sufficient. `AgentStateCache` uses one
`std::mutex` around map lookup, invariant checks, pointer replacement, and
coherent snapshot capture.

A single mutex is appropriate for the current small membership because:

- cache critical sections are short;
- summaries require a coherent cross-agent view;
- per-agent locks would add lock-ordering complexity;
- one low-rate reader does not justify `std::shared_mutex` overhead.

TF lookup, quaternion work, ROS message allocation/copying, formatting, and
logging occur outside the cache lock.

## Immutable records

Accepted states are stored as `std::shared_ptr<const CachedAgentState>`. The
candidate record is fully constructed before acquiring the mutex. Under the
lock, an accepted update replaces only one pointer; readers that already hold
the previous pointer continue to see a valid immutable record.

This provides clear lifetime and exception-safety behavior and prevents a
summary from observing fields while a writer mutates them in place.

## Coherent snapshots

`snapshot()` returns, for every configured agent, the state pointer, receipt
age, and `NEVER_SEEN`/`FRESH`/`STALE` classification calculated from one
captured time. Copying the immutable pointers and calculating ages happen under
one lock; all output formatting happens after release.

Separate `find()` and `freshness()` calls would allow an update between the two
reads and could pair an older message with a newer receipt time. The single API
prevents that torn logical view.

If a concurrent update records a receipt time just after the summary captured
its `now`, the resulting small negative age is clamped to zero.

## Verification strategy

Concurrency tests use a condition-variable start gate and validate invariants
without requiring a particular thread to win a race. Coverage includes
parallel updates to different agents, snapshots racing with writers,
duplicate/out-of-order rejection, rejected traffic not refreshing freshness,
and mixed multi-writer stress.

The optional ThreadSanitizer build targets memory data races in executed paths.
It does not prove semantic ordering, deadlock freedom, unexecuted paths, or
deadline compliance; those concerns require separate tests and measurements.

## Trade-offs

The current callbacks are lightweight, so no performance benefit is claimed
for multiple executor threads. The design primarily makes concurrency policy
and shared-state correctness explicit and can avoid head-of-line blocking if
future per-agent work becomes heavier.

The executor, standard mutex, dynamic allocation, logging, operating-system
scheduling, and DDS middleware are not configured or measured for bounded
worst-case latency. This is not a hard real-time design.
