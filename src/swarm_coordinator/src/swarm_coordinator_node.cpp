#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_coordinator/frame_aware_state_processor.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace swarm_coordinator
{
namespace
{

class CallbackActivity final
{
public:
  CallbackActivity(
    const rclcpp::Logger & logger,
    std::string agent_id,
    std::atomic<std::size_t> & active_callbacks)
  : logger_(logger),
    agent_id_(std::move(agent_id)),
    active_callbacks_(active_callbacks),
    thread_id_(thread_id_string())
  {
    const auto active = active_callbacks_.fetch_add(1, std::memory_order_relaxed) + 1;
    RCLCPP_INFO(
      logger_, "Callback START agent=%s thread=%s active_agent_callbacks=%zu",
      agent_id_.c_str(), thread_id_.c_str(), active);
  }

  ~CallbackActivity() noexcept
  {
    const auto active = active_callbacks_.fetch_sub(1, std::memory_order_relaxed) - 1;
    RCLCPP_INFO(
      logger_, "Callback END   agent=%s thread=%s active_agent_callbacks=%zu",
      agent_id_.c_str(), thread_id_.c_str(), active);
  }

  CallbackActivity(const CallbackActivity &) = delete;
  CallbackActivity & operator=(const CallbackActivity &) = delete;
  CallbackActivity(CallbackActivity &&) = delete;
  CallbackActivity & operator=(CallbackActivity &&) = delete;

private:
  [[nodiscard]] static std::string thread_id_string()
  {
    std::ostringstream output;
    output << std::this_thread::get_id();
    return output.str();
  }

  rclcpp::Logger logger_;
  std::string agent_id_;
  std::atomic<std::size_t> & active_callbacks_;
  std::string thread_id_;
};

}  // namespace

class SwarmCoordinatorNode final : public rclcpp::Node
{
public:
  SwarmCoordinatorNode()
  : Node("swarm_coordinator")
  {
    agent_ids_ = declare_parameter<std::vector<std::string>>(
      "agent_ids", std::vector<std::string>{"agent_1", "agent_2", "agent_3"});
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    const auto summary_period_ms =
      declare_parameter<std::int64_t>("summary_period_ms", 1000);
    const auto stale_timeout_ms =
      declare_parameter<std::int64_t>("stale_timeout_ms", 1500);
    const auto executor_threads =
      declare_parameter<std::int64_t>("executor_threads", 4);
    instrument_callbacks_ =
      declare_parameter<bool>("instrument_callbacks", false);
    const auto processing_delay_ms =
      declare_parameter<std::int64_t>("processing_delay_ms", 0);

    validate_configuration(
      summary_period_ms, stale_timeout_ms, executor_threads, processing_delay_ms);
    stale_timeout_ = std::chrono::milliseconds{stale_timeout_ms};
    executor_threads_ = static_cast<std::size_t>(executor_threads);
    processing_delay_ = std::chrono::milliseconds{processing_delay_ms};

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    state_processor_ = std::make_unique<FrameAwareStateProcessor>(
      target_frame_,
      [this](
        const std::string & target_frame,
        const std::string & source_frame,
        const builtin_interfaces::msg::Time & source_stamp)
      {
        try {
          // The three-argument lookup is immediate: it queries the exact source
          // timestamp and never blocks an agent callback waiting for future TF.
          return TransformLookupResult{
            tf_buffer_->lookupTransform(
              target_frame, source_frame, rclcpp::Time{source_stamp}),
            {}};
        } catch (const tf2::TransformException & exception) {
          return TransformLookupResult{std::nullopt, exception.what()};
        }
      });

    if (!instrument_callbacks_ && processing_delay_ms > 0) {
      RCLCPP_WARN(
        get_logger(),
        "processing_delay_ms is ignored unless instrument_callbacks is true");
    }

    subscriptions_.reserve(agent_ids_.size());
    agent_callback_groups_.reserve(agent_ids_.size());

    for (const auto & agent_id : agent_ids_) {
      // One MutuallyExclusive group per agent: callbacks for the same agent are
      // serialized, while groups belonging to different agents may run in parallel.
      auto callback_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      agent_callback_groups_.push_back(callback_group);

      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group;

      const std::string topic_name = "/" + agent_id + "/state";
      auto subscription = create_subscription<swarm_interfaces::msg::AgentState>(
        topic_name,
        rclcpp::SensorDataQoS{},
        [this, expected_agent_id = agent_id](
          const swarm_interfaces::msg::AgentState::SharedPtr message)
        {
          handle_state(expected_agent_id, *message);
        },
        options);

      subscriptions_.push_back(std::move(subscription));
      RCLCPP_INFO(
        get_logger(), "Expecting agent '%s' on '%s' in a dedicated callback group",
        agent_id.c_str(), topic_name.c_str());
    }

    // The timer can overlap agent processing but receives one coherent cache snapshot.
    summary_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    summary_timer_ = create_wall_timer(
      std::chrono::milliseconds{summary_period_ms},
      std::bind(&SwarmCoordinatorNode::publish_summary, this),
      summary_callback_group_);
  }

  [[nodiscard]] std::size_t executor_threads() const noexcept
  {
    return executor_threads_;
  }

private:
  void validate_configuration(
    const std::int64_t summary_period_ms,
    const std::int64_t stale_timeout_ms,
    const std::int64_t executor_threads,
    const std::int64_t processing_delay_ms) const
  {
    if (agent_ids_.empty()) {
      throw std::invalid_argument("agent_ids must contain at least one agent");
    }
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    if (summary_period_ms <= 0) {
      throw std::invalid_argument("summary_period_ms must be greater than zero");
    }
    if (stale_timeout_ms <= 0) {
      throw std::invalid_argument("stale_timeout_ms must be greater than zero");
    }
    if (executor_threads < 2 || executor_threads > 64) {
      throw std::invalid_argument("executor_threads must be in the range [2, 64]");
    }
    if (processing_delay_ms < 0 || processing_delay_ms > 60'000) {
      throw std::invalid_argument("processing_delay_ms must be in the range [0, 60000]");
    }

    std::unordered_set<std::string> unique_ids;
    for (const auto & agent_id : agent_ids_) {
      if (agent_id.empty() || agent_id.find('/') != std::string::npos) {
        throw std::invalid_argument(
                "each agent_id must be a non-empty namespace token without '/'");
      }
      if (!unique_ids.insert(agent_id).second) {
        throw std::invalid_argument("agent_ids must not contain duplicates");
      }
    }
  }

  void handle_state(
    const std::string & expected_agent_id,
    const swarm_interfaces::msg::AgentState & message)
  {
    std::optional<CallbackActivity> activity;
    if (instrument_callbacks_) {
      activity.emplace(get_logger(), expected_agent_id, active_agent_callbacks_);
      if (processing_delay_ > std::chrono::milliseconds::zero()) {
        std::this_thread::sleep_for(processing_delay_);
      }
    }

    ProcessResult process_result = state_processor_->process(
      expected_agent_id, message, std::chrono::steady_clock::now());

    if (!process_result.normalization.success()) {
      transform_rejections_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected %s seq=%llu before cache update: %s (%s)",
        expected_agent_id.c_str(), static_cast<unsigned long long>(message.sequence),
        to_string(process_result.normalization.status),
        process_result.normalization.detail.c_str());
      return;
    }

    const UpdateResult & result = *process_result.update;

    if (result.status == UpdateStatus::kRejectedIdentityMismatch) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected identity mismatch on /%s/state: expected '%s', payload says '%s'",
        expected_agent_id.c_str(), expected_agent_id.c_str(), message.agent_id.c_str());
      return;
    }
    if (result.status == UpdateStatus::kRejectedInvalidTimestamp) {
      RCLCPP_WARN(
        get_logger(), "Rejected %s seq=%llu: invalid timestamp",
        expected_agent_id.c_str(), static_cast<unsigned long long>(message.sequence));
      return;
    }
    if (result.status == UpdateStatus::kRejectedTimestamp) {
      RCLCPP_WARN(
        get_logger(), "Rejected %s seq=%llu: duplicate/out-of-order timestamp",
        expected_agent_id.c_str(), static_cast<unsigned long long>(message.sequence));
      return;
    }
    if (result.status == UpdateStatus::kRejectedSequence) {
      RCLCPP_WARN(
        get_logger(), "Rejected %s seq=%llu: duplicate/out-of-order sequence",
        expected_agent_id.c_str(), static_cast<unsigned long long>(message.sequence));
      return;
    }

    if (result.missing_samples > 0) {
      RCLCPP_WARN(
        get_logger(), "Detected %llu missing %s state sample(s)",
        static_cast<unsigned long long>(result.missing_samples),
        expected_agent_id.c_str());
    }

    RCLCPP_DEBUG(
      get_logger(), "Accepted %s seq=%llu",
      expected_agent_id.c_str(), static_cast<unsigned long long>(message.sequence));
  }

  void publish_summary()
  {
    if (instrument_callbacks_) {
      std::ostringstream thread_id;
      thread_id << std::this_thread::get_id();
      const std::string thread_id_text = thread_id.str();
      RCLCPP_INFO(
        get_logger(), "Summary callback thread=%s active_agent_callbacks=%zu",
        thread_id_text.c_str(),
        active_agent_callbacks_.load(std::memory_order_relaxed));
    }

    const auto snapshots = state_processor_->snapshot(
      agent_ids_, std::chrono::steady_clock::now(), stale_timeout_);

    // The cache mutex is already released. Potentially slow formatting and ROS
    // logging operate only on immutable snapshot records.
    std::ostringstream output;
    output << std::fixed << std::setprecision(2) << "Swarm summary:";

    for (const auto & snapshot : snapshots) {
      output << " " << snapshot.agent_id << "=[";
      if (!snapshot.state) {
        output << "NEVER_SEEN]";
        continue;
      }

      output << (snapshot.freshness == Freshness::kFresh ? "FRESH" : "STALE")
             << " seq=" << snapshot.state->message.sequence
             << " frame=" << snapshot.state->message.header.frame_id
             << " p=(" << snapshot.state->message.pose.position.x
             << "," << snapshot.state->message.pose.position.y
             << "," << snapshot.state->message.pose.position.z << ")"
             << " age_ms=" << snapshot.age->count()
             << " health=" << static_cast<unsigned int>(snapshot.state->message.health)
             << "]";
    }

    output << " transform_rejections="
           << transform_rejections_.load(std::memory_order_relaxed);

    const std::string summary = output.str();
    RCLCPP_INFO(get_logger(), "%s", summary.c_str());
  }

  std::vector<std::string> agent_ids_;
  std::string target_frame_{"map"};
  std::chrono::milliseconds stale_timeout_{1500};
  std::chrono::milliseconds processing_delay_{0};
  std::size_t executor_threads_{4};
  bool instrument_callbacks_{false};
  std::atomic<std::size_t> active_agent_callbacks_{0};
  std::atomic<std::uint64_t> transform_rejections_{0};

  // TransformListener owns its standard TF subscriptions. Buffer lookups are
  // safe from the different agent callback groups; no cache lock is held while
  // querying or applying transforms.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<FrameAwareStateProcessor> state_processor_;

  // NodeBase retains only weak ownership of callback groups. Keep strong
  // references for at least as long as their subscriptions/timer can execute.
  std::vector<rclcpp::CallbackGroup::SharedPtr> agent_callback_groups_;
  rclcpp::CallbackGroup::SharedPtr summary_callback_group_;
  std::vector<rclcpp::Subscription<swarm_interfaces::msg::AgentState>::SharedPtr>
    subscriptions_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

}  // namespace swarm_coordinator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<swarm_coordinator::SwarmCoordinatorNode>();
    rclcpp::executors::MultiThreadedExecutor executor{
      rclcpp::ExecutorOptions{}, node->executor_threads()};
    executor.add_node(node);

    RCLCPP_INFO(
      node->get_logger(), "Starting MultiThreadedExecutor with %zu worker threads",
      node->executor_threads());
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("swarm_coordinator"),
      "Coordinator terminated with an exception: %s", exception.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("swarm_coordinator"),
      "Coordinator terminated with an unknown exception");
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
