#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/agent_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace agent_simulator
{
namespace
{

constexpr double kTurnRateEpsilon = 1.0e-9;

struct KinematicState
{
  double x_m;
  double y_m;
  double z_m;
  double yaw_rad;
  double velocity_x_mps;
  double velocity_y_mps;
  double velocity_z_mps;
};

[[nodiscard]] geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw_rad)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw_rad);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

[[nodiscard]] bool finite(const double value) noexcept
{
  return std::isfinite(value);
}

}  // namespace

class AgentStatePublisher final : public rclcpp::Node
{
public:
  AgentStatePublisher()
  : Node("agent_state_publisher"),
    start_time_(std::chrono::steady_clock::now())
  {
    agent_id_ = declare_parameter<std::string>("agent_id", "agent_1");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "");
    if (odom_frame_id_.empty()) {
      odom_frame_id_ = agent_id_ + "/odom";
    }
    if (base_frame_id_.empty()) {
      base_frame_id_ = agent_id_ + "/base_link";
    }

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 2.0);
    initial_x_m_ = declare_parameter<double>("initial_x_m", 0.0);
    initial_y_m_ = declare_parameter<double>("initial_y_m", 0.0);
    initial_z_m_ = declare_parameter<double>("initial_z_m", 0.0);
    initial_yaw_rad_ = declare_parameter<double>("initial_yaw_rad", 0.0);
    initial_velocity_x_mps_ = declare_parameter<double>("velocity_x_mps", 0.5);
    initial_velocity_y_mps_ = declare_parameter<double>("velocity_y_mps", 0.0);
    velocity_z_mps_ = declare_parameter<double>("velocity_z_mps", 0.0);
    yaw_rate_radps_ = declare_parameter<double>("yaw_rate_radps", 0.0);

    map_origin_x_m_ = declare_parameter<double>("map_origin_x_m", 0.0);
    map_origin_y_m_ = declare_parameter<double>("map_origin_y_m", 0.0);
    map_origin_z_m_ = declare_parameter<double>("map_origin_z_m", 0.0);
    map_origin_yaw_rad_ = declare_parameter<double>("map_origin_yaw_rad", 0.0);

    simulate_dropout_after_s_ =
      declare_parameter<double>("simulate_dropout_after_s", -1.0);

    validate_configuration(publish_rate_hz);

    // State is a high-rate, freshness-oriented stream. SensorDataQoS uses
    // best-effort, volatile delivery with a small keep-last history.
    publisher_ = create_publisher<swarm_interfaces::msg::AgentState>(
      "state", rclcpp::SensorDataQoS{});

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_map_to_odom_transform();

    const auto period = std::chrono::duration<double>{1.0 / publish_rate_hz};
    timer_ = create_wall_timer(period, std::bind(&AgentStatePublisher::publish_state, this));

    RCLCPP_INFO(
      get_logger(),
      "Agent '%s' publishing %s -> %s state on '%s' at %.2f Hz; "
      "map origin=(%.2f, %.2f, %.2f, yaw=%.2f rad), yaw_rate=%.2f rad/s",
      agent_id_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(),
      publisher_->get_topic_name(), publish_rate_hz,
      map_origin_x_m_, map_origin_y_m_, map_origin_z_m_, map_origin_yaw_rad_,
      yaw_rate_radps_);
  }

private:
  void validate_configuration(const double publish_rate_hz) const
  {
    if (!finite(publish_rate_hz) || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
    }
    if (agent_id_.empty()) {
      throw std::invalid_argument("agent_id must not be empty");
    }
    if (map_frame_id_.empty() || odom_frame_id_.empty() || base_frame_id_.empty()) {
      throw std::invalid_argument("map, odom, and base frame IDs must not be empty");
    }
    if (odom_frame_id_ == base_frame_id_ || map_frame_id_ == base_frame_id_) {
      throw std::invalid_argument("base_frame_id must differ from map and odom frames");
    }

    const bool motion_is_finite =
      finite(initial_x_m_) && finite(initial_y_m_) && finite(initial_z_m_) &&
      finite(initial_yaw_rad_) && finite(initial_velocity_x_mps_) &&
      finite(initial_velocity_y_mps_) && finite(velocity_z_mps_) &&
      finite(yaw_rate_radps_) && finite(map_origin_x_m_) &&
      finite(map_origin_y_m_) && finite(map_origin_z_m_) &&
      finite(map_origin_yaw_rad_) && finite(simulate_dropout_after_s_);
    if (!motion_is_finite) {
      throw std::invalid_argument("motion and frame-origin parameters must be finite");
    }
  }

  void publish_map_to_odom_transform()
  {
    if (map_frame_id_ == odom_frame_id_) {
      RCLCPP_WARN(
        get_logger(),
        "map_frame_id and odom_frame_id are both '%s'; skipping the static transform",
        map_frame_id_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = map_frame_id_;
    transform.child_frame_id = odom_frame_id_;
    transform.transform.translation.x = map_origin_x_m_;
    transform.transform.translation.y = map_origin_y_m_;
    transform.transform.translation.z = map_origin_z_m_;
    transform.transform.rotation = yaw_to_quaternion(map_origin_yaw_rad_);
    static_tf_broadcaster_->sendTransform(transform);
  }

  [[nodiscard]] KinematicState calculate_state(const double elapsed_s) const noexcept
  {
    if (std::abs(yaw_rate_radps_) <= kTurnRateEpsilon) {
      return {
        initial_x_m_ + initial_velocity_x_mps_ * elapsed_s,
        initial_y_m_ + initial_velocity_y_mps_ * elapsed_s,
        initial_z_m_ + velocity_z_mps_ * elapsed_s,
        initial_yaw_rad_,
        initial_velocity_x_mps_,
        initial_velocity_y_mps_,
        velocity_z_mps_};
    }

    const double heading_change = yaw_rate_radps_ * elapsed_s;
    const double sin_heading = std::sin(heading_change);
    const double cos_heading = std::cos(heading_change);

    // Integrate a velocity vector that rotates at a constant yaw rate. This is
    // a deterministic coordinated-turn model, not a vehicle dynamics model.
    const double delta_x =
      (initial_velocity_x_mps_ * sin_heading +
      initial_velocity_y_mps_ * (cos_heading - 1.0)) / yaw_rate_radps_;
    const double delta_y =
      (initial_velocity_x_mps_ * (1.0 - cos_heading) +
      initial_velocity_y_mps_ * sin_heading) / yaw_rate_radps_;

    return {
      initial_x_m_ + delta_x,
      initial_y_m_ + delta_y,
      initial_z_m_ + velocity_z_mps_ * elapsed_s,
      initial_yaw_rad_ + heading_change,
      initial_velocity_x_mps_ * cos_heading - initial_velocity_y_mps_ * sin_heading,
      initial_velocity_x_mps_ * sin_heading + initial_velocity_y_mps_ * cos_heading,
      velocity_z_mps_};
  }

  void publish_state()
  {
    const double elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();

    if (simulate_dropout_after_s_ >= 0.0 && elapsed_s >= simulate_dropout_after_s_) {
      if (!dropout_reported_) {
        RCLCPP_WARN(
          get_logger(),
          "Agent '%s' is simulating state-stream dropout after %.2f s",
          agent_id_.c_str(), simulate_dropout_after_s_);
        dropout_reported_ = true;
      }
      return;
    }

    const KinematicState state = calculate_state(elapsed_s);
    const auto stamp = get_clock()->now();

    swarm_interfaces::msg::AgentState message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_id_;
    message.agent_id = agent_id_;
    message.child_frame_id = base_frame_id_;
    message.sequence = sequence_;
    message.pose.position.x = state.x_m;
    message.pose.position.y = state.y_m;
    message.pose.position.z = state.z_m;
    message.pose.orientation = yaw_to_quaternion(state.yaw_rad);
    message.velocity.linear.x = state.velocity_x_mps;
    message.velocity.linear.y = state.velocity_y_mps;
    message.velocity.linear.z = state.velocity_z_mps;
    message.velocity.angular.z = yaw_rate_radps_;
    message.health = swarm_interfaces::msg::AgentState::HEALTH_NOMINAL;

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header = message.header;
    odom_to_base.child_frame_id = base_frame_id_;
    odom_to_base.transform.translation.x = state.x_m;
    odom_to_base.transform.translation.y = state.y_m;
    odom_to_base.transform.translation.z = state.z_m;
    odom_to_base.transform.rotation = message.pose.orientation;

    // State and dynamic TF use exactly the same source timestamp and kinematic
    // sample, so the graph and AgentState contract cannot describe two poses.
    tf_broadcaster_->sendTransform(odom_to_base);
    publisher_->publish(message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Published %s seq=%llu frame=%s child=%s p=(%.2f, %.2f, %.2f) yaw=%.2f",
      agent_id_.c_str(), static_cast<unsigned long long>(sequence_),
      odom_frame_id_.c_str(), base_frame_id_.c_str(),
      state.x_m, state.y_m, state.z_m, state.yaw_rad);

    ++sequence_;
  }

  std::string agent_id_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double initial_x_m_{0.0};
  double initial_y_m_{0.0};
  double initial_z_m_{0.0};
  double initial_yaw_rad_{0.0};
  double initial_velocity_x_mps_{0.0};
  double initial_velocity_y_mps_{0.0};
  double velocity_z_mps_{0.0};
  double yaw_rate_radps_{0.0};
  double map_origin_x_m_{0.0};
  double map_origin_y_m_{0.0};
  double map_origin_z_m_{0.0};
  double map_origin_yaw_rad_{0.0};
  double simulate_dropout_after_s_{-1.0};
  bool dropout_reported_{false};
  std::uint64_t sequence_{0};
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<swarm_interfaces::msg::AgentState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace agent_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agent_simulator::AgentStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
