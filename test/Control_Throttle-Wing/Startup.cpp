#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/msg/hw_api_attitude_rate_cmd.hpp>

using namespace std::chrono_literals;

enum class State {
  STREAM_HEARTBEAT,
  REQUEST_ARM,
  REQUEST_OFFBOARD,
  MANUAL_THROTTLE_CONTROL
};

class OffboardAttitudeRateNode : public rclcpp::Node {
public:
  OffboardAttitudeRateNode() : Node("offboard_attitude_rate_node") {
    // Publishers & Subscribers
    pub_cmd_ = this->create_publisher<mrs_msgs::msg::HwApiAttitudeRateCmd>(
        "/uav1/hw_api/attitude_rate_cmd", 10);

    sub_throttle_ = this->create_subscription<std_msgs::msg::Float32>(
        "/uav1/set_throttle", 10,
        std::bind(&OffboardAttitudeRateNode::throttle_callback, this, std::placeholders::_1));

    // Services
    client_arm_ = this->create_client<std_srvs::srv::SetBool>("/uav1/hw_api/arming");
    client_offboard_ = this->create_client<std_srvs::srv::Trigger>("/uav1/hw_api/offboard");

    // Timers
    timer_cmd_ = this->create_wall_timer(
        20ms, std::bind(&OffboardAttitudeRateNode::publish_cmd, this)); // 50 Hz

    timer_seq_ = this->create_wall_timer(
        1s, std::bind(&OffboardAttitudeRateNode::sequence_step, this)); // 1 Hz
  }

private:
  void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // Clamp throttle between 0.0 (0%) and 1.0 (100%) for safety
    double new_throttle = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
    
    if (state_ == State::MANUAL_THROTTLE_CONTROL) {
      RCLCPP_INFO(this->get_logger(), "Throttle updated: %.2f", new_throttle);
    }
    target_thrust_ = new_throttle;
  }

  void publish_cmd() {
    mrs_msgs::msg::HwApiAttitudeRateCmd msg;
    msg.stamp = this->now();
    msg.body_rate.x = 0.0;
    msg.body_rate.y = 0.0;
    msg.body_rate.z = 0.0;
    msg.throttle = target_thrust_;

    pub_cmd_->publish(msg);
  }

  void sequence_step() {
    switch (state_) {
      case State::STREAM_HEARTBEAT: {
        ticks_++;
        if (ticks_ >= 2) {
          state_ = State::REQUEST_ARM;
        }
        break;
      }

      case State::REQUEST_ARM: {
        if (!client_arm_->service_is_ready()) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for arming service...");
          return;
        }

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        
        client_arm_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
          if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Arming successful!");
            this->state_ = State::REQUEST_OFFBOARD;
          } else {
            RCLCPP_WARN(this->get_logger(), "Arming rejected, retrying...");
          }
        });
        break;
      }

      case State::REQUEST_OFFBOARD: {
        if (!client_offboard_->service_is_ready()) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for offboard service...");
          return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

        client_offboard_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Offboard mode engaged! Ready for topic throttle input.");
            this->state_ = State::MANUAL_THROTTLE_CONTROL;
          } else {
            RCLCPP_WARN(this->get_logger(), "Offboard request rejected, retrying...");
          }
        });
        break;
      }

      case State::MANUAL_THROTTLE_CONTROL: {
        RCLCPP_INFO_ONCE(this->get_logger(), "Listening on topic /uav1/set_throttle");
        break;
      }
    }
  }

  rclcpp::Publisher<mrs_msgs::msg::HwApiAttitudeRateCmd>::SharedPtr pub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_arm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_offboard_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;
  rclcpp::TimerBase::SharedPtr timer_seq_;

  State state_ = State::STREAM_HEARTBEAT;
  int ticks_ = 0;
  double target_thrust_ = 0.05; // Idle throttle baseline
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardAttitudeRateNode>());
  rclcpp::shutdown();
  return 0;
}