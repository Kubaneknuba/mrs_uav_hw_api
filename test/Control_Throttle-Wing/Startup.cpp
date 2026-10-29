#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
#include <mrs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>

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
    // Create a reentrant callback group to allow concurrent execution
    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    // Publishers & Subscribers
    pub_cmd_ = this->create_publisher<mrs_msgs::msg::HwApiAttitudeRateCmd>(
        "/uav1/hw_api/attitude_rate_cmd", 10);

    sub_throttle_ = this->create_subscription<std_msgs::msg::Float32>(
        "/uav1/set_throttle", 10,
        std::bind(&OffboardAttitudeRateNode::throttle_callback, this, std::placeholders::_1));

    pub_ActuatorControl_ = this->create_publisher<mrs_msgs::msg::HwApiActuatorCmd>(
        "/uav1/hw_api/actuator_cmd", 10);

    sub_ActuatorControl = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/uav1/set_actuator_cmd", 10,
        std::bind(&OffboardAttitudeRateNode::actuator_callback, this, std::placeholders::_1));

    sub_arm_state_ = this->create_subscription<mrs_msgs::msg::HwApiStatus>(
        "/uav1/hw_api/status", 10,
        std::bind(&OffboardAttitudeRateNode::arm_state_callback, this, std::placeholders::_1));

    pub_arm_state_ = this->create_publisher<std_msgs::msg::Bool>(
        "/uav1/arm_disarm_cmd", 10);

    // Services
    client_arm_ = this->create_client<std_srvs::srv::SetBool>("/uav1/hw_api/arming");
    client_offboard_ = this->create_client<std_srvs::srv::Trigger>("/uav1/hw_api/offboard");

    service_arm_ = this->create_service<std_srvs::srv::SetBool>(
        "/uav1/arming", 
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          auto result = this->handle_arm_request(request->data);
          response->success = result.first;
          response->message = result.second;
        },
        rmw_qos_profile_services_default,
        callback_group_);  // Pass callback group here directly

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

  void actuator_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 6) {
      RCLCPP_WARN(this->get_logger(), "Actuator command array must have 6 elements.");
      return;
    }

    mrs_msgs::msg::HwApiActuatorCmd actuator_msg;
    actuator_msg.stamp = this->now();

    actuator_msg.motors.resize(6);
    for (size_t i = 0; i < 6; ++i) {
      actuator_msg.motors[i] = static_cast<float>(msg->data[i]);
    }

    pub_ActuatorControl_->publish(actuator_msg);
  }

  void arm_state_callback(const mrs_msgs::msg::HwApiStatus::SharedPtr msg) {
    if (msg->armed) {
      RCLCPP_INFO_ONCE(this->get_logger(), "UAV is armed.");
    } else {
      RCLCPP_INFO_ONCE(this->get_logger(), "UAV is disarmed.");
    }
    std_msgs::msg::Bool arm_state_msg;
    arm_state_msg.data = msg->armed;
    pub_arm_state_->publish(arm_state_msg);
  }

  std::pair<bool, std::string> handle_arm_request(bool arm) {
    if (!client_arm_->service_is_ready()) {
      return {false, "Arming service not ready"};
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = arm;

    // Create a promise/future bridge for the async request
    auto promise = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto future = promise->get_future();

    client_arm_->async_send_request(req, [promise, arm](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture client_future) {
      auto res = client_future.get();
      if (res && res->success) {
        promise->set_value({true, arm ? "UAV armed successfully" : "UAV disarmed successfully"});
      } else {
        promise->set_value({false, arm ? "Arming rejected" : "Disarming rejected"});
      }
    });

    // Standard C++ wait_for (safe with MultiThreadedExecutor, no ROS 2 spinning involved)
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      return future.get();
    } else {
      return {false, "Arming service call timed out"};
    }
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
        std_msgs::msg::Bool arm_msg;
        arm_msg.data = true;
        pub_arm_state_->publish(arm_msg); // Publish arm command for visualization
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
  rclcpp::Publisher<mrs_msgs::msg::HwApiActuatorCmd>::SharedPtr pub_ActuatorControl_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ActuatorControl;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arm_state_;
  rclcpp::Subscription<mrs_msgs::msg::HwApiStatus>::SharedPtr sub_arm_state_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_arm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_offboard_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_arm_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;
  rclcpp::TimerBase::SharedPtr timer_seq_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  State state_ = State::STREAM_HEARTBEAT;
  int ticks_ = 0;
  double target_thrust_ = 0.05; // Idle throttle baseline
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardAttitudeRateNode>();

  // MultiThreadedExecutor allows blocking calls inside service callbacks 
  // without deadlocking or throwing executor exceptions.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}