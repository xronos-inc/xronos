// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <cstddef>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xronos/sdk.hh"

using namespace std::chrono_literals;

// This is a generic publisher reactor that can be used to publish messages of
// given type on a given topic.
template <class T> class RosPublisher : public xronos::sdk::Reactor {
public:
  RosPublisher(std::string_view name, xronos::sdk::Context context, rclcpp::Node* node, std::string_view topic)
      : Reactor{name, context}
      , publisher_{node->create_publisher<T>(std::string{topic}, 10)} {}

  [[nodiscard]] auto message() noexcept -> auto& { return message_; }

private:
  class PublishReaction : public xronos::sdk::Reaction<RosPublisher<T>> {
    using xronos::sdk::Reaction<RosPublisher<T>>::Reaction;
    xronos::sdk::BaseReaction::Trigger<T> message_trigger{this->self().message_, this->context()};
    void handler() final { this->self().publisher_->publish(*message_trigger.get()); }
  };

  void assemble() final { add_reaction<PublishReaction>("publish"); }

  xronos::sdk::InputPort<T> message_{"message", context()};
  rclcpp::Publisher<T>::SharedPtr publisher_;
};

class Hello : public xronos::sdk::Reactor {
public:
  using xronos::sdk::Reactor::Reactor;

  [[nodiscard]] auto message() noexcept -> auto& { return message_; }

private:
  class HelloReaction : public xronos::sdk::Reaction<Hello> {
    using xronos::sdk::Reaction<Hello>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    PortEffect<std_msgs::msg::String> message_effect{self().message_, context()};
    void handler() final {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(self().count_++);
      std::cerr << "Publishing: " << message.data.c_str() << '\n';
      message_effect.set(message);
    }
  };

  void assemble() final { add_reaction<HelloReaction>("hello"); }

  xronos::sdk::OutputPort<std_msgs::msg::String> message_{"message", context()};
  xronos::sdk::PeriodicTimer timer_{"timer", context(), 500ms};

  size_t count_{0};

  rclcpp::Node* node_;
};

// This ROS node hosts the xronos program.
class NodeWrapper : public rclcpp::Node {
public:
  NodeWrapper()
      : Node("node_wrapper") {
    env_.connect(hello_.message(), publisher_.message());
    xronos_ececution_ = std::thread([this]() { env_.execute(); });
  }

  ~NodeWrapper() { xronos_ececution_.join(); }

private:
  xronos::sdk::Environment env_{};
  Hello hello_{"hello", env_.context()};
  RosPublisher<std_msgs::msg::String> publisher_{"publisher", env_.context(), this, "topic"};

  std::thread xronos_ececution_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeWrapper>());
  rclcpp::shutdown();
  return 0;
}
