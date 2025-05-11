// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <future>
#include <iostream>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xronos/sdk.hh"

// This is a generic subscriber reactor that can be used to receive messages of
// given type on a give topic.
template <class T> class RosSubscriber : public xronos::sdk::Reactor {
public:
  RosSubscriber(std::string_view name, xronos::sdk::Context context, rclcpp::Node* node, std::string_view topic)
      : Reactor{name, context}
      , subscription_{node->create_subscription<T>(
            std::string{topic}, 10, [this](typename T::ConstSharedPtr msg) { message_received_.trigger(*msg); })} {}

  [[nodiscard]] auto message() noexcept -> auto& { return message_; }

private:
  class MessageReceivedReaction : public xronos::sdk::Reaction<RosSubscriber<T>> {
    using xronos::sdk::Reaction<RosSubscriber<T>>::Reaction;
    xronos::sdk::BaseReaction::Trigger<T> message_received_trigger{this->self().message_received_, this->context()};
    xronos::sdk::BaseReaction::PortEffect<T> message_effect{this->self().message_, this->context()};
    void handler() final { message_effect.set(message_received_trigger.get()); }
  };

  void assemble() final { add_reaction<MessageReceivedReaction>("on_message_received"); }

  xronos::sdk::OutputPort<T> message_{"message", context()};
  xronos::sdk::PhysicalEvent<T> message_received_{"message_received", context()};

  rclcpp::Subscription<T>::SharedPtr subscription_;
};

class Printer : public xronos::sdk::Reactor {
public:
  using xronos::sdk::Reactor::Reactor;

  [[nodiscard]] auto message() noexcept -> auto& { return message_; }

private:
  class PrintReaction : public xronos::sdk::Reaction<Printer> {
    using xronos::sdk::Reaction<Printer>::Reaction;
    Trigger<std_msgs::msg::String> message_trigger{self().message_, context()};
    void handler() final { std::cerr << "I heard: " << message_trigger.get()->data.c_str() << '\n'; }
  };

  void assemble() final { add_reaction<PrintReaction>("print"); }

  xronos::sdk::InputPort<std_msgs::msg::String> message_{"message", context()};
};

// This ROS node hosts the xronos program.
class NodeWrapper : public rclcpp::Node {
public:
  NodeWrapper()
      : Node("node_wrapper") {
    env_.connect(subscriber_.message(), printer_.message());
    xronos_ececution_ = std::thread([this]() { env_.execute(); });
  }

  ~NodeWrapper() { xronos_ececution_.join(); }

private:
  xronos::sdk::Environment env_{};
  RosSubscriber<std_msgs::msg::String> subscriber_{"subscriber", env_.context(), this, "topic"};
  Printer printer_{"printer", env_.context()};

  std::thread xronos_ececution_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeWrapper>());
  rclcpp::shutdown();
  return 0;
}
