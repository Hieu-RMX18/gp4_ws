// Copyright 2026 hieu2
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hw_adapter/hw_adapter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hw_adapter::HwAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
