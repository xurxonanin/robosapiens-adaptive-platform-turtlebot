#pragma once

#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace rviz_panel_tutorial {
class DemoPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit DemoPanel(QWidget *parent = 0);
  ~DemoPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>
      node_ptr_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void topicCallback(const std_msgs::msg::String &msg);

  QLabel *label_;
  QPushButton *button_;

private Q_SLOTS:
  void buttonActivated();
};
} // namespace rviz_panel_tutorial
