#pragma once

#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <QTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include "spin_interfaces/msg/spin_periodic_commands.hpp"
#include <array>
#include <utility>

namespace spin_panel
{
  constexpr std::uint16_t operator"" _u(unsigned long long value)
  {
    return static_cast<std::uint16_t>(value);
  }
  class SpinPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    static constexpr std::array BOTS = {"TurtleBot3 sim", "TurtleBot3 real", "TurtleBot4 sim", "TurtleBot4 real"};
    static constexpr std::array LIDAR_SAMPLES = {360_u, 0_u, 0_u, 0_u};
    static constexpr std::array REGIONS_TEXT = {"Northwest", "Northeast", "Southwest", "Southeast"};
    static constexpr std::array REGIONS_PAIRS = {std::make_pair(0., 0.), std::make_pair(0., 0.75),
                                                 std::make_pair(0., 0.), std::make_pair(0., 0.)};

    explicit SpinPanel(QWidget *parent = 0);
    ~SpinPanel() override;

    void onInitialize() override;

  protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>
        node_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<spin_interfaces::msg::SpinPeriodicCommands>::SharedPtr subscription_;

    void topicCallback(const spin_interfaces::msg::SpinPeriodicCommands &msg);

    QTextEdit *label_;
    QPushButton *button_;
    QRadioButton *bot_variant_selected_;
    QGroupBox *bot_variants_;
    QRadioButton *region_variant_selected_;
    QGroupBox *region_variants_;

  private Q_SLOTS:
    void buttonActivated();
  };
} // namespace spin_panel
