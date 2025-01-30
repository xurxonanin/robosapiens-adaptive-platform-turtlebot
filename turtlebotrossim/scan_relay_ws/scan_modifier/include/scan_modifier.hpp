#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "math_help.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace scan
{

  class ScanModifierNode : public rclcpp::Node
  {
  public:
    using ranges_t = sensor_msgs::msg::LaserScan::_ranges_type;

    ScanModifierNode();

  private:
    void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg);
    void config_sub_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr config);

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr config_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    uint16_t lidar_filt_upper_; // Initially configured by parameter - otherwise runtime error
    uint16_t lidar_filt_lower_ = 0;
  };

} // namespace scan