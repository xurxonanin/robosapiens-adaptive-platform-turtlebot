#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_plugin/rviz_panel.hpp>
#include <std_msgs/msg/detail/u_int16_multi_array__struct.hpp>

namespace rviz_panel_tutorial {

DemoPanel::DemoPanel(QWidget *parent) : Panel(parent) {
  // Create a label and a button, displayed vertically (the V in VBox means
  // vertical)
  const auto layout = new QVBoxLayout(this);
  // Create a button and a label for the button
  label_ = new QLabel("[no spin config yet]");
  button_ = new QPushButton("TB3 Sim Occlusion!");
  // Add those elements to the GUI layout
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the buttonActivated callback being
  // called.
  QObject::connect(button_, &QPushButton::released, this,
                   &DemoPanel::buttonActivated);
}

DemoPanel::~DemoPanel() = default;

void DemoPanel::onInitialize() {
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making
  // subscriptions/publishers (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  // Create a String publisher for the output
  publisher_ = node->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "/scan_config", 10);

  // Create a String subscription and bind it to the topicCallback inside this
  // class.
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/input", 10,
      std::bind(&DemoPanel::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void DemoPanel::topicCallback(const std_msgs::msg::String &msg) {
  label_->setText(QString(msg.data.c_str()));
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void DemoPanel::buttonActivated() {
  auto msg = std_msgs::msg::UInt16MultiArray();
  msg.data = {0, 270};
  publisher_->publish(msg);
}

} // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::DemoPanel, rviz_common::Panel)
