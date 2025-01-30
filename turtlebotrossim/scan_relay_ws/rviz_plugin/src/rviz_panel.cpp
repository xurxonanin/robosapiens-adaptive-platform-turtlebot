#include <QVBoxLayout>
#include <QStringList>
#include <rviz_common/display_context.hpp>
#include <rviz_plugin/rviz_panel.hpp>
#include <std_msgs/msg/detail/u_int16_multi_array__struct.hpp>

namespace spin_panel
{

  QString toQString(const spin_interfaces::msg::SpinCommand &command)
  {
    return QString("{omega: %1, duration: %2}").arg(command.omega).arg(command.duration);
  }

  QString toQString(const spin_interfaces::msg::SpinPeriodicCommands &periodic_cmds)
  {

    QStringList cmds_list;
    for (const auto &cmd : periodic_cmds.commands)
    {
      cmds_list.append(toQString(cmd));
    }

    return "SpinPeriodicComands {commands: [" + cmds_list.join(", ") + "], period: " + QString::number(periodic_cmds.period, 'f', 2) + "}";
  }

  SpinPanel::SpinPanel(QWidget *parent) : Panel(parent)
  {
    // Create a label and a button, displayed vertically (the V in VBox means
    // vertical)
    const auto layout = new QVBoxLayout(this);
    // Create a button and a label for the button
    label_ = new QTextEdit("[no spin config yet]");
    label_->setReadOnly(true);
    button_ = new QPushButton("TB3 Sim Occlusion!");
    // Add those elements to the GUI layout
    layout->addWidget(label_);
    layout->addWidget(button_);

    // Connect the event of when the button is released to our callback,
    // so pressing the button results in the buttonActivated callback being
    // called.
    QObject::connect(button_, &QPushButton::released, this,
                     &SpinPanel::buttonActivated);
  }

  SpinPanel::~SpinPanel() = default;

  void SpinPanel::onInitialize()
  {
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
    subscription_ = node->create_subscription<spin_interfaces::msg::SpinPeriodicCommands>(
        "/spin_config", 10,
        std::bind(&SpinPanel::topicCallback, this, std::placeholders::_1));
  }

  // When the subscriber gets a message, this callback is triggered,
  // and then we copy its data into the widget's label
  void SpinPanel::topicCallback(const spin_interfaces::msg::SpinPeriodicCommands &msg)
  {
    label_->setText(toQString(msg));
  }

  // When the widget's button is pressed, this callback is triggered,
  // and then we publish a new message on our topic.
  void SpinPanel::buttonActivated()
  {
    auto msg = std_msgs::msg::UInt16MultiArray();
    msg.data = {0, 270};
    publisher_->publish(msg);
  }

} // namespace spin_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(spin_panel::SpinPanel, rviz_common::Panel)
