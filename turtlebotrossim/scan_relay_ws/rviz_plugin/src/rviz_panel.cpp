#include <QVBoxLayout>
#include <QStringList>
#include <rviz_common/display_context.hpp>
#include <rviz_plugin/rviz_panel.hpp>
#include <std_msgs/msg/detail/u_int16_multi_array__struct.hpp>
#include <rviz_common/logging.hpp>

namespace spin_panel
{
  std::string vec_to_string(const std::vector<uint16_t> &vec)
  {
    std::string str = "[";
    for (const auto &val : vec)
    {
      str += std::to_string(val) + ", ";
    }
    str += "]";
    return str;
  }

  // Function to create a group of radio buttons
  template <typename Iterable>
  QGroupBox *createRadioButtonGroup(const QString &title, const Iterable &options, QRadioButton *&selectedButton)
  {
    QGroupBox *groupBox = new QGroupBox(title);
    QGridLayout *layout = new QGridLayout(groupBox);

    int row = 0, col = 0;
    for (const auto &option : options)
    {
      QRadioButton *button = new QRadioButton(option);
      layout->addWidget(button, row, col);
      if (option == BOT_NAMES.at(bot_variant::TB3_Real).data() || option == BOT_NAMES.at(bot_variant::TB4_Sim).data())
        button->setEnabled(false);
      if (!selectedButton)
        selectedButton = button; // Default to first button
      col = (col + 1) % 2;       // Switch columns
      if (col == 0)
        row++; // Move to next row after two items
    }

    groupBox->setLayout(layout);
    return groupBox;
  }

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
    button_ = new QPushButton("Send mocked occlusion");
    bot_variant_selected_ = nullptr;
    std::vector<const char *> bot_names{};
    std::transform(BOTS.begin(), BOTS.end(), std::back_inserter(bot_names), [](auto bot)
                   { return BOT_NAMES.at(bot).data(); });
    bot_variants_ = createRadioButtonGroup("Select robot variant", bot_names, bot_variant_selected_);
    region_variant_selected_ = nullptr;
    std::vector<const char *> occlusion_dirs{};
    std::transform(DIRECTIONS.begin(), DIRECTIONS.end(), std::back_inserter(occlusion_dirs), [](auto occ)
                   { return DIRECTION_NAMES.at(occ).data(); });
    region_variants_ = createRadioButtonGroup("Select region to cover", occlusion_dirs, region_variant_selected_);

    // Add those elements to the GUI layout
    layout->addWidget(bot_variants_);
    layout->addWidget(region_variants_);
    layout->addWidget(button_);
    layout->addWidget(label_);

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
    auto buttons = bot_variants_->findChildren<QRadioButton *>();
    bot_variant bot_variant = BOTS[0];
    occlusion_direction occlusion_direction = DIRECTIONS[0];

    for (int i = 0; i < buttons.size(); ++i)
    {
      if (buttons[i]->isChecked())
      {
        bot_variant = BOTS[i];
        break;
      }
    }

    buttons = region_variants_->findChildren<QRadioButton *>();
    for (int i = 0; i < buttons.size(); ++i)
    {
      if (buttons[i]->isChecked())
      {
        occlusion_direction = DIRECTIONS[i];
        break;
      }
    }

    auto arr = make_occlusions_map().at(BotOcc{bot_variant, occlusion_direction});
    std::sort(arr.begin(), arr.end());
    for (const auto &elem : arr)
    {
      // The (0,0) is an artifact from rotation calculations
      if (elem != DONT_CARE && elem != std::make_pair(0_u, 0_u))
      {
        msg.data.emplace_back(elem.first);
        msg.data.emplace_back(elem.second);
      }
    }

    RVIZ_COMMON_LOG_INFO_STREAM("Setting following as non-occluded: " << vec_to_string(msg.data));
    publisher_->publish(msg);
  }

} // namespace spin_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(spin_panel::SpinPanel, rviz_common::Panel)
