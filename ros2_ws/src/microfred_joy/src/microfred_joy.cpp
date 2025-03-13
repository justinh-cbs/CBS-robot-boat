#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using std::placeholders::_1;

class TeleopFred : public rclcpp::Node
{
public:
  TeleopFred() : Node("teleop_fred")
  {
    // Declare parameters
    this->declare_parameter("axis_linear", 1);
    this->declare_parameter("axis_angular", 2);
    this->declare_parameter("scale_linear", 1.0);
    this->declare_parameter("scale_angular", 1.0);

    // Get parameters
    linear_ = this->get_parameter("axis_linear").as_int();
    angular_ = this->get_parameter("axis_angular").as_int();
    l_scale_ = this->get_parameter("scale_linear").as_double();
    a_scale_ = this->get_parameter("scale_angular").as_double();

    // Create publishers and subscribers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, 
      std::bind(&TeleopFred::joy_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "TeleopFred node initialized");
    RCLCPP_INFO(this->get_logger(), "Linear axis: %d, Angular axis: %d", linear_, angular_);
    RCLCPP_INFO(this->get_logger(), "Linear scale: %.2f, Angular scale: %.2f", l_scale_, a_scale_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    auto twist = geometry_msgs::msg::Twist();
    
    // Check if we have enough axes
    if (joy->axes.size() > static_cast<size_t>(std::max(linear_, angular_))) {
      twist.angular.z = a_scale_ * joy->axes[angular_];
      twist.linear.x = l_scale_ * joy->axes[linear_];
      
      RCLCPP_DEBUG(this->get_logger(), 
        "Joy input - linear: %.2f, angular: %.2f",
        joy->axes[linear_], joy->axes[angular_]);
        
      vel_pub_->publish(twist);
    } else {
      RCLCPP_WARN(this->get_logger(), 
        "Joystick has %zu axes, but we need access to axis %d",
        joy->axes.size(), std::max(linear_, angular_));
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  int linear_;
  int angular_;
  double l_scale_;
  double a_scale_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopFred>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}