#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
   MyNode() : Node("my_node")
   {
      subscriber_ = create_subscription<sensor_msgs::msg::Image>(
         "/image", rclcpp::SensorDataQoS(), std::bind(&MyNode::image_callback, this, _1));
      publisher_ = create_publisher<std_msgs::msg::UInt8>("/brightness", rclcpp::SensorDataQoS());
      timer_ = create_wall_timer(5s, std::bind(&MyNode::timer_callback, this));

      RCLCPP_INFO(get_logger(), "Node started!");
   }

private:
   void image_callback(const sensor_msgs::msg::Image::SharedPtr image)
   {
      long long sum = 0;
      for (uint8_t value : image->data)
      {
         sum += value;
      }
      int avg = sum / image->data.size();
      // RCLCPP_INFO(get_logger(), "Brightness: %d", avg);
      
      std_msgs::msg::UInt8 brightness_msg;
      brightness_msg.data = avg;
      publisher_->publish(brightness_msg);
   }

   void timer_callback()
   {
      RCLCPP_INFO(get_logger(), "Timer activate");
   }

   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
   rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}