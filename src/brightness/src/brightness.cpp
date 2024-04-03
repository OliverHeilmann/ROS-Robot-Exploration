#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

class MyNode : public rclcpp::Node
{
public:
   MyNode() : Node("my_node")
   {
      subscriber_ = create_subscription<sensor_msgs::msg::Image>(
         "/image", rclcpp::SensorDataQoS(), std::bind(&MyNode::image_callback, this, _1));

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
      RCLCPP_INFO(get_logger(), "Brightness: %d", avg);
   }

   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}