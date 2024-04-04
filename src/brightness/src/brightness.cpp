// Build with: 
//    colcon build --packages-select brightness --symlink-install

// Run with: 
//    ros2 launch brightness brightness.yaml 

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
   MyNode() : Node("my_node")
   {
      // gets the timer_period_s parameter from the yaml file
      declare_parameter("timer_period_s", 5);
      auto timer_period_s = std::chrono::seconds(get_parameter("timer_period_s").as_int());
      RCLCPP_INFO(get_logger(), "Timer period: %ld", timer_period_s.count());

      // subscribes to the /image topic and calls the image_callback method which is defined below
      subscriber_ = create_subscription<sensor_msgs::msg::Image>(
         "/image", rclcpp::SensorDataQoS(), std::bind(&MyNode::image_callback, this, _1));
      
      // publishes the brightness value to the /brightness topic. This is to publish a message of type std_msgs::msg::UInt8
      publisher_ = create_publisher<std_msgs::msg::UInt8>("/brightness", rclcpp::SensorDataQoS());
      
      // creates a timer that calls the timer_callback method every timer_period_s seconds
      timer_ = create_wall_timer(timer_period_s, std::bind(&MyNode::timer_callback, this));

      // creates a client to call the /save service. This is called in the timer_callback method which then calls the service
      // in the image_saver node to save the image (i.e. not in this node file)
      client_ = create_client<std_srvs::srv::Empty>("/save");

      // creates a service to call the /image_counter service. This is triggered when the service is called by the client (i,e
      // not in this node file - usage could be like 'ros2 service call /image_counter std_srvs/srv/Trigger {}' for example)
      server_ = create_service<std_srvs::srv::Trigger>(
         "/image_counter", std::bind(&MyNode::counter_callback, this, _1, _2));


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

      if (!client_->wait_for_service(1s))
      {
         RCLCPP_ERROR(get_logger(), "Failed to connect to the image save service");
         return;
      }

      saved_imgs_++;
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = client_->async_send_request(request);
   }

   void counter_callback(const std_srvs::srv::Trigger::Request::SharedPtr req,
      const std_srvs::srv::Trigger::Response::SharedPtr res)
   {
      res->success = 1;
      res->message = "Saved images: " + std::to_string(saved_imgs_);
   }

   uint saved_imgs_ = 0;
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
   rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MyNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}