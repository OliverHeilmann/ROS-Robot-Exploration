#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/cvconfig.h>   // if using GPU acceleration, include this header

class Detection : public rclcpp::Node
{
public:
  /**
   * @brief Structure to hold the detection information from the YoloV5 model outputs. Made public for easy access from 
   * other classes (though this is not essential as we use ROS2 topics to communicate detections between nodes).
   * 
   */
  struct Match
  {
    int class_id;
    float confidence;
    cv::Rect box;
    cv::Scalar color;
  };

  /**
   * @brief Construct a new Detection object for the ROS2 node. This node is responsible for detecting objects in images
   * and publishing the results as image messages.
   * 
   */
  Detection();

private:
  void _imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  void _initDetection();
  std::vector<std::string> _loadClassList(const std::string &path);
  void _loadNet(cv::dnn::Net &net, bool is_cuda, const std::string &path);

  cv::Mat _formatToYolo(const cv::Mat &source);
  void _detect(cv::Mat &frame, std::vector<Match> &output);
  void _drawBox(cv::Mat &frame, Match &match);

  /**
   * @brief Helper function to initialise the parameters for the detection application. Template function to handle different types.
   * 
   * @tparam T is the type of the parameter.
   * @param[in] name is the name of the parameter.
   * @param[in] default_value is the default value of the parameter.
   * @return T is the value of the parameter of type T.
   */
  template<typename T>
  T _initParameter(const std::string& name, T default_value) {
      declare_parameter(name, default_value);
      T value = get_parameter(name).get_value<T>();
      RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), _to_string(value).c_str());
      return value;
  }

  /**
   * @brief Helper function to convert different types to string.
   * 
   * @tparam T is the type of the value.
   * @param[in] value is the value to convert to string.
   * @return std::string is the string representation of the value.
   */
  template<typename T>
  std::string _to_string(const T& value) {
      return std::to_string(value);
  }
  std::string _to_string(const std::string& value) { return value; }
  std::string _to_string(const bool& value) { return value ? "true" : "false"; }

  /**
   * @brief ROS2 subscriber and publisher for image messages.
   * 
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _detection_pub;

  /**
   * @brief Load the classes and the model for the detection application.
   * 
   */
  bool _is_detection_initialised;
  std::string _classes_path;
  std::vector<std::string> _classes;
  std::string _model_path;
  cv::dnn::Net _net;

  /**
   * @brief Detection parameters.
   * 
   */
  bool _is_cuda;
  int _input_width;
  int _input_height;
  float _score_threshold;
  float _confidence_threshold;
  float _nms_threshold;
};