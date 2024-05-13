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
   * @brief Structure to hold the detection information from the YoloV5 model outputs.
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
   * @brief Construct a new Detection object
   * 
   */
  Detection();

private:

  /**
   * @brief 
   * 
   * @tparam T 
   * @param name 
   * @param default_value 
   */
  template<typename T>
  T init_parameter(const std::string& name, T default_value) {
      declare_parameter(name, default_value);
      T value = get_parameter(name).get_value<T>();
      RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), to_string(value).c_str());
      return value;
  }
  
  /**
   * @brief Template function to convert any type to string
   * 
   * @tparam T 
   * @param value 
   * @return std::string 
   */
  template<typename T>
  std::string to_string(const T& value) {
      return std::to_string(value);
  }

  /**
   * @brief Overload for string type
   * 
   * @param value 
   * @return std::string 
   */
  std::string to_string(const std::string& value) { return value; }

  /**
   * @brief Overload for bool type
   * 
   * @param value 
   * @return std::string 
   */
  std::string to_string(const bool& value) { return value ? "true" : "false"; }

  /**
   * @brief 
   * 
   * @param msg 
   */
  void _imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 
   * 
   * @param frame 
   * @param obj 
   */
  void _initDetection(cv::Mat frame, cv::Rect obj);

  /**
   * @brief 
   * 
   * @param path 
   * @return std::vector<std::string> 
   */
  std::vector<std::string> _loadClassList(const std::string &path);

  /**
   * @brief 
   * 
   * @param net 
   * @param is_cuda 
   * @param path 
   */
  void _loadNet(cv::dnn::Net &net, bool is_cuda, const std::string &path);

  /**
   * @brief 
   * 
   * @param source 
   * @return cv::Mat 
   */
  cv::Mat _format_to_yolo(const cv::Mat &source);

  /**
   * @brief 
   * 
   * @param frame 
   * @param output 
   */
  void _detect(cv::Mat &frame, std::vector<Match> &output);

  /**
   * @brief 
   * 
   * @param frame 
   * @param match 
   */
  void _drawBox(cv::Mat &frame, Match &match);

  /**
   * @brief 
   * 
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;

  /**
   * @brief 
   * 
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _detection_pub;

  /**
   * @brief 
   * 
   */
  bool _is_detection_initialised;

  /**
   * @brief 
   * 
   */
  std::string _classes_path;

  /**
   * @brief 
   * 
   */
  std::string _model_path;

  /**
   * @brief 
   * 
   */
  bool _is_cuda;

  int _input_width;

  int _input_height;

  float _score_threshold;

  float _confidence_threshold;

  float _nms_threshold;

  /**
   * @brief 
   * 
   */
  std::vector<std::string> _classes;

  /**
   * @brief Private
   * 
   */
  cv::dnn::Net _net;

};