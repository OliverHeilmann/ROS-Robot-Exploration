#include "rosbot/detection.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <fstream>
#include <filesystem>

using namespace std::placeholders;

Detection::Detection() : Node("Detection"), _is_detection_initialised(false)
{
    // print current path
    RCLCPP_INFO(get_logger(), "Current path: %s", std::filesystem::current_path().c_str());

    _classes_path = init_parameter<std::string>("path_to_classes", "src/rosbot/models/classes.txt");
    _model_path = init_parameter<std::string>("path_to_weights", "src/rosbot/models/yolov5s.onnx");
    _is_cuda = init_parameter<bool>("is_cuda", false);
    _input_width = init_parameter<int>("_input_width", 640);
    _input_height = init_parameter<int>("_input_height", 640);
    _score_threshold = init_parameter<float>("score_threshold", 0.2);
    _confidence_threshold = init_parameter<float>("confidence_threshold", 0.5);
    _nms_threshold = init_parameter<float>("nms_threshold", 0.4);

    // Subscribers
    _img_sub = create_subscription<sensor_msgs::msg::Image>("/image", rclcpp::SensorDataQoS(), bind(&Detection::_imageCallback, this, _1));

    // Publishers
    _detection_pub = create_publisher<sensor_msgs::msg::Image>("/detections", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Detection Node started!");
}

void Detection::_imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Image received, Detection running!");

    // Convert the image message to an OpenCV Mat
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_image->image;
    cv::Rect obj;

    if (!_is_detection_initialised)
    {
        _initDetection(frame, obj);
    }

    // MAIN DETECTION LOGIC GOES HERE //
    std::vector<Match> matches;
    _detect(frame, matches);

    for (auto &match : matches)
    {
        _drawBox(frame, match);
    }

    cv_image->image = frame;
    auto img_msg = cv_image->toImageMsg();
    _detection_pub->publish(*img_msg);
}

void Detection::_initDetection(cv::Mat frame, cv::Rect obj)
{
    // load classes
    _classes = _loadClassList(_classes_path);

    // load net
    this->_loadNet(_net, _is_cuda, _model_path);

    _is_detection_initialised = true;
}

std::vector<std::string> Detection::_loadClassList(const std::string &path = "models/classes.txt")
{
    // check if path exists
    if (!std::filesystem::exists(path))
    {
        RCLCPP_WARN(get_logger(), "Class list path %s does not exist", path.c_str());
        return {};
    }

    std::vector<std::string> classes;
    std::ifstream ifs(path.c_str());
    std::string line;
    while (std::getline(ifs, line))
    {
        classes.push_back(line);
    }
    return classes;
}

void Detection::_loadNet(cv::dnn::Net &net, bool is_cuda, const std::string &path = "models/yolov5s.onnx")
{
    // check if path exists
    if (!std::filesystem::exists(path))
    {
        RCLCPP_WARN(get_logger(), "Model path %s does not exist", path.c_str());
        return;
    }

    auto result = cv::dnn::readNet(path.c_str());
    if (is_cuda)
    {
        RCLCPP_INFO(get_logger(), "Running on GPU");
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Running on CPU");
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}

cv::Mat Detection::_format_to_yolo(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void Detection::_detect(cv::Mat &frame, std::vector<Match> &output)
{
    cv::Mat blob;

    auto input_image = _format_to_yolo(frame);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(_input_width, _input_height), cv::Scalar(), true, false);
    _net.setInput(blob);
    std::vector<cv::Mat> outputs;
    _net.forward(outputs, _net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / _input_width;
    float y_factor = input_image.rows / _input_height;
    
    float *data = (float *)outputs[0].data;

    // calculate the size of the data array
    auto shape = outputs[0].size;
    const int rows = shape[1]; // This might be specific to your model's configuration
    const int dimensions = shape[2]; // Number of classes + 5 (x, y, w, h, confidence)
    const int classes = dimensions - 5; // Number of classes for your model
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        float confidence = data[4];
        if (confidence >= _confidence_threshold) {

            float * classes_scores = data + 5;
            cv::Mat scores(1, classes, CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > _score_threshold) {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, _score_threshold, _nms_threshold, nms_result);
    for (int i = 0; i < nms_result.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "Detected %s with confidence %f", _classes[class_ids[i]].c_str(), confidences[i]);
        int idx = nms_result[i];
        Detection::Match result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}

cv::Point Detection::_getCentre(Match &match)
{
    return cv::Point(match.box.x + match.box.width / 2, match.box.y + match.box.height / 2);
}

void Detection::_drawBox(cv::Mat &frame, Match &match)
{
    cv::rectangle(frame, match.box, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, _classes[match.class_id], cv::Point(match.box.x, match.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Detection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}