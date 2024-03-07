// Copyright 2024 Team Reboot.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <yolov8_msgs/msg/detection_array.hpp>

class YoloBallTracking : public rclcpp::Node
{
public:
  YoloBallTracking() : Node("yolo_ball_tracking")
  {
    width_ = declare_parameter<double>("width");
    height_ = declare_parameter<double>("height");
    target_class_name_ = declare_parameter<std::string>("target_class_name");

    yolo_detection_array_subscriber_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
      "/yolo/tracking", 10,
      std::bind(&YoloBallTracking::callback_yolo_detection, this, std::placeholders::_1));
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&YoloBallTracking::callback_joint_state, this, std::placeholders::_1));
    target_joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("target_joint_states", 10);
  }
  ~YoloBallTracking() = default;

  void callback_yolo_detection(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) return;
    if (current_joint_state_.name.empty()) return;

    sensor_msgs::msg::JointState target_joint_state = current_joint_state_;
    for (auto & detection : msg->detections) {
      if (detection.class_name == "sports ball") {
        horizontal_ -= (0.1 * ((detection.bbox.center.position.x / width_) - 0.5));
        vertical_ -= (0.1 * ((detection.bbox.center.position.y / height_) - 0.5));

        // head_pan
        target_joint_state.position[18] = horizontal_;
        // head_tilt
        target_joint_state.position[19] = vertical_;

        break;
      }
    }

    target_joint_state_publisher_->publish(target_joint_state);
  }

  void callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state_ = *msg;
  }

private:
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr
    yolo_detection_array_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_publisher_;

  sensor_msgs::msg::JointState current_joint_state_;

  double width_;
  double height_;
  double vertical_{0.0};
  double horizontal_{0.0};
  std::string target_class_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloBallTracking>());
  rclcpp::shutdown();
  return 0;
}
