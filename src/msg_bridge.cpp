// Copyright (c) 2024，D-Robotics.
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
// limitations under the License.

# include "rclcpp/rclcpp.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include "ai_msgs/msg/perception_targets.hpp"

class MsgBridgeNode : public rclcpp::Node {
public:
    MsgBridgeNode()
    : Node("tros_apriltag_msg_bridge")
    {
      tag_raw_det_topic_ = this->declare_parameter("tag_raw_det_topic", tag_raw_det_topic_);
      tros_bridged_tag_topic_ = this->declare_parameter("tros_bridged_tag_topic", tros_bridged_tag_topic_);

      RCLCPP_WARN(this->get_logger(),
        "\n      tag_raw_det_topic: %s" \
        "\n tros_bridged_tag_topic: %s",
        tag_raw_det_topic_.c_str(),
        tros_bridged_tag_topic_.c_str()
      );

      tag_detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
          tag_raw_det_topic_, 10, std::bind(&MsgBridgeNode::tag_detection_callback, this, std::placeholders::_1));
      perception_targets_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
          tros_bridged_tag_topic_, 10);
    }
    ~MsgBridgeNode()
    {
    }

private:
  std::string tag_raw_det_topic_ = "raw_apriltag_detections";
  std::string tros_bridged_tag_topic_ = "tros_apriltag_detections";
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detection_sub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr perception_targets_pub_;
  void tag_detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
};

void MsgBridgeNode::tag_detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
  if (!msg || !perception_targets_pub_) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "tag_detection_callback, this msg appears onece.");
  ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg(new ai_msgs::msg::PerceptionTargets());
  targets_msg->header = msg->header;
  for (const auto& detection : msg->detections) {
    ai_msgs::msg::Target target;
    target.set__type("apriltag");
    target.set__track_id(detection.id);
    ai_msgs::msg::Roi roi;
    roi.type = "centre";
    int x_offset = detection.centre.x - 5;
    int y_offset = detection.centre.y - 5;
    if (x_offset < 0) x_offset = 0;
    if (y_offset < 0) y_offset = 0;
    roi.rect.set__x_offset(x_offset);
    roi.rect.set__y_offset(y_offset);
    roi.rect.set__width(10);
    roi.rect.set__height(10);
    target.rois.emplace_back(roi);
    targets_msg->targets.emplace_back(std::move(target));
  }

  perception_targets_pub_->publish(std::move(*targets_msg));
}

int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgBridgeNode>());
  rclcpp::shutdown();
  
  return 0;
}