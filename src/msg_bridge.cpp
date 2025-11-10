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
# include "sensor_msgs/msg/compressed_image.hpp"
# include "std_msgs/msg/bool.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include "ai_msgs/msg/perception_targets.hpp"

class MsgBridgeNode : public rclcpp::Node {
public:
    MsgBridgeNode()
    : Node("tros_apriltag_msg_bridge")
    {
      bridge_type_ = this->declare_parameter("bridge_type", bridge_type_);
      tag_raw_det_topic_ = this->declare_parameter("tag_raw_det_topic", tag_raw_det_topic_);
      tros_bridged_tag_topic_ = this->declare_parameter("tros_bridged_tag_topic", tros_bridged_tag_topic_);
      raw_compressd_img_topic_ = this->declare_parameter("raw_compressd_img_topic", raw_compressd_img_topic_);
      tros_bridged_compressd_img_topic_ = this->declare_parameter("tros_bridged_compressd_img_topic", tros_bridged_compressd_img_topic_);
      controller_tick_topic_ = this->declare_parameter("controller_tick_topic", controller_tick_topic_);
      tick_timeout_sec_ = this->declare_parameter("tick_timeout_sec", tick_timeout_sec_);

      RCLCPP_WARN(this->get_logger(),
        "\n            bridge_type: '%s' [perc/compressd_img/raw_img]"
        "\n      tag_raw_det_topic: '%s'" \
        "\n tros_bridged_tag_topic: '%s'" \
        "\nraw_compressd_img_topic: '%s'"
        "\n tros_bridged_compressd_img_topic: '%s'" \
        "\n  controller_tick_topic: '%s'" \
        "\n       tick_timeout_sec: %.2f",
        bridge_type_.c_str(),
        tag_raw_det_topic_.c_str(),
        tros_bridged_tag_topic_.c_str(),
        raw_compressd_img_topic_.c_str(),
        tros_bridged_compressd_img_topic_.c_str(),
        controller_tick_topic_.c_str(),
        tick_timeout_sec_
      );

      // perc/compressd_img/raw_img
      if (bridge_type_ == "perc") {
        RCLCPP_INFO_ONCE(this->get_logger(),
          "Bridge type: perc, tag_raw_det_topic: %s, tros_bridged_tag_topic: %s",
          tag_raw_det_topic_.c_str(), tros_bridged_tag_topic_.c_str());
        tag_detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            tag_raw_det_topic_, 10, std::bind(&MsgBridgeNode::tag_detection_callback, this, std::placeholders::_1));
        perception_targets_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
            tros_bridged_tag_topic_, 10);
      } else if (bridge_type_ == "compressd_img") { 
        RCLCPP_INFO_ONCE(this->get_logger(),
          "Bridge type: perc, tag_raw_det_topic: %s, tros_bridged_tag_topic: %s",
          tag_raw_det_topic_.c_str(), tros_bridged_tag_topic_.c_str());
        compressd_img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            tag_raw_det_topic_, 10, std::bind(&MsgBridgeNode::compressd_img_callback, this, std::placeholders::_1));
        compressd_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            tros_bridged_tag_topic_, 10);
      }

      if (!controller_tick_topic_.empty() && tick_timeout_sec_ > 0) {
        controller_tick_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          controller_tick_topic_, 10, std::bind(&MsgBridgeNode::controller_tick_callback, this, std::placeholders::_1));
      }
    }
    ~MsgBridgeNode()
    {
    }

private:
  // perc/compressd_img/raw_img
  std::string bridge_type_ = "perc";
  std::string tag_raw_det_topic_ = "raw_apriltag_detections";
  std::string tros_bridged_tag_topic_ = "tros_apriltag_detections";
  std::string raw_compressd_img_topic_ = "image_jpeg";
  std::string tros_bridged_compressd_img_topic_ = "image_jpeg_compressed";
  std::string controller_tick_topic_ = "";

  float tick_timeout_sec_ = 3.0;

  bool isEnabled(const rclcpp::Time & stamp);

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detection_sub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr perception_targets_pub_;
  void tag_detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressd_img_sub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressd_img_pub_ = nullptr;
  void compressd_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr controller_tick_sub_ = nullptr;
  void controller_tick_callback(const std_msgs::msg::Bool::SharedPtr msg);
  std::mutex controller_tick_mutex_;
  rclcpp::Time last_tick_time_ = rclcpp::Time(0);
};

bool MsgBridgeNode::isEnabled(const rclcpp::Time & stamp) {
  if (!controller_tick_topic_.empty() && tick_timeout_sec_ > 0) {
    std::lock_guard<std::mutex> lock(controller_tick_mutex_);
    if (stamp.seconds() - last_tick_time_.seconds() > tick_timeout_sec_) {
      return false;
    }
  }
  return true;
}

void MsgBridgeNode::tag_detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
  if (!msg || !perception_targets_pub_) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "tag_detection_callback, this msg appears once.");

  if (!isEnabled(rclcpp::Time(msg->header.stamp))) {
    RCLCPP_WARN_ONCE(this->get_logger(), "controller_tick_callback: timeout, this msg appears once.");
    return;
  }

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

void MsgBridgeNode::compressd_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if (!msg || !compressd_img_pub_) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "compressd_img_callback, this msg appears once.");

  if (!isEnabled(rclcpp::Time(msg->header.stamp))) {
    RCLCPP_WARN_ONCE(this->get_logger(), "controller_tick_callback: timeout, this msg appears once.");
    return;
  }

  compressd_img_pub_->publish(*msg);
}

void MsgBridgeNode::controller_tick_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg) return;
  RCLCPP_INFO_ONCE(this->get_logger(), "controller_tick_callback, this msg appears once.");
  std::lock_guard<std::mutex> lock(controller_tick_mutex_);
  last_tick_time_ = this->now();
}

int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgBridgeNode>());
  rclcpp::shutdown();
  
  return 0;
}