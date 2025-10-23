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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "angles/angles.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PoseDetNode : public rclcpp::Node {
public:
    PoseDetNode()
    : Node("tros_apriltag_pose_det")
    {
      tag_refined_position_topic_ = this->declare_parameter("tag_refined_position_topic", tag_refined_position_topic_);
      tag_refined_pose_topic_ = this->declare_parameter("tros_bridged_tag_topic", tag_refined_pose_topic_);
      tag_raw_pose_frame_ = this->declare_parameter("tag_raw_pose_frame", tag_raw_pose_frame_);
      tag_refined_pose_frame_ = this->declare_parameter("tag_refined_pose_frame", tag_refined_pose_frame_);

      RCLCPP_WARN(this->get_logger(),
        "\n     tros_bridged_tag_topic: %s" \
        "\n tag_refined_position_topic: %s" \
        "\n         tag_raw_pose_frame: %s" \
        "\n     tag_refined_pose_frame: %s",
        tag_refined_position_topic_.c_str(),
        tag_refined_pose_topic_.c_str(),
        tag_raw_pose_frame_.c_str(),
        tag_refined_pose_frame_.c_str()
      );

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::shared_ptr<tf2_ros::TransformListener>(
        new tf2_ros::TransformListener(*tf_buffer_, this, false));

      tag_raw_pose_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          tag_refined_position_topic_, 10, std::bind(&PoseDetNode::tag_detection_callback, this, std::placeholders::_1));
      tag_refined_pose_sub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          tag_refined_pose_topic_, 10);
      tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf", 10);

      // timer_ = this->create_wall_timer(
      //   std::chrono::milliseconds(1000), std::bind(&PoseDetNode::timer_callback, this));
    }
    ~PoseDetNode()
    {
    }

private:
  const float pi = 3.1415926;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = nullptr;

  std::string tag_raw_pose_frame_ = "tag36h11:0";
  std::string tag_refined_pose_frame_ = "camera_link";

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  void timer_callback();

  std::string tag_refined_position_topic_ = "tros_tag_refined_position";
  std::string tag_refined_pose_topic_ = "tros_tag_refined_pose";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr tag_raw_pose_sub_ = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_refined_pose_sub_ = nullptr;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_ = nullptr;

  void tag_detection_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  
};

void PoseDetNode::tag_detection_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  if (!msg || !tag_refined_pose_sub_ || msg->targets.empty()) return;
  const auto& target = msg->targets.front();
  if (target.attributes.empty()) return;

  float x_cm = 0, y_cm = 0, z_cm = 0;
  int parsed_count = 0;
  for (const auto& attr : target.attributes) {
    if (attr.type == "x_cm") {
      x_cm = attr.value;
      parsed_count++;
    } else if (attr.type == "y_cm") {
      y_cm = attr.value;
      parsed_count++;
    } else if (attr.type == "z_cm") {
      z_cm = attr.value;
      parsed_count++;
    }
  }
  if (parsed_count != 3) return;

  RCLCPP_INFO_ONCE(this->get_logger(), "tag_detection_callback, this msg appears onece.");

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.header.frame_id = tag_refined_pose_frame_;
  pose_stamped.pose.position.x = x_cm * 0.01;
  pose_stamped.pose.position.y = y_cm * 0.01;
  pose_stamped.pose.position.z = z_cm * 0.01;

  try
  {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      tag_refined_pose_frame_, tag_raw_pose_frame_,
      tf2::TimePointZero
      // msg->header.stamp,
      // rclcpp::Duration::from_seconds(1.0)
    );
    double r, p, y;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    float yaw = y;
    if (yaw > pi) yaw -= (pi * 2.0);
    if (yaw < -1.0 * pi) yaw += (pi * 2.0);
    yaw += pi / 2.0;
    tf2::Matrix3x3 rot_mat;
    rot_mat.setRPY(0, 0, yaw);
    tf2::Quaternion pose_q;
    rot_mat.getRotation(pose_q);
    pose_stamped.pose.orientation =
      tf2::toMsg(pose_q);

    if (tf_pub_) {
      tf2_msgs::msg::TFMessage tf_msg;
      // geometry_msgs/TransformStamped[] transforms
      geometry_msgs::msg::TransformStamped tf;
      tf.header = pose_stamped.header;
      tf.header.frame_id = tag_refined_pose_frame_;
      tf.child_frame_id = "tros_refined_tag";
      tf.transform.translation.x = pose_stamped.pose.position.x;
      tf.transform.translation.y = pose_stamped.pose.position.y;
      tf.transform.translation.z = pose_stamped.pose.position.z;
      tf.transform.rotation = pose_stamped.pose.orientation;

      tf_msg.transforms.push_back(tf);
      tf_pub_->publish(std::move(tf_msg));
    }

    tag_refined_pose_sub_->publish(std::move(pose_stamped));
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return;
  }
}
  
void PoseDetNode::timer_callback()
{
  try
  {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      tag_refined_pose_frame_, tag_raw_pose_frame_,
      rclcpp::Time(0),
      // this->now(),
      rclcpp::Duration::from_seconds(1.0)
    );

    // get rpy
    double r, p, y;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    auto angle_r = angles::to_degrees(r);
    auto angle_p = angles::to_degrees(p);
    auto angle_y = angles::to_degrees(y);
    if (angle_r > 180) angle_r -= 360;
    if (angle_p > 180) angle_p -= 360;
    if (angle_y > 180) angle_y -= 360;
    if (angle_r < -180) angle_r += 360;
    if (angle_p < -180) angle_p += 360;
    if (angle_y < -180) angle_y += 360;

    angle_y += 90.0;

    RCLCPP_INFO(this->get_logger(),
      "rpy: %.2f, %.2f, %.2f",
      angle_r, angle_p, angle_y);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  

}

int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDetNode>());
  rclcpp::shutdown();
  
  return 0;
}