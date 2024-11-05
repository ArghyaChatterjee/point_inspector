#include <iostream>
#include <rclcpp/rclcpp.hpp>

#define ROS2
#include <point_inspector.hpp>

class PointInspectorNode : public rclcpp::Node {
public:
  PointInspectorNode(rclcpp::NodeOptions& options) : rclcpp::Node("point_inspector", options) {
    RCLCPP_INFO_STREAM(this->get_logger(), "point_inspector");

    using std::placeholders::_1;
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS(), [](const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
      summarize(*points_msg);
    });

    RCLCPP_INFO_STREAM(this->get_logger(), "listening to " << points_sub->get_topic_name());
  }

private:
  rclcpp::SubscriptionBase::SharedPtr points_sub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<PointInspectorNode>(options));

  return 0;
}