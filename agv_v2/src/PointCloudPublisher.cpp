#include "PointCloudPublisher.hpp"

PointCloudRepublisher::PointCloudRepublisher()
    : Node("point_cloud_republisher")
{
    // Create a publisher for PointCloud2
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/agv/points_fixed", 10);

    // Create a subscriber to the /agv/points topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/agv/points", 10,
        std::bind(&PointCloudRepublisher::point_cloud_callback, this, std::placeholders::_1)
    );
}

void PointCloudRepublisher::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Create a new PointCloud2 message to republish with the new frame
    auto new_point_cloud_msg = *msg;

    // Change the frame ID to camera_depth_optical_frame
    new_point_cloud_msg.header.frame_id = "camera_depth_frame";

    // Publish the modified PointCloud2 message
    publisher_->publish(new_point_cloud_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRepublisher>());
    rclcpp::shutdown();
    return 0;
}