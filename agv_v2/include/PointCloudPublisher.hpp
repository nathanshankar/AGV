#ifndef POINT_CLOUD_REPUBLISHER_HPP
#define POINT_CLOUD_REPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudRepublisher : public rclcpp::Node
{
public:
    PointCloudRepublisher();
    ~PointCloudRepublisher() = default;

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#endif // POINT_CLOUD_REPUBLISHER_HPP
