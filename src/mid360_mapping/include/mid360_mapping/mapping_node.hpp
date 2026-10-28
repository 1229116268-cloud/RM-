#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mid360_mapping
{
class MappingNode : public rclcpp::Node
{
public:
  MappingNode();

private:
  struct VoxelKey
  {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      auto seed = std::hash<std::int64_t>{}(key.x);
      seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
  struct PointXYZI
  {
    float x;
    float y;
    float z;
    float intensity;
  };

  static bool make_voxel_key(
    const PointXYZI & point, double voxel_size, VoxelKey & key);

  void on_registered_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void publish_map();
  void save_map(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response);

  std::uint64_t cloud_count_{0};
  std::string map_frame_;
  std::string map_output_path_;
  double voxel_size_{0.10};
  std::unordered_map<VoxelKey, PointXYZI, VoxelKeyHash> map_points_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_registered_cloud_;
  bool registered_cloud_pending_{false};
  bool map_dirty_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace mid360_mapping
