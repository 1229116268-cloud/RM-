#include "mid360_mapping/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace mid360_mapping
{
bool MappingNode::make_voxel_key(const PointXYZI & point, double voxel_size, VoxelKey & key)
{
  const auto x = std::floor(static_cast<long double>(point.x) / voxel_size);
  const auto y = std::floor(static_cast<long double>(point.y) / voxel_size);
  const auto z = std::floor(static_cast<long double>(point.z) / voxel_size);
  const auto minimum = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  const auto maximum = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  if (x < minimum || x > maximum || y < minimum || y > maximum || z < minimum || z > maximum) {
    return false;
  }
  key = VoxelKey{
    static_cast<std::int64_t>(x), static_cast<std::int64_t>(y), static_cast<std::int64_t>(z)};
  return true;
}

MappingNode::MappingNode() : Node("mapping_node")
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  startup.description = "Startup setting; restart the node after changing configuration";
  const auto registered_cloud_topic = declare_parameter<std::string>(
    "registered_cloud_topic", "/cloud_registered", startup);
  const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/Odometry", startup);
  const auto path_topic = declare_parameter<std::string>("path_topic", "/path", startup);
  map_output_path_ = declare_parameter<std::string>("map_output_path", "", startup);
  voxel_size_ = declare_parameter<double>("voxel_size", 0.10, startup);
  if (registered_cloud_topic.empty() || odometry_topic.empty() || path_topic.empty()) {
    throw std::invalid_argument("FAST-LIO output topics must not be empty");
  }
  if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0) {
    throw std::invalid_argument("voxel_size must be finite and greater than zero");
  }

  registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_cloud_topic, rclcpp::SensorDataQoS().keep_last(10),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {on_registered_cloud(message);});
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic, rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
      odom_pub_->publish(*message);
    });
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(10),
    [this](nav_msgs::msg::Path::ConstSharedPtr message) {path_pub_->publish(*message);});

  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/map", rclcpp::SensorDataQoS());
  registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/cloud_registered", rclcpp::SensorDataQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("mapping/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("mapping/path", 10);
  save_service_ = create_service<std_srvs::srv::Trigger>(
    "mapping/save_map",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      save_map(response);
    });
  publish_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_map();});
  status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    RCLCPP_INFO(
      get_logger(), "FAST-LIO clouds=%llu, accumulated map points=%zu",
      static_cast<unsigned long long>(cloud_count_), map_points_.size());
  });
  RCLCPP_INFO(
    get_logger(), "Waiting for FAST-LIO registered clouds on '%s'",
    registered_cloud_topic.c_str());
}

void MappingNode::on_registered_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++cloud_count_;
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring registered cloud with an empty frame_id");
    return;
  }
  if (!map_frame_.empty() && map_frame_ != message->header.frame_id) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Registered cloud frame '%s' does not match map_frame '%s'",
      message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  std::vector<PointXYZI> scan_points;
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x_in(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_in(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_in(*message, "z");
    const auto has_intensity = std::any_of(
      message->fields.begin(), message->fields.end(),
      [](const sensor_msgs::msg::PointField & field) {return field.name == "intensity";});
    scan_points.reserve(static_cast<std::size_t>(message->width) * message->height);
    if (has_intensity) {
      sensor_msgs::PointCloud2ConstIterator<float> intensity_in(*message, "intensity");
      for (std::size_t i = 0; i < static_cast<std::size_t>(message->width) * message->height;
        ++i, ++x_in, ++y_in, ++z_in, ++intensity_in)
      {
        if (std::isfinite(*x_in) && std::isfinite(*y_in) && std::isfinite(*z_in)) {
          scan_points.push_back(PointXYZI{*x_in, *y_in, *z_in, *intensity_in});
        }
      }
    } else {
      for (std::size_t i = 0; i < static_cast<std::size_t>(message->width) * message->height;
        ++i, ++x_in, ++y_in, ++z_in)
      {
        if (std::isfinite(*x_in) && std::isfinite(*y_in) && std::isfinite(*z_in)) {
          scan_points.push_back(PointXYZI{*x_in, *y_in, *z_in, 0.0F});
        }
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Cannot read FAST-LIO registered cloud: %s", error.what());
    return;
  }

  for (const auto & point : scan_points) {
    VoxelKey key{};
    if (make_voxel_key(point, voxel_size_, key)) {
      const auto inserted = map_points_.try_emplace(key, point).second;
      map_dirty_ = map_dirty_ || inserted;
    }
  }
  latest_registered_cloud_ = std::move(message);
  registered_cloud_pending_ = true;
}

void MappingNode::publish_map()
{
  if (registered_cloud_pending_ && latest_registered_cloud_) {
    registered_pub_->publish(*latest_registered_cloud_);
    registered_cloud_pending_ = false;
  }
  if (!map_dirty_ || map_points_.empty() || !latest_registered_cloud_) {
    return;
  }

  auto map = sensor_msgs::msg::PointCloud2();
  map.header = latest_registered_cloud_->header;
  sensor_msgs::PointCloud2Modifier modifier(map);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(map_points_.size());
  sensor_msgs::PointCloud2Iterator<float> x_out(map, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(map, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(map, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_out(map, "intensity");
  for (const auto & entry : map_points_) {
    *x_out = entry.second.x;
    *y_out = entry.second.y;
    *z_out = entry.second.z;
    *intensity_out = entry.second.intensity;
    ++x_out;
    ++y_out;
    ++z_out;
    ++intensity_out;
  }
  map.is_dense = true;
  map_pub_->publish(map);
  map_dirty_ = false;
}

void MappingNode::save_map(
  const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
{
  if (map_points_.empty()) {
    response->success = false;
    response->message = "No FAST-LIO registered points are available to save";
    return;
  }
  if (map_output_path_.empty()) {
    response->success = false;
    response->message = "The map_output_path parameter is empty";
    return;
  }

  try {
    const std::filesystem::path output_path(map_output_path_);
    if (output_path.extension() != ".pcd") {
      throw std::invalid_argument("map_output_path must have a .pcd extension");
    }
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream output(output_path);
    if (!output) {
      throw std::runtime_error("Cannot open " + output_path.string());
    }

    std::vector<std::pair<VoxelKey, PointXYZI>> points(map_points_.begin(), map_points_.end());
    std::sort(points.begin(), points.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.first.x != rhs.first.x) {
        return lhs.first.x < rhs.first.x;
      }
      if (lhs.first.y != rhs.first.y) {
        return lhs.first.y < rhs.first.y;
      }
      return lhs.first.z < rhs.first.z;
    });
    output << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F F\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH " << points.size() << '\n'
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << points.size() << "\n"
           << "DATA ascii\n"
           << std::setprecision(std::numeric_limits<float>::max_digits10);
    for (const auto & entry : points) {
      const auto & point = entry.second;
      output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << '\n';
    }
    output.close();
    if (!output) {
      throw std::runtime_error("Failed while writing " + output_path.string());
    }
    response->success = true;
    response->message = "Saved " + std::to_string(points.size()) + " points to " +
      output_path.string();
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("Failed to save map: ") + error.what();
  }
}
}  // namespace mid360_mapping
