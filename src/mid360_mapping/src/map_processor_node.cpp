#include <chrono>
#include <cstdint>
#include <exception>
#include <cmath>
#include <algorithm>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapProcessorNode : public rclcpp::Node
{
public:
  MapProcessorNode() : Node("map_processor_node")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/mapping/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    output_directory_ = declare_parameter<std::string>("output_directory", "");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.10);
    // These are starting values to tune, not measured ground/obstacle thresholds.
    const auto minimum = declare_parameter<double>("obstacle_height_min", 0.15);
    const auto maximum = declare_parameter<double>("obstacle_height_max", 1.50);
    if (input.empty() || map_frame_.empty() || !std::isfinite(voxel_size_) || voxel_size_ <= 0 ||
      !std::isfinite(grid_resolution_) || grid_resolution_ <= 0 || !std::isfinite(minimum) ||
      !std::isfinite(maximum) || maximum <= minimum)
    {
      throw std::invalid_argument("Invalid map-processing startup parameters");
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++received_;
        accumulated_points_.clear();
        latest_cleaned_points_.clear();
        latest_grid_ = nav_msgs::msg::OccupancyGrid();
        has_processed_map_ = false;
        try {
          sensor_msgs::PointCloud2ConstIterator<float> x_in(*msg, "x");
          sensor_msgs::PointCloud2ConstIterator<float> y_in(*msg, "y");
          sensor_msgs::PointCloud2ConstIterator<float> z_in(*msg, "z");
          const auto point_count = static_cast<std::size_t>(msg->width) * msg->height;
          for (std::size_t i = 0; i < point_count; ++i, ++x_in, ++y_in, ++z_in) {
            const auto x_value = *x_in;
            const auto y_value = *y_in;
            const auto z_value = *z_in;
            if (!std::isfinite(x_value) || !std::isfinite(y_value) || !std::isfinite(z_value)) {
              continue;
            }

            const auto voxel_x = std::floor(static_cast<long double>(x_value) / voxel_size_);
            const auto voxel_y = std::floor(static_cast<long double>(y_value) / voxel_size_);
            const auto voxel_z = std::floor(static_cast<long double>(z_value) / voxel_size_);
            const auto min_index = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
            const auto max_index = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
            if (voxel_x < min_index || voxel_x > max_index || voxel_y < min_index ||
              voxel_y > max_index || voxel_z < min_index || voxel_z > max_index)
            {
              continue;
            }

            const VoxelKey key{
              static_cast<std::int64_t>(voxel_x), static_cast<std::int64_t>(voxel_y),
              static_cast<std::int64_t>(voxel_z)};
            accumulated_points_.try_emplace(key, PointXYZ{x_value, y_value, z_value});
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "Cannot read XYZ fields from map cloud: %s",
            error.what());
          return;
        }

        if (accumulated_points_.empty()) {
          return;
        }

        double mean_z = 0.0;
        double sum_squared_z = 0.0;
        std::size_t count = 0;
        for (const auto & entry : accumulated_points_) {
          ++count;
          const auto delta = static_cast<double>(entry.second.z) - mean_z;
          mean_z += delta / static_cast<double>(count);
          sum_squared_z += delta * (static_cast<double>(entry.second.z) - mean_z);
        }
        const auto standard_deviation = std::sqrt(sum_squared_z / static_cast<double>(count));
        const auto z_min = mean_z - 3.0 * standard_deviation;
        const auto z_max = mean_z + 3.0 * standard_deviation;

        std::vector<PointXYZ> cleaned_points;
        cleaned_points.reserve(accumulated_points_.size());
        for (const auto & entry : accumulated_points_) {
          const auto & point = entry.second;
          if (static_cast<double>(point.z) >= z_min && static_cast<double>(point.z) <= z_max) {
            cleaned_points.push_back(point);
          }
        }

        auto cleaned = sensor_msgs::msg::PointCloud2();
        cleaned.header = msg->header;
        if (cleaned.header.frame_id.empty()) {
          cleaned.header.frame_id = map_frame_;
        }
        sensor_msgs::PointCloud2Modifier modifier(cleaned);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cleaned_points.size());
        sensor_msgs::PointCloud2Iterator<float> x_out(cleaned, "x");
        sensor_msgs::PointCloud2Iterator<float> y_out(cleaned, "y");
        sensor_msgs::PointCloud2Iterator<float> z_out(cleaned, "z");
        for (const auto & point : cleaned_points) {
          *x_out = point.x;
          *y_out = point.y;
          *z_out = point.z;
          ++x_out;
          ++y_out;
          ++z_out;
        }
        cleaned.is_dense = true;
        cleaned_pub_->publish(cleaned);

        if (cleaned_points.empty()) {
          return;
        }

        struct GridCell
        {
          std::int64_t x;
          std::int64_t y;
        };
        std::vector<GridCell> cells;
        cells.reserve(cleaned_points.size());
        auto min_x = std::numeric_limits<std::int64_t>::max();
        auto min_y = std::numeric_limits<std::int64_t>::max();
        auto max_x = std::numeric_limits<std::int64_t>::min();
        auto max_y = std::numeric_limits<std::int64_t>::min();
        for (const auto & point : cleaned_points) {
          const auto cell_x = std::floor(static_cast<long double>(point.x) / grid_resolution_);
          const auto cell_y = std::floor(static_cast<long double>(point.y) / grid_resolution_);
          const auto min_index = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
          const auto max_index = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
          if (cell_x < min_index || cell_x > max_index || cell_y < min_index || cell_y > max_index) {
            continue;
          }
          const auto x_cell = static_cast<std::int64_t>(cell_x);
          const auto y_cell = static_cast<std::int64_t>(cell_y);
          cells.push_back(GridCell{x_cell, y_cell});
          min_x = std::min(min_x, x_cell);
          min_y = std::min(min_y, y_cell);
          max_x = std::max(max_x, x_cell);
          max_y = std::max(max_y, y_cell);
        }
        if (cells.empty()) {
          return;
        }

        const auto max_dimension = static_cast<std::int64_t>(
          std::numeric_limits<std::uint32_t>::max());
        if ((min_x < 0 && max_x > std::numeric_limits<std::int64_t>::max() + min_x) ||
          (min_y < 0 && max_y > std::numeric_limits<std::int64_t>::max() + min_y))
        {
          RCLCPP_ERROR(get_logger(), "Generated occupancy grid dimensions are too large");
          return;
        }
        const auto width_delta = max_x - min_x;
        const auto height_delta = max_y - min_y;
        if (width_delta >= max_dimension || height_delta >= max_dimension) {
          RCLCPP_ERROR(get_logger(), "Generated occupancy grid dimensions are too large");
          return;
        }
        const auto width_signed = width_delta + 1;
        const auto height_signed = height_delta + 1;
        const auto width = static_cast<std::uint32_t>(width_signed);
        const auto height = static_cast<std::uint32_t>(height_signed);
        if (static_cast<std::size_t>(width) >
          std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height))
        {
          RCLCPP_ERROR(get_logger(), "Generated occupancy grid dimensions are too large");
          return;
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header = cleaned.header;
        grid.info.resolution = static_cast<float>(grid_resolution_);
        grid.info.width = width;
        grid.info.height = height;
        // The origin is the lower-left corner of the minimum XY cell; observed cells are occupied,
        // while unobserved cells stay unknown because this point cloud provides no ray clearing.
        grid.info.origin.position.x = static_cast<double>(min_x) * grid_resolution_;
        grid.info.origin.position.y = static_cast<double>(min_y) * grid_resolution_;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(static_cast<std::size_t>(width) * height, -1);
        for (const auto & cell : cells) {
          const auto row = static_cast<std::uint64_t>(cell.y) - static_cast<std::uint64_t>(min_y);
          const auto column = static_cast<std::uint64_t>(cell.x) - static_cast<std::uint64_t>(min_x);
          const auto index = static_cast<std::size_t>(row * width + column);
          grid.data[index] = 100;
        }
        latest_cleaned_points_ = cleaned_points;
        latest_grid_ = grid;
        has_processed_map_ = true;
        grid_pub_->publish(grid);
      });
    cleaned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapping/map_filtered", rclcpp::SensorDataQoS());
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "mapping/map_2d", rclcpp::QoS(1).reliable().transient_local());
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "mapping/save_artifacts",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        save_artifacts(response);
      });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(
        get_logger(), "Received %llu map messages; latest processed map has %zu points",
        static_cast<unsigned long long>(received_), latest_cleaned_points_.size());
    });
  }

private:
  std::uint64_t received_{0};
  struct PointXYZ
  {
    float x;
    float y;
    float z;
  };
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
  void save_artifacts(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!has_processed_map_) {
      response->success = false;
      response->message = "No processed map is available to save";
      return;
    }
    if (output_directory_.empty()) {
      response->success = false;
      response->message = "The output_directory parameter is empty";
      return;
    }

    try {
      const std::filesystem::path directory(output_directory_);
      std::filesystem::create_directories(directory);
      const auto pcd_path = directory / "map_filtered.pcd";
      const auto pgm_path = directory / "map.pgm";
      const auto yaml_path = directory / "map.yaml";

      std::ofstream pcd(pcd_path);
      if (!pcd) {
        throw std::runtime_error("Cannot open " + pcd_path.string());
      }
      pcd << "# .PCD v0.7 - Point Cloud Data file format\n"
          << "VERSION 0.7\n"
          << "FIELDS x y z\n"
          << "SIZE 4 4 4\n"
          << "TYPE F F F\n"
          << "COUNT 1 1 1\n"
          << "WIDTH " << latest_cleaned_points_.size() << '\n'
          << "HEIGHT 1\n"
          << "VIEWPOINT 0 0 0 1 0 0 0\n"
          << "POINTS " << latest_cleaned_points_.size() << '\n'
          << "DATA ascii\n" << std::setprecision(std::numeric_limits<float>::max_digits10);
      for (const auto & point : latest_cleaned_points_) {
        pcd << point.x << ' ' << point.y << ' ' << point.z << '\n';
      }
      pcd.close();
      if (!pcd) {
        throw std::runtime_error("Failed while writing " + pcd_path.string());
      }

      const auto width = latest_grid_.info.width;
      const auto height = latest_grid_.info.height;
      std::ofstream pgm(pgm_path, std::ios::binary);
      if (!pgm) {
        throw std::runtime_error("Cannot open " + pgm_path.string());
      }
      pgm << "P5\n" << width << ' ' << height << "\n255\n";
      for (std::uint32_t image_y = 0; image_y < height; ++image_y) {
        const auto grid_y = height - 1 - image_y;
        for (std::uint32_t x = 0; x < width; ++x) {
          const auto occupancy = latest_grid_.data[
            static_cast<std::size_t>(grid_y) * width + x];
          const auto pixel = static_cast<unsigned char>(occupancy >=50 ? 0 : (occupancy < 0 ? 205 : 0));
          pgm.write(reinterpret_cast<const char *>(&pixel), sizeof(pixel));
        }
      }
      pgm.close();
      if (!pgm) {
        throw std::runtime_error("Failed while writing " + pgm_path.string());
      }

      std::ofstream yaml(yaml_path);
      if (!yaml) {
        throw std::runtime_error("Cannot open " + yaml_path.string());
      }
      yaml << "image: map.pgm\n"
           << std::setprecision(9)
           << "resolution: " << latest_grid_.info.resolution << '\n'
           << "origin: [" << latest_grid_.info.origin.position.x << ", "
           << latest_grid_.info.origin.position.y << ", 0.0]\n"
           << "negate: 0\n"
           << "occupied_thresh: 0.65\n"
           << "free_thresh: 0.196\n";
      yaml.close();
      if (!yaml) {
        throw std::runtime_error("Failed while writing " + yaml_path.string());
      }

      response->success = true;
      response->message = "Saved " + pcd_path.string() + ", " + pgm_path.string() +
        " and " + yaml_path.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Failed to save map artifacts: ") + error.what();
    }
  }

  std::string map_frame_;
  std::string output_directory_;
  double voxel_size_{0.10};
  double grid_resolution_{0.10};
  std::unordered_map<VoxelKey, PointXYZ, VoxelKeyHash> accumulated_points_;
  std::vector<PointXYZ> latest_cleaned_points_;
  nav_msgs::msg::OccupancyGrid latest_grid_;
  bool has_processed_map_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<MapProcessorNode>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_processor_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
