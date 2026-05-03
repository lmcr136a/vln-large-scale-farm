/**
 * map_loader_node.cpp
 * GLIM saved map 로드 → /map_pointcloud (PointCloud2) publish
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <glim/mapping/global_mapping.hpp>
#include <glim/util/config.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <boost/filesystem.hpp>

class MapLoaderNode : public rclcpp::Node {
public:
  MapLoaderNode() : Node("map_loader_node") {
    map_path_     = declare_parameter<std::string>("map_path", "");
    points_topic_ = declare_parameter<std::string>("points_topic", "/map_pointcloud");
    map_frame_    = declare_parameter<std::string>("map_frame", "map");

    if (map_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "map_path param is empty!");
      throw std::runtime_error("map_path required");
    }

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::QoS(1).transient_local());

    load_and_publish();
  }

private:
  void load_and_publish() {
    RCLCPP_INFO(get_logger(), "Loading GLIM map: %s", map_path_.c_str());

    const std::string cfg = map_path_ + "/config";
    if (boost::filesystem::exists(cfg))
      glim::GlobalConfig::instance(cfg, true);

    glim::GlobalMappingParams params;
    params.enable_optimization = false;
    params.isam2_relinearize_skip = 1;
    params.isam2_relinearize_thresh = 0.0;

    auto gm = std::make_shared<glim::GlobalMapping>(params);
    if (!gm->load(map_path_)) {
      RCLCPP_FATAL(get_logger(), "Failed to load map from %s", map_path_.c_str());
      throw std::runtime_error("map load failed");
    }

    // export_points() → gtsam_points::PointCloud::Ptr (base class)
    auto merged = gm->export_points();
    if (!merged || !merged->has_points()) {
      RCLCPP_ERROR(get_logger(), "Map has no points!");
      return;
    }

    pub_->publish(to_pointcloud2(*merged));
    RCLCPP_INFO(get_logger(), "Published %zu pts on [%s]",
      merged->size(), points_topic_.c_str());
  }

  // gtsam_points::PointCloud (base) → ROS PointCloud2
  sensor_msgs::msg::PointCloud2 to_pointcloud2(const gtsam_points::PointCloud& cloud)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.height = 1;
    msg.width = cloud.size();
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(cloud.size());

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    for (size_t i = 0; i < cloud.size(); ++i, ++ix, ++iy, ++iz) {
      *ix = static_cast<float>(cloud.points[i][0]);
      *iy = static_cast<float>(cloud.points[i][1]);
      *iz = static_cast<float>(cloud.points[i][2]);
    }
    return msg;
  }

  std::string map_path_, points_topic_, map_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoaderNode>());
  rclcpp::shutdown();
}