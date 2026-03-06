#include <cmath>
#include <memory>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

static constexpr double WGS84_A  = 6378137.0;
static constexpr double WGS84_E2 = 0.00669437999014;

static std::array<double, 3> geodetic_to_ecef(double lat, double lon, double alt)
{
  const double lat_r = lat * M_PI / 180.0;
  const double lon_r = lon * M_PI / 180.0;
  const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_r) * std::sin(lat_r));
  return {
    (N + alt) * std::cos(lat_r) * std::cos(lon_r),
    (N + alt) * std::cos(lat_r) * std::sin(lon_r),
    (N * (1.0 - WGS84_E2) + alt) * std::sin(lat_r)
  };
}

static std::array<double, 3> ecef_to_enu(
  const std::array<double, 3>& ecef,
  const std::array<double, 3>& origin_ecef,
  double origin_lat, double origin_lon)
{
  const double lat_r = origin_lat * M_PI / 180.0;
  const double lon_r = origin_lon * M_PI / 180.0;
  const double dx = ecef[0] - origin_ecef[0];
  const double dy = ecef[1] - origin_ecef[1];
  const double dz = ecef[2] - origin_ecef[2];
  return {
    -std::sin(lon_r) * dx + std::cos(lon_r) * dy,
    -std::sin(lat_r) * std::cos(lon_r) * dx
      - std::sin(lat_r) * std::sin(lon_r) * dy
      + std::cos(lat_r) * dz,
    std::cos(lat_r) * std::cos(lon_r) * dx
      + std::cos(lat_r) * std::sin(lon_r) * dy
      + std::sin(lat_r) * dz
  };
}

class GlimConverters : public rclcpp::Node
{
public:
  GlimConverters() : Node("glim_converters"), gps_origin_set_(false)
  {
    declare_parameter("lidar_input_topic",  "/livox/lidar");
    declare_parameter("lidar_output_topic", "/livox/lidar_pointcloud2");
    declare_parameter("gps_input_topic",    "/gps/fix");
    declare_parameter("gps_output_topic",   "/gps/fix_pose");
    declare_parameter("gps_enu_yaw_deg",    214.8);
    declare_parameter("gps_offset_x",       0.0);
    declare_parameter("gps_offset_y",       0.1);
    declare_parameter("gps_offset_z",       0.0);

    const auto lidar_in  = get_parameter("lidar_input_topic").as_string();
    const auto lidar_out = get_parameter("lidar_output_topic").as_string();
    const auto gps_in    = get_parameter("gps_input_topic").as_string();
    const auto gps_out   = get_parameter("gps_output_topic").as_string();

    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_in, 10,
      std::bind(&GlimConverters::lidar_callback, this, std::placeholders::_1));
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_out, 10);

    fields_.resize(5);
    fields_[0].name = "x";         fields_[0].offset = 0;  fields_[0].datatype = sensor_msgs::msg::PointField::FLOAT32; fields_[0].count = 1;
    fields_[1].name = "y";         fields_[1].offset = 4;  fields_[1].datatype = sensor_msgs::msg::PointField::FLOAT32; fields_[1].count = 1;
    fields_[2].name = "z";         fields_[2].offset = 8;  fields_[2].datatype = sensor_msgs::msg::PointField::FLOAT32; fields_[2].count = 1;
    fields_[3].name = "intensity"; fields_[3].offset = 12; fields_[3].datatype = sensor_msgs::msg::PointField::FLOAT32; fields_[3].count = 1;
    fields_[4].name = "time";      fields_[4].offset = 16; fields_[4].datatype = sensor_msgs::msg::PointField::FLOAT32; fields_[4].count = 1;

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_in, 10,
      std::bind(&GlimConverters::gps_callback, this, std::placeholders::_1));
    gps_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(gps_out, 10);

    RCLCPP_INFO(get_logger(), "[LiDAR] %s -> %s", lidar_in.c_str(), lidar_out.c_str());
    RCLCPP_INFO(get_logger(), "[GPS]   %s -> %s", gps_in.c_str(), gps_out.c_str());
  }

private:
  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const uint32_t n = msg->point_num;
    if (n == 0) return;

    constexpr uint32_t POINT_STEP = 20;
    std::vector<uint8_t> data(n * POINT_STEP);

    float* ptr = reinterpret_cast<float*>(data.data());
    for (uint32_t i = 0; i < n; ++i) {
      const auto& p = msg->points[i];
      ptr[i * 5 + 0] = p.x;
      ptr[i * 5 + 1] = p.y;
      ptr[i * 5 + 2] = p.z;
      ptr[i * 5 + 3] = static_cast<float>(p.reflectivity);
      ptr[i * 5 + 4] = static_cast<float>(p.offset_time) * 1e-9f;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header       = msg->header;
    out.height       = 1;
    out.width        = n;
    out.fields       = fields_;
    out.is_bigendian = false;
    out.point_step   = POINT_STEP;
    out.row_step     = POINT_STEP * n;
    out.data         = std::move(data);
    out.is_dense     = false;

    lidar_pub_->publish(out);
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < 0) return;

    const double lat = msg->latitude;
    const double lon = msg->longitude;
    const double alt = msg->altitude;
    const auto ecef = geodetic_to_ecef(lat, lon, alt);

    if (!gps_origin_set_) {
      gps_origin_ecef_ = ecef;
      gps_origin_lat_  = lat;
      gps_origin_lon_  = lon;
      gps_origin_set_  = true;
      RCLCPP_INFO(get_logger(), "[GPS] origin set: lat=%.6f lon=%.6f alt=%.2f", lat, lon, alt);
    }

    const auto enu = ecef_to_enu(ecef, gps_origin_ecef_, gps_origin_lat_, gps_origin_lon_);

    const double yaw   = get_parameter("gps_enu_yaw_deg").as_double() * M_PI / 180.0;
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);

    const double ox = get_parameter("gps_offset_x").as_double();
    const double oy = get_parameter("gps_offset_y").as_double();
    const double oz = get_parameter("gps_offset_z").as_double();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header          = msg->header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x    = cos_y * enu[0] + sin_y * enu[1] - ox;
    pose_msg.pose.pose.position.y    = -sin_y * enu[0] + cos_y * enu[1] - oy;
    pose_msg.pose.pose.position.z    = enu[2] - oz;

    const double yaw_corr = get_parameter("gps_enu_yaw_deg").as_double() * M_PI / 180.0;
    pose_msg.pose.pose.orientation.z = std::cos(yaw_corr / 2.0);
    pose_msg.pose.pose.orientation.w = std::sin(yaw_corr / 2.0);

    if (msg->position_covariance_type > 0) {
      pose_msg.pose.covariance[0]  = msg->position_covariance[0];
      pose_msg.pose.covariance[1]  = msg->position_covariance[1];
      pose_msg.pose.covariance[2]  = msg->position_covariance[2];
      pose_msg.pose.covariance[6]  = msg->position_covariance[3];
      pose_msg.pose.covariance[7]  = msg->position_covariance[4];
      pose_msg.pose.covariance[8]  = msg->position_covariance[5];
      pose_msg.pose.covariance[12] = msg->position_covariance[6];
      pose_msg.pose.covariance[13] = msg->position_covariance[7];
      pose_msg.pose.covariance[14] = msg->position_covariance[8];
    } else {
      pose_msg.pose.covariance[0] = pose_msg.pose.covariance[7] = pose_msg.pose.covariance[14] = 1.0;
    }

    gps_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         lidar_pub_;
  std::vector<sensor_msgs::msg::PointField> fields_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  gps_pub_;
  bool gps_origin_set_;
  std::array<double, 3> gps_origin_ecef_{};
  double gps_origin_lat_{0.0}, gps_origin_lon_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlimConverters>());
  rclcpp::shutdown();
  return 0;
}