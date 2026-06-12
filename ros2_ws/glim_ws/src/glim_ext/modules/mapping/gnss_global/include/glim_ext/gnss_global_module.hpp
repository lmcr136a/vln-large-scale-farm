#pragma once

#include <cmath>
#include <deque>
#include <atomic>
#include <thread>
#include <numeric>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/QR>

#define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>

#ifdef GLIM_ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS2;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using NavSatFixConstPtr = sensor_msgs::msg::NavSatFix::ConstSharedPtr;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}
#else
#include <glim/util/extension_module_ros.hpp>
using ExtensionModuleBase = glim::ExtensionModuleROS;
#endif

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <glim_ext/geodetic.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

class GNSSGlobal : public ExtensionModuleBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSGlobal() : logger(create_module_logger("gnss_global")) {
    logger->info("initializing GNSS global constraints");
    const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_global");
    logger->info("gnss_global_config_path={}", config_path);

    glim::Config config(config_path);
    gnss_topic = config.param<std::string>("gnss", "gnss_topic", "/gps/fix");
    prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "prior_inf_scale", Eigen::Vector3d(1e3, 1e3, 0.0));
    min_baseline = config.param<double>("gnss", "min_baseline", 5.0);

    lever_arm_body = config.param<Eigen::Vector3d>("gnss", "lever_arm_body", Eigen::Vector3d::Zero());
    lever_arm_refine_min_submaps = config.param<int>("gnss", "lever_arm_refine_min_submaps", 30);

    logger->info("lever_arm_body (approx): [{:.3f}, {:.3f}, {:.3f}] m",
      lever_arm_body.x(), lever_arm_body.y(), lever_arm_body.z());

    transformation_initialized = false;
    lever_arm_refined = false;
    enu_origin_set = false;
    T_world_enu.setIdentity();
    R_enu_ecef.setIdentity();
    T_sg_set = false;
    T_sg.setIdentity();
    needs_republish = false;

    pub_node = rclcpp::Node::make_shared("gnss_global_pub");
    gps_path_pub = pub_node->create_publisher<nav_msgs::msg::Path>("/glim_ros/fixed_gps", 10);
    gps_path_msg.header.frame_id = "map";
    raw_gps_path_pub = pub_node->create_publisher<nav_msgs::msg::Path>("/glim_ros/raw_gps_world", 10);
    raw_gps_path_msg.header.frame_id = "map";

    rclcpp::QoS tl_qos(1);
    tl_qos.transient_local().reliable();
    t_sg_sub = pub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localizer/T_glimworld_savedworld", tl_qos,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        Eigen::Isometry3d T_gw = Eigen::Isometry3d::Identity();
        T_gw.translation() = Eigen::Vector3d(
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        T_gw.linear() = Eigen::Quaterniond(
          msg->pose.orientation.w, msg->pose.orientation.x,
          msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();
        std::lock_guard<std::mutex> lock(sg_mutex);
        if (!T_sg_set) {
          T_sg = T_gw.inverse();
          T_sg_set = true;
          if (transformation_initialized) {
            T_world_enu = T_sg * T_world_enu;
            needs_republish = true;
            logger->info("T_world_enu remapped to saved map frame (post-alignment), republishing all GPS points");
          } else {
            logger->info("T_sg received (pre-alignment), will apply when T_world_enu is initialized");
          }
        }
      });

    pub_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    pub_executor->add_node(pub_node);
    pub_executor_thread = std::thread([this] { pub_executor->spin(); });

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
  }

  ~GNSSGlobal() {
    kill_switch = true;
    thread.join();
    pub_executor->cancel();
    pub_executor_thread.join();
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    const auto gps_sub = std::make_shared<TopicSubscription<NavSatFix>>(
      gnss_topic,
      [this](const NavSatFixConstPtr msg) { gnss_callback(msg); });
    return {gps_sub};
  }

  void gnss_callback(const NavSatFixConstPtr& msg) {
    if (msg->status.status < 0) return;

    const double lat = msg->latitude;
    const double lon = msg->longitude;
    const double alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    const double stamp = to_sec(msg->header.stamp);

    const Eigen::Vector3d ecef = wgs84_to_ecef(lat, lon, alt);

    if (!enu_origin_set) {
      origin_ecef = ecef;
      const double lat_r = lat * M_PI / 180.0;
      const double lon_r = lon * M_PI / 180.0;
      R_enu_ecef.row(0) = Eigen::Vector3d(-std::sin(lon_r), std::cos(lon_r), 0.0);
      R_enu_ecef.row(1) = Eigen::Vector3d(-std::sin(lat_r) * std::cos(lon_r), -std::sin(lat_r) * std::sin(lon_r), std::cos(lat_r));
      R_enu_ecef.row(2) = Eigen::Vector3d(std::cos(lat_r) * std::cos(lon_r), std::cos(lat_r) * std::sin(lon_r), std::sin(lat_r));
      enu_origin_set = true;
      logger->info("ENU origin set: lat={:.7f} lon={:.7f} alt={:.2f}", lat, lon, alt);
    }

    const Eigen::Vector3d enu = R_enu_ecef * (ecef - origin_ecef);

    Eigen::Vector4d gnss_data;
    gnss_data << stamp, enu.x(), enu.y(), enu.z();
    input_gnss_queue.push_back(gnss_data);

    if (transformation_initialized) {
      const Eigen::Vector3d xyz_world = T_world_enu * enu;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = msg->header.stamp;
      pose.pose.position.x = xyz_world.x();
      pose.pose.position.y = xyz_world.y();
      pose.pose.position.z = xyz_world.z();
      pose.pose.orientation.w = 1.0;
      raw_gps_path_msg.header.stamp = msg->header.stamp;
      raw_gps_path_msg.poses.push_back(pose);
      raw_gps_path_pub->publish(raw_gps_path_msg);
    }
  }

  void on_insert_submap(const SubMap::ConstPtr& submap) {
    input_submap_queue.push_back(submap);
  }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      logger->debug("insert {} GNSS prior factors", factors.size());
      new_factors.add(factors);
    }
  }

  void backend_task() {
    logger->info("starting GNSS global thread");
    std::deque<Eigen::Vector4d> enu_queue;
    std::deque<SubMap::ConstPtr> submap_queue;

    while (!kill_switch) {
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      enu_queue.insert(enu_queue.end(), gnss_data.begin(), gnss_data.end());

      const auto new_submaps = input_submap_queue.get_all_and_clear();
      if (new_submaps.empty()) {
        {
          std::lock_guard<std::mutex> lock(sg_mutex);
          if (needs_republish && transformation_initialized) {
            gps_path_msg.poses.clear();
            raw_gps_path_msg.poses.clear();
            num_factored = 0;
            needs_republish = false;
            const auto model = gtsam::noiseModel::Diagonal::Precisions(prior_inf_scale);
            for (size_t i = 0; i < submaps.size(); i++) {
              const Eigen::Vector3d p_gnss_world = T_world_enu * submap_coords[i].tail<3>();
              const Eigen::Matrix3d R_world_lidar = submaps[i]->T_world_origin.linear();
              const Eigen::Vector3d xyz = p_gnss_world - R_world_lidar * lever_arm_body;
              publish_gps_point(xyz, submaps[i]->frames.back()->stamp);
              output_factors.push_back(
                gtsam::NonlinearFactor::shared_ptr(
                  new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submaps[i]->id), xyz, model)));
            }
            num_factored = submaps.size();
            logger->info("Republished {} GPS points in saved map frame", submaps.size());
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

      while (!enu_queue.empty() && !submap_queue.empty() &&
             submap_queue.front()->frames.front()->stamp < enu_queue.front()[0]) {
        submap_queue.pop_front();
      }

      while (!enu_queue.empty() && !submap_queue.empty() &&
             submap_queue.front()->frames.front()->stamp > enu_queue.front()[0] &&
             submap_queue.front()->frames.back()->stamp < enu_queue.back()[0]) {
        const auto& submap = submap_queue.front();
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto right = std::lower_bound(
          enu_queue.begin(), enu_queue.end(), stamp,
          [](const Eigen::Vector4d& v, const double t) { return v[0] < t; });

        if (right == enu_queue.end() || (right + 1) == enu_queue.end()) {
          logger->warn("invalid condition in GNSS global module!!");
          break;
        }
        const auto left = right - 1;

        const double tl = (*left)[0];
        const double tr = (*right)[0];
        const double p = (stamp - tl) / (tr - tl);
        const Eigen::Vector4d interpolated = (1.0 - p) * (*left) + p * (*right);

        submaps.push_back(submap);
        submap_coords.push_back(interpolated);

        submap_queue.pop_front();
        enu_queue.erase(enu_queue.begin(), left);
      }

      if (!transformation_initialized && !submaps.empty() &&
          (submaps.front()->T_world_origin.inverse() * submaps.back()->T_world_origin).translation().norm() > min_baseline) {
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < submaps.size(); i++) {
          mean_est += submaps[i]->T_world_origin.translation();
          mean_gnss += submap_coords[i].tail<3>();
        }
        mean_est /= submaps.size();
        mean_gnss /= submaps.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < submaps.size(); i++) {
          cov += (submap_coords[i].tail<3>() - mean_gnss) *
                 (submaps[i]->T_world_origin.translation() - mean_est).transpose();
        }
        cov /= submaps.size();

        const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
          cov.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
        if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
          S(1, 1) = -1;
        }

        Eigen::Isometry3d T_enu_world = Eigen::Isometry3d::Identity();
        T_enu_world.linear().block<2, 2>(0, 0) = svd.matrixU() * S * svd.matrixV().transpose();
        T_enu_world.translation() = mean_gnss - T_enu_world.linear() * mean_est;

        T_world_enu = T_enu_world.inverse();

        // If T_sg arrived before alignment was ready, apply it now
        {
          std::lock_guard<std::mutex> lock(sg_mutex);
          if (T_sg_set) {
            T_world_enu = T_sg * T_world_enu;
            logger->info("T_world_enu remapped to saved map frame (post-init)");
          }
        }

        transformation_initialized = true;
        logger->info("T_world_enu initialized — publishing aligned GPS to /glim_ros/fixed_gps");
      }

      if (transformation_initialized && !lever_arm_refined &&
          (int)submaps.size() >= lever_arm_refine_min_submaps) {
        refine_lever_arm();
      }

      if (transformation_initialized) {
        const auto model = gtsam::noiseModel::Diagonal::Precisions(prior_inf_scale);
        for (size_t i = num_factored; i < submaps.size(); i++) {
          const Eigen::Vector3d p_gnss_world = T_world_enu * submap_coords[i].tail<3>();
          const Eigen::Matrix3d R_world_lidar = submaps[i]->T_world_origin.linear();
          const Eigen::Vector3d xyz = p_gnss_world - R_world_lidar * lever_arm_body;

          logger->debug("submap={} gnss_corrected={}",
            convert_to_string(submaps[i]->T_world_origin.translation().eval()),
            convert_to_string(xyz));

          publish_gps_point(xyz, submaps[i]->frames.back()->stamp);

          output_factors.push_back(
            gtsam::NonlinearFactor::shared_ptr(
              new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submaps[i]->id), xyz, model)));
        }
        num_factored = submaps.size();
      }
    }
  }

private:
  void publish_gps_point(const Eigen::Vector3d& xyz, double stamp_sec) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp.sec = static_cast<int32_t>(stamp_sec);
    pose.header.stamp.nanosec = static_cast<uint32_t>((stamp_sec - pose.header.stamp.sec) * 1e9);
    pose.pose.position.x = xyz.x();
    pose.pose.position.y = xyz.y();
    pose.pose.position.z = xyz.z();
    pose.pose.orientation.w = 1.0;

    gps_path_msg.header.stamp = pose.header.stamp;
    gps_path_msg.poses.push_back(pose);
    gps_path_pub->publish(gps_path_msg);
  }

  void publish_all_gps_points() {
    for (size_t i = 0; i < submaps.size(); i++) {
      const Eigen::Vector3d p_gnss_world = T_world_enu * submap_coords[i].tail<3>();
      const Eigen::Matrix3d R_world_lidar = submaps[i]->T_world_origin.linear();
      const Eigen::Vector3d xyz = p_gnss_world - R_world_lidar * lever_arm_body;
      publish_gps_point(xyz, submaps[i]->frames.back()->stamp);
    }
  }

  void refine_lever_arm() {
    const int N = submaps.size();
    Eigen::MatrixXd A(3 * N, 3);
    Eigen::VectorXd b(3 * N);

    for (int i = 0; i < N; i++) {
      const Eigen::Vector3d p_gnss_world = T_world_enu * submap_coords[i].tail<3>();
      const Eigen::Matrix3d R = submaps[i]->T_world_origin.linear();
      const Eigen::Vector3d p_lidar = submaps[i]->T_world_origin.translation();
      A.block<3, 3>(3 * i, 0) = R;
      b.segment<3>(3 * i) = p_gnss_world - p_lidar;
    }

    const Eigen::Vector3d refined = A.colPivHouseholderQr().solve(b);

    if ((refined - lever_arm_body).norm() > 2.0) {
      logger->warn("lever arm refinement outlier (delta={:.3f} m), keeping approximate",
        (refined - lever_arm_body).norm());
      lever_arm_refined = true;
      return;
    }

    logger->info(
      "lever arm refined: [{:.3f}, {:.3f}, {:.3f}] m  (approx was [{:.3f}, {:.3f}, {:.3f}])",
      refined.x(), refined.y(), refined.z(),
      lever_arm_body.x(), lever_arm_body.y(), lever_arm_body.z());

    lever_arm_body = refined;
    lever_arm_refined = true;
  }

  std::atomic_bool kill_switch;
  std::thread thread;

  ConcurrentVector<Eigen::Vector4d> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<Eigen::Vector4d> submap_coords;
  size_t num_factored = 0;

  std::string gnss_topic;
  Eigen::Vector3d prior_inf_scale;
  double min_baseline;

  bool enu_origin_set;
  Eigen::Vector3d origin_ecef;
  Eigen::Matrix3d R_enu_ecef;

  bool transformation_initialized;
  Eigen::Isometry3d T_world_enu;

  std::mutex sg_mutex;
  bool T_sg_set;
  Eigen::Isometry3d T_sg;
  bool needs_republish;

  Eigen::Vector3d lever_arm_body;
  bool lever_arm_refined;
  int lever_arm_refine_min_submaps;

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_path_pub;
  nav_msgs::msg::Path gps_path_msg;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_gps_path_pub;
  nav_msgs::msg::Path raw_gps_path_msg;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr t_sg_sub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> pub_executor;
  std::thread pub_executor_thread;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}