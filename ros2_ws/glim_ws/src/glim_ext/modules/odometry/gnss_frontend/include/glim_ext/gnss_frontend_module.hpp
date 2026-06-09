#pragma once

#include <cmath>
#include <deque>
#include <vector>
#include <atomic>
#include <thread>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/SVD>

#define GLIM_ROS2

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>

#include <glim_ext/util/config_ext.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

using NavSatFix = sensor_msgs::msg::NavSatFix;
using NavSatFixConstPtr = sensor_msgs::msg::NavSatFix::ConstSharedPtr;

template <typename Stamp>
static double fe_to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}

static Eigen::Vector3d fe_wgs84_to_ecef(double lat, double lon, double alt) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double lat_r = lat * M_PI / 180.0;
  const double lon_r = lon * M_PI / 180.0;
  const double N = a / std::sqrt(1.0 - e2 * std::sin(lat_r) * std::sin(lat_r));
  return Eigen::Vector3d(
    (N + alt) * std::cos(lat_r) * std::cos(lon_r),
    (N + alt) * std::cos(lat_r) * std::sin(lon_r),
    (N * (1.0 - e2) + alt) * std::sin(lat_r));
}

class GNSSFrontend : public ExtensionModuleROS2 {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSFrontend() : logger(create_module_logger("gnss_frontend")) {
    logger->info("initializing GNSS front-end constraints");
    const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_frontend");
    glim::Config config(config_path);

    gnss_topic = config.param<std::string>("gnss_frontend", "gnss_topic", "/gps/fix");
    prior_inf_scale = config.param<Eigen::Vector3d>("gnss_frontend", "prior_inf_scale", Eigen::Vector3d(1e4, 1e4, 0.0));
    min_baseline = config.param<double>("gnss_frontend", "min_baseline", 5.0);
    min_matches = config.param<int>("gnss_frontend", "min_matches", 10);
    lever_arm_body = config.param<Eigen::Vector3d>("gnss_frontend", "lever_arm_body", Eigen::Vector3d::Zero());
    max_factor_lag_frames = config.param<int>("gnss_frontend", "max_factor_lag_frames", 40);

    enu_origin_set = false;
    aligned = false;
    origin_ecef.setZero();
    R_enu_ecef.setIdentity();
    T_world_enu.setIdentity();
    latest_frame_id = 0;
    num_factored = 0;

    pub_node = rclcpp::Node::make_shared("gnss_frontend_pub");
    gps_path_pub = pub_node->create_publisher<nav_msgs::msg::Path>("/glim_ros/frontend_gps", 10);
    gps_path_msg.header.frame_id = "map";
    pub_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    pub_executor->add_node(pub_node);
    pub_executor_thread = std::thread([this] { pub_executor->spin(); });

    OdometryEstimationCallbacks::on_update_frames.add([this](const std::vector<EstimationFrame::ConstPtr>& frames) {
      latest_frame_id = frames.back()->id;
      input_frame_queue.push_back(frames.back()->clone_wo_points());
    });

    OdometryEstimationCallbacks::on_smoother_update.add([this](
                                                          gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
                                                          gtsam::NonlinearFactorGraph& new_factors,
                                                          gtsam::Values& new_values,
                                                          std::map<std::uint64_t, double>& new_stamps) {
      const auto factors = output_factors.get_all_and_clear();
      if (!factors.empty()) {
        new_factors.add(factors);
      }
    });

    kill_switch = false;
    thread = std::thread([this] { task(); });
    logger->info("ready (prior_inf_scale=[{:.1e},{:.1e},{:.1e}] min_baseline={:.1f})",
      prior_inf_scale.x(), prior_inf_scale.y(), prior_inf_scale.z(), min_baseline);
  }

  ~GNSSFrontend() {
    kill_switch = true;
    thread.join();
    pub_executor->cancel();
    pub_executor_thread.join();
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    const auto sub = std::make_shared<TopicSubscription<NavSatFix>>(gnss_topic, [this](const NavSatFixConstPtr msg) { gnss_callback(msg); });
    return {sub};
  }

private:
  void gnss_callback(const NavSatFixConstPtr& msg) {
    if (msg->status.status < 0) {
      return;
    }
    const double lat = msg->latitude;
    const double lon = msg->longitude;
    const double alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    const double stamp = fe_to_sec(msg->header.stamp);
    const Eigen::Vector3d ecef = fe_wgs84_to_ecef(lat, lon, alt);

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
    Eigen::Vector4d data;
    data << stamp, enu.x(), enu.y(), enu.z();
    input_gnss_queue.push_back(data);
  }

  void task() {
    std::deque<Eigen::Vector4d> enu_queue;
    std::deque<EstimationFrame::ConstPtr> frame_queue;

    while (!kill_switch) {
      const auto new_gnss = input_gnss_queue.get_all_and_clear();
      enu_queue.insert(enu_queue.end(), new_gnss.begin(), new_gnss.end());
      const auto new_frames = input_frame_queue.get_all_and_clear();
      frame_queue.insert(frame_queue.end(), new_frames.begin(), new_frames.end());

      if (frame_queue.empty() || enu_queue.size() < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // Drop frames older than the available GPS span (cannot interpolate)
      while (!frame_queue.empty() && frame_queue.front()->stamp < enu_queue.front()[0]) {
        frame_queue.pop_front();
      }

      // Match frames bracketed by GPS via linear interpolation
      while (!frame_queue.empty() && frame_queue.front()->stamp <= enu_queue.back()[0]) {
        const auto frame = frame_queue.front();
        const double stamp = frame->stamp;

        const auto right = std::lower_bound(enu_queue.begin(), enu_queue.end(), stamp, [](const Eigen::Vector4d& v, double t) { return v[0] < t; });
        if (right == enu_queue.begin() || right == enu_queue.end()) {
          break;
        }
        const auto left = right - 1;
        const double tl = (*left)[0];
        const double tr = (*right)[0];
        const double p = (tr > tl) ? (stamp - tl) / (tr - tl) : 0.0;
        const Eigen::Vector3d enu = (1.0 - p) * left->tail<3>() + p * right->tail<3>();

        match_ids.push_back(frame->id);
        match_est.push_back(frame->T_world_imu.translation());
        match_R.push_back(frame->T_world_imu.linear());
        match_enu.push_back(enu);

        frame_queue.pop_front();
        enu_queue.erase(enu_queue.begin(), left);
      }

      // Estimate odom<-ENU alignment once enough motion is observed
      if (!aligned && (int)match_est.size() >= min_matches && (match_est.front() - match_est.back()).norm() > min_baseline) {
        estimate_alignment();
        aligned = true;
        num_factored = match_est.size();  // skip backlog (out of smoother window)
        logger->info("odom<-ENU alignment done with {} matches; GPS front-end factors active", match_est.size());
      }

      // Add strong GPS position priors for recent frames only
      if (aligned) {
        const auto model = gtsam::noiseModel::Diagonal::Precisions(prior_inf_scale);
        for (size_t i = num_factored; i < match_enu.size(); i++) {
          if (latest_frame_id - match_ids[i] > max_factor_lag_frames) {
            continue;  // too old, likely outside the fixed-lag window
          }
          const Eigen::Vector3d p_world = T_world_enu * match_enu[i];
          const Eigen::Vector3d xyz = p_world - match_R[i] * lever_arm_body;
          output_factors.push_back(gtsam::NonlinearFactor::shared_ptr(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(match_ids[i]), xyz, model)));
          publish_gps_point(xyz);
        }
        num_factored = match_enu.size();
      }
    }
  }

  void estimate_alignment() {
    const size_t N = match_est.size();
    Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; i++) {
      mean_est += match_est[i];
      mean_gnss += match_enu[i];
    }
    mean_est /= N;
    mean_gnss /= N;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; i++) {
      cov += (match_enu[i] - mean_gnss) * (match_est[i] - mean_est).transpose();
    }
    cov /= N;

    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
      S(1, 1) = -1;
    }

    Eigen::Isometry3d T_enu_world = Eigen::Isometry3d::Identity();
    T_enu_world.linear().block<2, 2>(0, 0) = svd.matrixU() * S * svd.matrixV().transpose();
    T_enu_world.translation() = mean_gnss - T_enu_world.linear() * mean_est;
    T_world_enu = T_enu_world.inverse();
  }

  void publish_gps_point(const Eigen::Vector3d& xyz) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = xyz.x();
    pose.pose.position.y = xyz.y();
    pose.pose.position.z = xyz.z();
    pose.pose.orientation.w = 1.0;
    gps_path_msg.poses.push_back(pose);
    gps_path_pub->publish(gps_path_msg);
  }

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  ConcurrentVector<Eigen::Vector4d> input_gnss_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<long> match_ids;
  std::vector<Eigen::Vector3d> match_est;
  std::vector<Eigen::Matrix3d> match_R;
  std::vector<Eigen::Vector3d> match_enu;
  size_t num_factored;

  std::string gnss_topic;
  Eigen::Vector3d prior_inf_scale;
  double min_baseline;
  int min_matches;
  Eigen::Vector3d lever_arm_body;
  int max_factor_lag_frames;

  std::atomic<long> latest_frame_id;

  bool enu_origin_set;
  Eigen::Vector3d origin_ecef;
  Eigen::Matrix3d R_enu_ecef;

  bool aligned;
  Eigen::Isometry3d T_world_enu;

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_path_pub;
  nav_msgs::msg::Path gps_path_msg;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> pub_executor;
  std::thread pub_executor_thread;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSFrontend();
}