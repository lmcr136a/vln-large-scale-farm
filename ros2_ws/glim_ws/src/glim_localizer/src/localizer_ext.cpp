/**
 * localizer_ext.cpp - GLIM extension module for localization against a saved map.
 *
 * Key transform:
 *   T_savedmap_glimworld = T_savedmap_lidar * T_glimworld_lidar^-1
 *
 * mount_R_ (Rx(180deg) for upside-down LiDAR):
 *   submap->T_world_origin includes Rx(180deg) from IMU gravity correction.
 *   Adding mount_R_ to T_savedmap_lidar makes Rx(180deg) cancel:
 *     (R_yaw * Rx180) * Rx180^-1 = R_yaw
 *
 * /glim_ros/entire_map publishes:
 *   saved map pts + GLIM submap pts (via on_update_submaps) + new localized pts
 *   all transformed into saved map world frame.
 */

#include <deque>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <cstdio>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <random>
#include <cmath>
#include <limits>

#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

#include "glim_localizer/gps_utils.hpp"

using gtsam::symbol_shorthand::X;

namespace glim_localizer {

class LocalizerExt : public glim::ExtensionModuleROS2 {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizerExt()
  : logger(glim::create_module_logger("localizer_ext")),
    initialized_(false), map_loaded_(false),
    scan_count_(0), kill_switch_(false),
    has_glim_pose_(false), pending_init_(false)
  {
    // Default parameters
    map_path_              = "";
    output_path_           = "~/tmp/dump";
    map_frame_             = "map";
    lidar_frame_           = "livox_frame";
    vgicp_res_             = 1.0;
    prior_inf_scale_trans_ = 1e3;
    prior_inf_scale_rot_   = 1e2;
    mount_R_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Load config_localizer.json
    try {
      const glim::Config cfg(glim::GlobalConfig::get_config_path("config_localizer"));
      map_path_              = cfg.param<std::string>("localizer", "map_path",               map_path_);
      output_path_           = cfg.param<std::string>("localizer", "output_map_path",        output_path_);
      map_frame_             = cfg.param<std::string>("localizer", "map_frame",              map_frame_);
      lidar_frame_           = cfg.param<std::string>("localizer", "lidar_frame",            lidar_frame_);
      vgicp_res_             = cfg.param<double>     ("localizer", "vgicp_voxel_resolution", vgicp_res_);
      prior_inf_scale_trans_ = cfg.param<double>     ("localizer", "prior_inf_scale_trans",  prior_inf_scale_trans_);
      prior_inf_scale_rot_   = cfg.param<double>     ("localizer", "prior_inf_scale_rot",    prior_inf_scale_rot_);

      // Map-constraint fusion: inject VGICP results as priors on GLIM's X(i)
      // so GLIM optimizes against the saved map instead of running free.
      enable_map_constraint_ = cfg.param<bool>  ("localizer", "enable_map_constraint", false);
      map_prior_prec_trans_  = cfg.param<double>("localizer", "map_prior_prec_trans", 1e2);
      map_prior_prec_rot_    = cfg.param<double>("localizer", "map_prior_prec_rot",   1e2);

      // Local yaw search around the init heading (recovers small heading error)
      enable_yaw_search_    = cfg.param<bool>  ("localizer", "enable_yaw_search",    true);
      yaw_search_steps_     = cfg.param<int>   ("localizer", "yaw_search_steps",     9);
      yaw_search_range_deg_ = cfg.param<double>("localizer", "yaw_search_range_deg", 40.0);
      yaw_search_at_submap_ = cfg.param<int>   ("localizer", "yaw_search_at_submap", 3);
      yaw_search_pos_tol_   = cfg.param<double>("localizer", "yaw_search_pos_tol",   0.5);
      freeze_after_         = cfg.param<int>   ("localizer", "freeze_after",         5);
      post_freeze_correction_limit_ = cfg.param<double>("localizer", "post_freeze_correction_limit", 1.0);
      coarse_res_scale_     = cfg.param<double>("localizer", "coarse_res_scale",     2.5);

      // Live map update: replace saved-map voxels with freshly observed points
      // once the pose is trusted (after freeze), and persist on shutdown.
      enable_map_update_   = cfg.param<bool>  ("localizer", "enable_map_update",   true);
      map_update_voxel_    = cfg.param<double>("localizer", "map_update_voxel",    0.5);
      map_update_clear_radius_ = cfg.param<double>("localizer", "map_update_clear_radius", 15.0);
      map_update_rate_     = cfg.param<double>("localizer", "map_update_rate",       0.05);
      map_update_min_weight_ = cfg.param<double>("localizer", "map_update_min_weight", 0.1);
      map_update_min_submap_pts_ = cfg.param<int>("localizer", "map_update_min_submap_pts", 500);
      // Per-voxel point cap. The saved/global map accumulates points across all
      // submap passes (no cap), so to approximate that density use a large value
      // here, well above sub_mapping's per-submap max_num_points_in_voxel.
      map_update_max_points_ = cfg.param<int>   ("localizer", "map_update_max_points", 300);
      updated_map_path_    = cfg.param<std::string>("localizer", "updated_map_path", "");
      pose_max_lag_sec_    = cfg.param<double>("localizer", "pose_max_lag_sec", 1.0);

      // GPS fallback / init. lever_arm is GPS antenna position in the LiDAR frame.
      enable_gps_           = cfg.param<bool>  ("localizer", "enable_gps",            false);
      gps_topic_            = cfg.param<std::string>("localizer", "gps_topic",        "/gps/fix");
      gps_init_pose_        = cfg.param<bool>  ("localizer", "gps_init_pose",         true);
      gps_fallback_         = cfg.param<bool>  ("localizer", "gps_fallback",          true);
      gps_fail_err_         = cfg.param<double>("localizer", "gps_fail_vgicp_error",  5000.0);
      gps_fail_jump_        = cfg.param<double>("localizer", "gps_fail_jump",         3.0);
      gps_fresh_sec_        = cfg.param<double>("localizer", "gps_fresh_sec",         2.0);
      gps_heading_min_move_ = cfg.param<double>("localizer", "gps_heading_min_move",  0.5);

      // Default initial pose (used when no /initial_pose topic received)
      default_init_x_   = cfg.param<double>("localizer", "default_init_pose_x",       0.0);
      default_init_y_   = cfg.param<double>("localizer", "default_init_pose_y",       0.0);
      default_init_z_   = cfg.param<double>("localizer", "default_init_pose_z",       0.0);
      default_init_yaw_ = cfg.param<double>("localizer", "default_init_pose_yaw_deg", 0.0);
      has_default_init_ = true;
      const double roll  = cfg.param<double>("localizer", "mount_roll_deg",  0.0);
      const double pitch = cfg.param<double>("localizer", "mount_pitch_deg", 0.0);
      const double yaw   = cfg.param<double>("localizer", "mount_yaw_deg",   0.0);
      mount_R_ = (Eigen::AngleAxisd(yaw   * M_PI / 180.0, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll  * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
      logger->info("config_localizer.json loaded: map_path={}", map_path_);
      logger->info("freeze_after={} submaps, yaw_search={}", freeze_after_,
        enable_yaw_search_ ? "on" : "off");
    } catch (...) {
      logger->warn("config_localizer.json not found, using defaults");
    }

    // IMU topic: read from config_ros (glim_ros/imu_topic), the same key GLIM
    // itself uses, so the live-clock subscription matches the actual IMU stream.
    try {
      const glim::Config cfg_ros(glim::GlobalConfig::get_config_path("config_ros"));
      imu_topic_ = cfg_ros.param<std::string>("glim_ros", "imu_topic", imu_topic_);
      logger->info("imu_topic from config_ros: {}", imu_topic_);
    } catch (...) {
      logger->warn("config_ros not found, imu_topic defaults to {}", imu_topic_);
    }

    // GPS lever arm: read from config_gnss_global (gnss/lever_arm_body), the same
    // key the GNSS module uses, so localizer and GNSS share one antenna offset.
    if (enable_gps_) {
      try {
        const glim::Config cfg_g(glim::GlobalConfig::get_config_path("config_gnss_global"));
        gps_lever_arm_ = cfg_g.param<Eigen::Vector3d>("gnss", "lever_arm_body", gps_lever_arm_);
        logger->info("gps lever_arm from config_gnss_global: [{:.3f}, {:.3f}, {:.3f}]",
          gps_lever_arm_.x(), gps_lever_arm_.y(), gps_lever_arm_.z());
      } catch (...) {
        logger->warn("config_gnss_global not found, gps lever_arm stays [0,0,0]");
      }
    }

    // ROS parameter override (highest priority)
    // e.g. -p localizer_ext.map_path:=/path/to/map
    // glim_rosnode passes node parameters via rclcpp global param client
    // We read them from the ROS parameter server via environment/args
    // Simple approach: check for GLIM node's parameter in rclcpp context
    // glim_rosnode sets these as node params on the global node
    {
      // Try to get from rclcpp global context param override
      // GLIM passes -p localizer_ext.map_path as node param
      // We retrieve it by creating a temp node — but that crashes.
      // Instead, use environment variable set by wrapper or direct env read.
      // Best approach: read from /proc/self/cmdline
      std::ifstream cmdline("/proc/self/cmdline");
      std::string arg, prev_arg;
      while (std::getline(cmdline, arg, '\0')) {
        if (prev_arg == "-p" || prev_arg == "--param") {
          // arg format: "localizer_ext.map_path:=/some/path"
          const std::string key = "localizer_ext.map_path:=";
          if (arg.find(key) == 0) {
            map_path_ = arg.substr(key.size());
            logger->info("map_path from ROS param: {}", map_path_);
          }
        }
        prev_arg = arg;
      }
    }

    // Environment variable fallback
    if (map_path_.empty()) {
      const char* e = std::getenv("LOCALIZER_MAP_PATH");
      if (e) map_path_ = e;
    }

    // Register GLIM callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&LocalizerExt::on_insert_submap_cb, this, _1));
    glim::GlobalMappingCallbacks::on_smoother_update.add(
      std::bind(&LocalizerExt::on_smoother_update_cb, this, _1, _2, _3));
    glim::GlobalMappingCallbacks::on_update_submaps.add(
      std::bind(&LocalizerExt::on_update_submaps_cb, this, _1));

    localization_mode_ = !map_path_.empty();

    if (!localization_mode_) {
      // No saved map: behave exactly like vanilla GLIM. Force-disable the
      // map constraint regardless of config so nothing extra runs.
      enable_map_constraint_ = false;
      logger->warn("map_path not set — new-mapping mode (no localization, no map constraint)");
      return;
    }

    if (enable_map_constraint_)
      logger->info("map-constraint fusion ENABLED (prec trans={:.1f}, rot={:.1f})",
        map_prior_prec_trans_, map_prior_prec_rot_);

    backend_thread_ = std::thread([this] { backend_task(); });
    logger->info("Loading saved map in background: {}", map_path_);
  }

  ~LocalizerExt() override {
    kill_switch_ = true;
    if (backend_thread_.joinable()) backend_thread_.join();
    if (map_loaded_) save_updated_map();
    save_origin_sidecar();
    curr_pose_pub_.reset();
    traj_pub_.reset();
    map_pub_.reset();
    transform_pub_.reset();
  }

  // Write the recorded origin to {output_path}/gps_origin.json (mapping mode) or
  // {map_path}/gps_origin.json (localization mode) so the next localization run
  // can place itself in WGS84.
  void save_origin_sidecar() {
    if (!origin_gps_set_) return;
    std::string dir = localization_mode_ ? map_path_ : expand_user(output_path_);
    if (dir.empty()) return;
    const std::string path = dir + "/gps_origin.json";
    if (save_gps_origin(path, {origin_lat_, origin_lon_, origin_alt_}))
      logger->info("GPS origin saved: {}", path);
    else
      logger->warn("Failed to write GPS origin: {}", path);
  }

  static std::string expand_user(const std::string& p) {
    if (!p.empty() && p[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) return std::string(home) + p.substr(1);
    }
    return p;
  }

  virtual std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    logger->info("pose stall guard: pause if no /glim_ros/odom for >{:.1f}s", pose_max_lag_sec_);

    curr_pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/glim_ros/localized_curr_pose", rclcpp::QoS(1).best_effort());
    traj_pub_ = node.create_publisher<nav_msgs::msg::Path>(
      "/glim_ros/localized_trajectory", rclcpp::QoS(1).reliable());
    map_pub_  = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/glim_ros/entire_map", rclcpp::QoS(1).transient_local());
    transform_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localizer/T_glimworld_savedworld", rclcpp::QoS(1).transient_local());

    // 10 Hz — curr_pose with best_effort QoS for minimum latency
    pose_timer_ = node.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_curr_pose(); });

    // 1 Hz — full trajectory for save_map_glim.py rendering
    traj_timer_ = node.create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() { publish_trajectory(); });

    // dynamic-rate entire_map
    map_pub_timer_ = node.create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { map_pub_tick(); });

    auto sub = std::make_shared<glim::TopicSubscription<geometry_msgs::msg::PoseStamped>>(
      "/initial_pose",
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        on_initial_pose(msg);
      });

    // /glim_ros/odom: LiDAR-rate (~10 Hz) odometry in GLIM world frame
    // Used for real-time curr_pose — transforms into saved map frame when localization active
    auto odom_sub = std::make_shared<glim::TopicSubscription<nav_msgs::msg::Odometry>>(
      "/glim_ros/odom",
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
        on_odom(msg);
      });

    // IMU is the "live clock": it flows continuously from the sensor (or rosbag)
    // independent of GLIM's processing. Its latest stamp is treated as the
    // current time. Comparing it to the latest odom stamp tells how far GLIM has
    // fallen behind — works identically for live sensors and rosbag playback,
    // with no dependence on use_sim_time.
    auto imu_sub = std::make_shared<glim::TopicSubscription<sensor_msgs::msg::Imu>>(
      imu_topic_,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        latest_sensor_stamp_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());
      });

    std::vector<glim::GenericTopicSubscription::Ptr> subs = {sub, odom_sub, imu_sub};
    if (enable_gps_) {
      auto gps_sub = std::make_shared<glim::TopicSubscription<sensor_msgs::msg::NavSatFix>>(
        gps_topic_,
        [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
          on_gps(msg);
        });
      subs.push_back(gps_sub);
    }
    return subs;
  }

private:
  // -----------------------------------------------------------------------
  // GLIM callbacks
  // -----------------------------------------------------------------------

  void on_insert_submap_cb(const glim::SubMap::ConstPtr& submap) {
    // GLIM assigns X(i) by insertion order (i = submaps.size() at insert time),
    // not by submap->id. The insert-callback order matches that index, so we
    // track it here and map submap->id -> GLIM X index.
    const int glim_x_index = glim_insert_counter_++;
    {
      std::lock_guard<std::mutex> lock(x_index_mutex_);
      submap_to_x_index_[submap->id] = glim_x_index;
    }

    // Track latest GLIM pose without TF
    {
      std::lock_guard<std::mutex> lock(glim_pose_mutex_);
      latest_glim_pose_ = submap->T_world_origin;
      has_glim_pose_ = true;
    }

    // ── No-localization (new-mapping) mode ────────────────────────────────
    if (!localization_mode_) {
      const double s = submap->frames.empty() ? 0.0
          : submap->frames[submap->frames.size() / 2]->stamp;
      
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp    = (s > 0.0)
          ? rclcpp::Time(static_cast<uint64_t>(s * 1e9))
          : rclcpp::Clock().now();
      msg.header.frame_id = map_frame_;
      const Eigen::Quaterniond q(submap->T_world_origin.rotation());
      const auto& t = submap->T_world_origin.translation();
      msg.pose.position.x    = t.x(); msg.pose.position.y    = t.y();
      msg.pose.position.z    = t.z();
      msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
      {
        std::lock_guard<std::mutex> lk(last_pose_mutex_);
        last_pose_msg_ = msg;
        has_last_pose_ = true;
      }
      {
        std::lock_guard<std::mutex> tlk(traj_mutex_);
        trajectory_.poses.push_back(msg);
      }
      // Accumulate map points for /glim_ros/entire_map
      {
        std::lock_guard<std::mutex> lk(map_pts_mutex_);
        for (size_t j = 0; j < submap->frame->size(); j++) {
          const Eigen::Vector4d p = submap->T_world_origin * submap->frame->points[j];
          new_submap_pts_.push_back({(float)p[0], (float)p[1], (float)p[2]});
        }
      }
      if (!kill_switch_) publish_entire_map();
      return;
    }
    // ── End no-localization branch ─────────────────────────────────────────

    // If initial_pose arrived before any submap, recalculate T_savedmap_glimworld
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_init_) {
        T_savedmap_glimworld_ =
          T_pending_savedmap_lidar_ * submap->T_world_origin.inverse();
        initialized_ = true;
        pending_init_ = false;
        logger->info("T_sg recalculated with first submap glim=({:.2f},{:.2f},{:.2f})",
          submap->T_world_origin.translation().x(),
          submap->T_world_origin.translation().y(),
          submap->T_world_origin.translation().z());
      }
    }

    input_submap_queue_.push_back(submap);
  }

  void on_smoother_update_cb(
    gtsam_points::ISAM2Ext& isam2,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values)
  {
    const auto factors = output_factors_.get_all_and_clear();
    if (!factors.empty()) {
      logger->info("Injecting {} map-constraint priors into GLIM", factors.size());
      new_factors.add(factors);
    }
  }

  void on_update_submaps_cb(const std::vector<glim::SubMap::Ptr>& submaps) {
    // ── No-localization mode: rebuild map in GLIM world frame ─────────────
    if (!localization_mode_) {
      std::vector<Eigen::Vector3f> pts;
      pts.reserve(submaps.size() * 20000);
      for (const auto& sm : submaps) {
        if (!sm || !sm->frame || !sm->frame->has_points()) continue;
        for (size_t j = 0; j < sm->frame->size(); j++) {
          const Eigen::Vector4d p = sm->T_world_origin * sm->frame->points[j];
          pts.push_back({(float)p[0], (float)p[1], (float)p[2]});
        }
      }
      {
        std::lock_guard<std::mutex> lock(map_pts_mutex_);
        glim_submap_pts_ = std::move(pts);
        new_submap_pts_.clear();
      }
      if (!kill_switch_) publish_entire_map();
      return;
    }
    // ── End no-localization branch ─────────────────────────────────────────

    // Called after GLIM global optimization - rebuild GLIM pts in saved map frame
    Eigen::Isometry3d T_sg;
    { std::lock_guard<std::mutex> lock(mutex_); T_sg = T_savedmap_glimworld_; }
    if (!initialized_) return;

    // Find the most recently processed submap and update T_savedmap_glimworld.
    // This absorbs GLIM's global optimization (GPS factors, loop closure) moving
    // T_world_origin, so the live robot pose tracks the same corrections the GPS
    // path does. Before freeze we always apply it. After freeze we still apply
    // small corrections (steady drift) but reject large ones (a loop closure that
    // would swing the whole map), keeping the benefit without the snap.
    if (!submaps.empty() && last_processed_submap_id_ >= 0) {
      for (const auto& sm : submaps) {
        if (sm && sm->id == last_processed_submap_id_) {
          const Eigen::Isometry3d T_new = last_T_savedmap_submap_ * sm->T_world_origin.inverse();
          std::lock_guard<std::mutex> lk(mutex_);
          const double jump = (T_new.translation() - T_savedmap_glimworld_.translation()).norm();
          if (!transform_frozen_ || jump < post_freeze_correction_limit_) {
            T_savedmap_glimworld_ = T_new;
            T_sg = T_savedmap_glimworld_;
          } else {
            logger->warn("Rejected large post-freeze correction ({:.2f}m > {:.2f}m)",
              jump, post_freeze_correction_limit_);
          }
          break;
        }
      }
    }

    std::vector<Eigen::Vector3f> glim_pts;
    glim_pts.reserve(submaps.size() * 20000);
    for (const auto& sm : submaps) {
      if (!sm || !sm->frame || !sm->frame->has_points()) continue;
      const Eigen::Isometry3d T_savedmap_origin = T_sg * sm->T_world_origin;
      for (size_t j = 0; j < sm->frame->size(); j++) {
        const Eigen::Vector4d p = T_savedmap_origin * sm->frame->points[j];
        glim_pts.push_back({(float)p[0], (float)p[1], (float)p[2]});
      }
    }

    {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      glim_submap_pts_ = std::move(glim_pts);
    }

    if (!kill_switch_) publish_entire_map();
  }

  // -----------------------------------------------------------------------
  // Initial pose handler
  // T_savedmap_glimworld = T_savedmap_lidar * T_glimworld_lidar^-1
  // mount_R_ is applied so it cancels with Rx(180) in submap T_world_origin
  // -----------------------------------------------------------------------
  void on_initial_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    if (!map_loaded_) { logger->warn("Map not loaded yet"); return; }

    Eigen::Isometry3d T_savedmap_lidar = Eigen::Isometry3d::Identity();
    T_savedmap_lidar.translation() = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    T_savedmap_lidar.linear() = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix()
      * mount_R_;

    Eigen::Isometry3d T_glimworld_lidar = Eigen::Isometry3d::Identity();
    bool got_glim_pose = false;
    {
      std::lock_guard<std::mutex> lock(glim_pose_mutex_);
      if (has_glim_pose_) {
        T_glimworld_lidar = latest_glim_pose_;
        got_glim_pose = true;
        logger->info("GLIM pose at click: ({:.2f}, {:.2f}, {:.2f})",
          T_glimworld_lidar.translation().x(),
          T_glimworld_lidar.translation().y(),
          T_glimworld_lidar.translation().z());
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (got_glim_pose) {
      T_savedmap_glimworld_ = T_savedmap_lidar * T_glimworld_lidar.inverse();
      initialized_ = true;
      pending_init_ = false;
      logger->info("Initial pose set immediately: ({:.2f},{:.2f},{:.2f})",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    } else {
      T_pending_savedmap_lidar_ = T_savedmap_lidar;
      pending_init_ = true;
      initialized_ = false;
      logger->info("Initial pose pending (waiting for first submap): ({:.2f},{:.2f},{:.2f})",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
  }

  // -----------------------------------------------------------------------
  // Odom callback — LiDAR rate (~10 Hz), updates curr_pose in real time
  // -----------------------------------------------------------------------
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    // T_glimworld_lidar from /glim_ros/odom (GLIM world frame)
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;
    Eigen::Isometry3d T_glimworld_lidar = Eigen::Isometry3d::Identity();
    T_glimworld_lidar.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);
    T_glimworld_lidar.linear() = Eigen::Quaterniond(
      ori.w, ori.x, ori.y, ori.z).toRotationMatrix();

    // Real-time guard: record this odom's sensor stamp. publish_curr_pose()
    // compares it to the latest IMU stamp (the live clock) to detect how far
    // GLIM has fallen behind. Both are ROS sensor stamps on the same timeline,
    // so this is identical for live sensors and rosbag playback.
    last_odom_stamp_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());

    Eigen::Isometry3d T_pose;
    if (localization_mode_) {
      // Transform into saved map frame: T_savedmap_lidar = T_savedmap_glimworld * T_glimworld_lidar
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initialized_) return;
      T_pose = T_savedmap_glimworld_ * T_glimworld_lidar;
    } else {
      // No localization: GLIM world frame IS the map frame
      T_pose = T_glimworld_lidar;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = msg->header.stamp;
    pose_msg.header.frame_id = map_frame_;
    const Eigen::Quaterniond q(T_pose.rotation());
    const auto& t = T_pose.translation();
    pose_msg.pose.position.x    = t.x();
    pose_msg.pose.position.y    = t.y();
    pose_msg.pose.position.z    = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Overwrite last_pose_msg_ — 10Hz timer publishes it (gated by odom freshness).
    {
      std::lock_guard<std::mutex> lock(last_pose_mutex_);
      last_pose_msg_ = pose_msg;
      has_last_pose_ = true;
    }
  }

  // -----------------------------------------------------------------------
  // GPS callback.
  //  Localization mode: convert WGS84 to the saved-map ENU frame using the
  //    map's stored origin, giving a position directly in saved-map coords.
  //  Mapping mode: record the first fix as this map's origin so it can be
  //    written to gps_origin.json and reused for localization later.
  // -----------------------------------------------------------------------
  void on_gps(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
    if (msg->status.status < 0) return;            // no fix
    const double alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;

    if (!origin_gps_set_) {
      if (localization_mode_) return;              // map had no origin -> can't use GPS
      // Mapping mode: first valid fix defines the origin.
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      origin_alt_ = alt;
      origin_ecef_ = wgs84_to_ecef(origin_lat_, origin_lon_, origin_alt_);
      R_enu_ecef_  = enu_rotation(origin_lat_, origin_lon_);
      origin_gps_set_ = true;
      logger->info("Map origin GPS recorded: lat={:.7f} lon={:.7f} alt={:.2f}",
        origin_lat_, origin_lon_, origin_alt_);
    }

    const Eigen::Vector3d ecef = wgs84_to_ecef(msg->latitude, msg->longitude, alt);
    const Eigen::Vector3d enu = R_enu_ecef_ * (ecef - origin_ecef_);

    std::lock_guard<std::mutex> lock(gps_mutex_);
    // Heading from motion: once the antenna has moved more than gps_heading_min_move_
    // since the last heading update, take the XY travel direction as yaw. While
    // (nearly) stationary, keep the previous heading to avoid noise.
    if (gps_heading_valid_) {
      const double dx = enu.x() - gps_heading_ref_.x();
      const double dy = enu.y() - gps_heading_ref_.y();
      if (std::hypot(dx, dy) >= gps_heading_min_move_) {
        gps_heading_ = std::atan2(dy, dx);
        gps_heading_ref_ = enu;
        gps_heading_set_ = true;
      }
    } else {
      gps_heading_ref_ = enu;
      gps_heading_valid_ = true;
    }

    gps_enu_ = enu;
    gps_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
    has_gps_ = true;
  }

  bool gps_is_fresh() {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    if (!has_gps_) return false;
    const int64_t imu_ns = latest_sensor_stamp_ns_.load();
    if (imu_ns == 0) return false;
    // Compare against the live sensor clock (IMU), same timeline as the GPS stamp.
    return (imu_ns - gps_stamp_ns_) * 1e-9 <= gps_fresh_sec_;
  }

  // -----------------------------------------------------------------------
  // Backend thread: load map, then process submaps
  // -----------------------------------------------------------------------
  void backend_task() {
    try {
      load_map();
      map_loaded_ = true;
      logger->info("Map ready. Set /initial_pose to start localization.");
    } catch (const std::exception& e) {
      logger->error("load_map failed: {}", e.what());
      return;
    }

    last_map_pub_time_ = std::chrono::steady_clock::now();

    while (!kill_switch_) {
      const auto submaps = input_submap_queue_.get_all_and_clear();

      // Apply initial pose once map is ready and no pose has been set yet.
      // Priority: GPS position (if enabled, map has origin, fix is fresh) then
      // the config default. Heading comes from default_init_yaw_; the yaw search
      // refines it afterward.
      if (!initialized_ && has_default_init_) {
        bool glim_ready = false;
        Eigen::Isometry3d glim_pose;
        {
          std::lock_guard<std::mutex> lock(glim_pose_mutex_);
          glim_ready = has_glim_pose_;
          glim_pose = latest_glim_pose_;
        }

        bool use_gps_init = false;
        Eigen::Vector3d gps_xyz = Eigen::Vector3d::Zero();
        if (glim_ready && enable_gps_ && gps_init_pose_ && origin_gps_set_ && gps_is_fresh()) {
          std::lock_guard<std::mutex> glock(gps_mutex_);
          gps_xyz = gps_enu_;
          use_gps_init = true;
        }

        if (glim_ready) {
          std::lock_guard<std::mutex> lk(mutex_);
          if (!initialized_) {
            const double yaw = default_init_yaw_ * M_PI / 180.0;
            const Eigen::Matrix3d R_yaw =
              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            Eigen::Isometry3d T_savedmap_lidar = Eigen::Isometry3d::Identity();
            if (use_gps_init) {
              // GPS gives the antenna position; subtract the lever arm rotated
              // into the saved-map frame to get the LiDAR position.
              T_savedmap_lidar.translation() = gps_xyz - R_yaw * gps_lever_arm_;
            } else {
              T_savedmap_lidar.translation() = Eigen::Vector3d(
                default_init_x_, default_init_y_, default_init_z_);
            }
            T_savedmap_lidar.linear() = R_yaw * mount_R_;
            T_savedmap_glimworld_ = T_savedmap_lidar * glim_pose.inverse();
            initialized_ = true;
            const auto& tt = T_savedmap_lidar.translation();
            logger->info("Initial pose applied [{}]: ({:.2f},{:.2f},{:.2f}, yaw={:.1f}deg)",
              use_gps_init ? "GPS" : "default", tt.x(), tt.y(), tt.z(), default_init_yaw_);
          }
        }
      }

      // map publishing is now handled by map_pub_timer_ (dynamic rate)
      // pose publishing is now handled by pose_timer_ (5 Hz)

      if (submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (!initialized_) continue;
      for (const auto& sm : submaps) process_submap(sm);
    }
  }

  // -----------------------------------------------------------------------
  // Submap processing
  // -----------------------------------------------------------------------
  void process_submap(const glim::SubMap::ConstPtr& submap) {
    if (!submap || !submap->frame || !submap->frame->has_points()) return;

    Eigen::Isometry3d T_sg;
    { std::lock_guard<std::mutex> lock(mutex_); T_sg = T_savedmap_glimworld_; }

    // Transform submap points from sensor local to GLIM world frame
    auto cloud = submap_to_glimworld(*submap);

    Eigen::Isometry3d T_result;
    double vgicp_err = 0.0;

    if (transform_frozen_) {
      // After the freeze point, T_savedmap_glimworld is locked. We no longer run
      // VGICP (which occasionally jumped and swung the whole map). The robot
      // pose still tracks via GLIM odometry through the frozen transform, and
      // priors keep being injected below.
      std::lock_guard<std::mutex> lock(mutex_);
      T_result = T_savedmap_glimworld_;
    } else if (enable_yaw_search_ && !yaw_search_done_) {
      // Wait for the Nth submap (by then the robot has moved a little, so the
      // submap is richer), then run the yaw search on that single submap using
      // the same path that works with a precise init yaw. No accumulation.
      yaw_accum_count_++;
      if (yaw_accum_count_ < yaw_search_at_submap_)
        return;  // skip early sparse submaps; nothing published yet

      logger->info("Running yaw search on {} points (submap id={})...",
        cloud->size(), submap->id);

      // Reference: the OFF-path result (direct fine fit from init). The dyaw=0
      // candidate inside the search should match this; if it doesn't, the bug
      // is in the seed construction, not the search selection.
      double ref_err;
      Eigen::Isometry3d ref = vgicp_world_scored(cloud, T_sg, 1e9, ref_err);
      auto yaw_of = [](const Eigen::Isometry3d& T) {
        return std::atan2(T.linear()(1, 0), T.linear()(0, 0)) * 180.0 / M_PI;
      };
      logger->info("REF (direct fit from init): yaw={:.1f} pos=({:.2f},{:.2f},{:.2f}) err={:.1f}",
        yaw_of(ref), ref.translation().x(), ref.translation().y(), ref.translation().z(), ref_err);

      T_result = vgicp_world_global_yaw(cloud, T_sg);
      logger->info("SEARCH result: yaw={:.1f} pos=({:.2f},{:.2f},{:.2f})",
        yaw_of(T_result), T_result.translation().x(), T_result.translation().y(), T_result.translation().z());
      yaw_search_done_ = true;
    } else {
      const double jump_limit = (scan_count_ < 5) ? 30.0 : 5.0;
      T_result = vgicp_world_scored(cloud, T_sg, jump_limit, vgicp_err);
    }

    // ── Failure detection + GPS fallback ──────────────────────────────────
    // Two signals: (a) VGICP error too high or a rejected jump (pre-freeze),
    // (b) after freeze, the tracked pose drifting far from GPS (divergence).
    if (enable_gps_ && gps_fallback_ && localization_mode_ && origin_gps_set_ &&
        scan_count_ >= 2) {
      const Eigen::Isometry3d T_try = T_result * submap->T_world_origin;

      bool failed = false;
      if (!transform_frozen_) {
        if (vgicp_err > gps_fail_err_) failed = true;
      }
      // GPS consistency check (works both before and after freeze)
      bool gps_ok = gps_is_fresh();
      Eigen::Vector3d g_enu;
      if (gps_ok) {
        std::lock_guard<std::mutex> glock(gps_mutex_);
        g_enu = gps_enu_;
      }
      if (gps_ok) {
        const Eigen::Vector3d antenna =
          T_try.translation() + T_try.linear() * gps_lever_arm_;
        const double dxy = (antenna.head<2>() - g_enu.head<2>()).norm();
        if (dxy > gps_fail_jump_) failed = true;
      }

      if (failed) {
        if (gps_ok) {
          // Keep rotation from VGICP, override xyz from GPS (minus lever arm).
          Eigen::Isometry3d T_fix = T_try;
          T_fix.translation() = g_enu - T_try.linear() * gps_lever_arm_;
          // Back out to T_savedmap_glimworld: T_sg = T_fix * submap^-1
          T_result = T_fix * submap->T_world_origin.inverse();
          logger->warn("Localization failed (err={:.0f}) — GPS fallback xyz=({:.2f},{:.2f},{:.2f})",
            vgicp_err, g_enu.x(), g_enu.y(), g_enu.z());
          localization_failed_.store(false);   // GPS gave a usable pose
        } else {
          // No GPS: pause mapping for this submap and retry on the next one.
          logger->warn("Localization failed (err={:.0f}) and no fresh GPS — pausing, will retry",
            vgicp_err);
          localization_failed_.store(true);    // stop publishing wrong pose
          return;                              // skip this submap entirely
        }
      } else {
        localization_failed_.store(false);
      }
    }

    { std::lock_guard<std::mutex> lock(mutex_); T_savedmap_glimworld_ = T_result; }

    // Freeze the transform once enough fixes have been made, so a later VGICP
    // jump can never swing the whole map again.
    if (!transform_frozen_ && scan_count_ + 1 >= freeze_after_) {
      transform_frozen_ = true;
      logger->info("T_savedmap_glimworld frozen after {} fixes", scan_count_ + 1);
    }

    // Save for on_update_submaps to correct after loop closure
    last_processed_submap_id_ = submap->id;
    last_T_savedmap_submap_ = T_result * submap->T_world_origin;

    // Robot position in saved map frame
    const Eigen::Isometry3d T_savedmap_submap = T_result * submap->T_world_origin;

    const double vgicp_delta = (T_result.translation() - T_sg.translation()).norm();
    // Pre-freeze, a ~0 delta means VGICP didn't move the estimate (possible bad
    // init). Post-freeze, VGICP isn't run at all, so ~0 is expected and fine.
    const bool vgicp_stalled = (vgicp_delta < 1e-4) && !transform_frozen_;
    const char* status = transform_frozen_ ? " [FROZEN]"
                       : (vgicp_stalled ? " [STALLED — check init pose]" : "");
    logger->info("#{} [{}]: ({:.2f}, {:.2f}, {:.2f}) | vgicp_delta={:.4f}m{}",
      scan_count_ + 1,
      scan_count_ < 5 ? "INIT" : "LOCALIZED",
      T_savedmap_submap.translation().x(),
      T_savedmap_submap.translation().y(),
      T_savedmap_submap.translation().z(),
      vgicp_delta,
      status);

    // Map-constraint fusion: inject T_savedmap_submap as a prior on GLIM's
    // X(glim_index) so the global optimizer aligns GLIM to the saved map.
    // Skipped when the VGICP result is unreliable (stalled/rejected) or while
    // the heading is still being resolved by the yaw search.
    const bool yaw_ready = !enable_yaw_search_ || yaw_search_done_;
    // vgicp_stalled is already false when frozen (delta ~0 is expected there),
    // so priors keep flowing after the freeze.
    const bool prior_ok = !vgicp_stalled;
    if (localization_mode_ && enable_map_constraint_ && yaw_ready &&
        prior_ok && scan_count_ >= 2) {
      int glim_x_index = -1;
      {
        std::lock_guard<std::mutex> lock(x_index_mutex_);
        auto it = submap_to_x_index_.find(submap->id);
        if (it != submap_to_x_index_.end()) glim_x_index = it->second;
      }
      if (glim_x_index >= 0) {
        gtsam::Vector6 prec;
        prec << map_prior_prec_rot_, map_prior_prec_rot_, map_prior_prec_rot_,
                map_prior_prec_trans_, map_prior_prec_trans_, map_prior_prec_trans_;
        auto noise = gtsam::noiseModel::Diagonal::Precisions(prec);
        output_factors_.push_back(
          gtsam::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            X(glim_x_index), gtsam::Pose3(T_savedmap_submap.matrix()), noise));
        logger->debug("Queued map prior on X({}) for submap id={}", glim_x_index, submap->id);
      } else {
        logger->warn("No GLIM X index for submap id={}, prior skipped", submap->id);
      }
    }

    // Viewer update handled by init_pose_viewer in GUI mode
    // Publish T_glimworld_savedworld for viewer update
    if (transform_pub_ && !kill_switch_) {
      const Eigen::Isometry3d T_gw = T_result.inverse();
      geometry_msgs::msg::PoseStamped tmsg;
      tmsg.header.frame_id = "glim_world";
      const Eigen::Quaterniond tq(T_gw.rotation());
      tmsg.pose.position.x = T_gw.translation().x();
      tmsg.pose.position.y = T_gw.translation().y();
      tmsg.pose.position.z = T_gw.translation().z();
      tmsg.pose.orientation.x = tq.x();
      tmsg.pose.orientation.y = tq.y();
      tmsg.pose.orientation.z = tq.z();
      tmsg.pose.orientation.w = tq.w();
      transform_pub_->publish(tmsg);
    }
    if (!kill_switch_) {
      const double stamp_sec = submap->frames.empty()
        ? 0.0 : submap->frames[submap->frames.size() / 2]->stamp;
      if (stamp_sec > 0) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(stamp_sec * 1e9));
        msg.header.frame_id = map_frame_;
        const Eigen::Quaterniond q(T_savedmap_submap.rotation());
        msg.pose.position.x = T_savedmap_submap.translation().x();
        msg.pose.position.y = T_savedmap_submap.translation().y();
        msg.pose.position.z = T_savedmap_submap.translation().z();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        // Store for 10 Hz republish + append to trajectory
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        last_pose_msg_ = msg;
        has_last_pose_ = true;
        std::lock_guard<std::mutex> tlk(traj_mutex_);
        trajectory_.poses.push_back(msg);
      }
    }

    // Map update: once the transform is frozen (pose trusted), this submap's
    // observation is authoritative around the robot. Within map_update_clear_radius
    // of the robot, every voxel that the new scan does NOT occupy is cleared
    // (observed empty / unknown-no-more), and occupied voxels are refilled with
    // the fresh points. Voxels outside the radius are left untouched (unknown).
    // Before freeze the pose is still settling, so we only accumulate for view.
    //
    // Guards: skip the update when the pose isn't fresh (odom lagging / stalled —
    // a recovering LiDAR can deliver a sparse, mistimed submap) or when the
    // submap is too sparse to trust as an "empty space" observation. Carving on a
    // bad submap would wrongly empty the map.
    const size_t submap_pts = submap->frame->size();
    const bool pose_trusted = enable_map_update_ && transform_frozen_
                              && pose_fresh_.load()
                              && (int)submap_pts >= map_update_min_submap_pts_;

    std::vector<Eigen::Vector3f> obs_pts;
    obs_pts.reserve(submap_pts);
    for (size_t j = 0; j < submap->frame->size(); j++) {
      const Eigen::Vector4d p = T_savedmap_submap * submap->frame->points[j];
      obs_pts.push_back({(float)p[0], (float)p[1], (float)p[2]});
    }

    if (pose_trusted) {
      // Group new observation points by voxel.
      std::unordered_map<int64_t, std::vector<Eigen::Vector3f>> occupied;
      for (const auto& p : obs_pts) {
        const int64_t k = voxel_key(p[0], p[1], p[2], map_update_voxel_);
        occupied[k].push_back(p);
      }

      const double rx = T_savedmap_submap.translation().x();
      const double ry = T_savedmap_submap.translation().y();
      const double r2 = map_update_clear_radius_ * map_update_clear_radius_;
      const double leaf = map_update_voxel_;
      const float  a = (float)map_update_rate_;   // learning rate

      std::lock_guard<std::mutex> lock(output_voxels_mutex_);

      // Decay cells inside the radius that the new scan did NOT hit (observed
      // empty). weight <- (1-a)*weight; drop when it falls below the threshold.
      for (auto it = output_voxels_.begin(); it != output_voxels_.end(); ) {
        if (occupied.find(it->first) != occupied.end()) { ++it; continue; }
        float cx, cy, cz;
        voxel_center(it->first, leaf, cx, cy, cz);
        const double dx = cx - rx, dy = cy - ry;
        if (dx * dx + dy * dy <= r2) {
          it->second.weight *= (1.0f - a);
          if (it->second.weight < (float)map_update_min_weight_) {
            it = output_voxels_.erase(it);   // carved away gradually
            continue;
          }
        }
        ++it;
      }

      // Reinforce hit cells: weight <- (1-a)*weight + a*1, and blend in the new
      // points (cap per voxel, newest win) to keep density GLIM-like.
      for (auto& kv : occupied) {
        auto& cell = output_voxels_[kv.first];
        cell.weight = (1.0f - a) * cell.weight + a;
        // Prepend new points, then trim to the cap so fresh geometry dominates.
        auto& dst = cell.points;
        std::vector<Eigen::Vector3f> merged;
        merged.reserve(map_update_max_points_);
        for (const auto& p : kv.second) {
          if ((int)merged.size() >= map_update_max_points_) break;
          merged.push_back(p);
        }
        for (const auto& p : dst) {
          if ((int)merged.size() >= map_update_max_points_) break;
          merged.push_back(p);
        }
        dst.swap(merged);
      }
    }

    // Visualization / publish. After freeze the persistent geometry lives in
    // output_voxels_ (already updated), so publish reflects new points and the
    // cleared empty space. Before freeze, accumulate into new_submap_pts_.
    if (!pose_trusted) {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      new_submap_pts_.insert(new_submap_pts_.end(), obs_pts.begin(), obs_pts.end());
    } else {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      new_submap_pts_.clear();
    }
    if (!kill_switch_) publish_entire_map();

    scan_count_++;
  }

  // -----------------------------------------------------------------------
  // Map publishing
  // -----------------------------------------------------------------------

  // Called at 5 Hz by map_pub_timer_.
  // Decides publish interval based on mode and total point count,
  // so large saved maps don't flood the network.
  void map_pub_tick() {
    if (kill_switch_) return;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_map_pub_time_).count();

    double interval;
    if (!localization_mode_) {
      interval = 1.0;   // new-mapping: 1 Hz — map grows in real time
    } else {
      size_t total_pts = 0;
      {
        std::lock_guard<std::mutex> lock(map_pts_mutex_);
        total_pts = saved_map_pts_.size() + glim_submap_pts_.size() + new_submap_pts_.size();
      }
      if      (total_pts < 2000000)  interval = 2.0;   // <  2M pts → 0.5 Hz
      else if (total_pts < 10000000) interval = 5.0;   // < 10M pts → 0.2 Hz
      else                           interval = 10.0;  // ≥ 10M pts → 0.1 Hz
    }

    if (elapsed >= interval) {
      publish_entire_map();
      last_map_pub_time_ = now;
    }
  }

  // 10 Hz: republish last known pose with best_effort QoS for minimum latency.
  // In localization mode, pauses while odom has stalled (no fresh /glim_ros/odom
  // within pose_max_lag_sec_) so downstream never sees an outdated pose. The gap
  // is measured on the wall clock and is independent of use_sim_time. Resumes
  // automatically as soon as odom starts arriving again.
  void publish_curr_pose() {
    if (!curr_pose_pub_) return;

    if (localization_mode_) {
      if (localization_failed_.load()) return;   // failed, no GPS — paused
      const int64_t imu_ns  = latest_sensor_stamp_ns_.load();
      const int64_t odom_ns = last_odom_stamp_ns_.load();
      if (imu_ns == 0 || odom_ns == 0) return;    // not streaming yet
      // How far the latest odom lags behind the live sensor clock.
      const double behind = (imu_ns - odom_ns) * 1e-9;
      if (behind > pose_max_lag_sec_) {
        if (pose_fresh_.exchange(false))
          logger->warn("Odom {:.2f}s behind live sensor > {:.2f}s — pausing pose output",
            behind, pose_max_lag_sec_);
        // Odom stalled (e.g. LiDAR frozen). Submaps stop too, so the submap-side
        // fallback can't fire. Publish a GPS-only pose here if GPS is fresh:
        // position from GPS ENU, heading from GPS motion direction.
        publish_gps_only_pose();
        return;
      } else if (!pose_fresh_.exchange(true)) {
        logger->info("Odom caught up ({:.2f}s behind) — resuming pose output", behind);
      }
    }

    std::lock_guard<std::mutex> lock(last_pose_mutex_);
    if (!has_last_pose_) return;
    curr_pose_pub_->publish(last_pose_msg_);
  }

  // Publish a pose derived purely from GPS when odom has stalled. No effect if
  // GPS is disabled, has no origin, isn't fresh, or has no heading yet.
  void publish_gps_only_pose() {
    if (!enable_gps_ || !gps_fallback_ || !origin_gps_set_) return;
    if (!gps_is_fresh()) return;

    Eigen::Vector3d enu;
    double yaw;
    bool have_yaw;
    {
      std::lock_guard<std::mutex> lock(gps_mutex_);
      enu = gps_enu_;
      yaw = gps_heading_;
      have_yaw = gps_heading_set_;
    }
    if (!have_yaw) return;   // not moved enough yet to know heading

    const Eigen::Matrix3d R =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d lidar_xyz = enu - R * gps_lever_arm_;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = rclcpp::Time(latest_sensor_stamp_ns_.load());
    msg.pose.position.x = lidar_xyz.x();
    msg.pose.position.y = lidar_xyz.y();
    msg.pose.position.z = lidar_xyz.z();
    const Eigen::Quaterniond q(R);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    curr_pose_pub_->publish(msg);
  }

  // 1 Hz: publish full trajectory for save_map_glim.py
  void publish_trajectory() {
    if (!traj_pub_) return;
    nav_msgs::msg::Path path;
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      path = trajectory_;
    }
    if (path.poses.empty()) return;
    path.header.frame_id = map_frame_;
    traj_pub_->publish(path);
  }

  void publish_entire_map() {
    if (!map_pub_) return;

    std::vector<Eigen::Vector3f> all_pts;
    if (enable_map_update_ && localization_mode_) {
      // Persistent geometry is the voxel store (saved map with observed voxels
      // replaced and empty space cleared). Before freeze the voxels aren't being
      // updated yet, so also append new_submap_pts_ for a live view; after freeze
      // new observations are already folded into the voxels and new_submap_pts_
      // is empty.
      {
        std::lock_guard<std::mutex> lock(output_voxels_mutex_);
        size_t n = 0;
        for (const auto& kv : output_voxels_) n += kv.second.points.size();
        all_pts.reserve(n);
        for (const auto& kv : output_voxels_)
          all_pts.insert(all_pts.end(), kv.second.points.begin(), kv.second.points.end());
      }
      {
        std::lock_guard<std::mutex> lock(map_pts_mutex_);
        all_pts.insert(all_pts.end(), new_submap_pts_.begin(), new_submap_pts_.end());
      }
    } else {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      all_pts.reserve(saved_map_pts_.size() + glim_submap_pts_.size() + new_submap_pts_.size());
      all_pts.insert(all_pts.end(), saved_map_pts_.begin(),   saved_map_pts_.end());
      all_pts.insert(all_pts.end(), glim_submap_pts_.begin(), glim_submap_pts_.end());
      all_pts.insert(all_pts.end(), new_submap_pts_.begin(),  new_submap_pts_.end());
    }

    // 20% random sampling
    std::vector<Eigen::Vector3f> sampled;
    sampled.reserve(all_pts.size() / 5 + 1);
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    for (const auto& p : all_pts)
      if (dist(rng) < 0.2f) sampled.push_back(p);

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.height = 1;
    msg.width  = sampled.size();
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(sampled.size());

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    for (const auto& p : sampled) {
      *ix = p[0]; *iy = p[1]; *iz = p[2];
      ++ix; ++iy; ++iz;
    }
    map_pub_->publish(msg);
  }

  // -----------------------------------------------------------------------
  // VGICP helpers
  // -----------------------------------------------------------------------
  std::shared_ptr<gtsam_points::PointCloudCPU> submap_to_glimworld(const glim::SubMap& sm) {
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>();
    std::vector<Eigen::Vector4d> pts;
    pts.reserve(sm.frame->size());
    for (size_t j = 0; j < sm.frame->size(); j++)
      pts.push_back(sm.T_world_origin * sm.frame->points[j]);
    cloud->add_points(pts);
    add_isotropic_covs(*cloud, 0.01);
    return cloud;
  }

  // X(0) = T_savedmap_glimworld (optimized directly).
  // Returns the optimized pose and writes the final graph error into out_error
  // (lower = better fit). On reject/exception, returns init with a large error.
  Eigen::Isometry3d vgicp_world_scored(
    const std::shared_ptr<gtsam_points::PointCloudCPU>& cloud,
    const Eigen::Isometry3d& init,
    double jump_limit,
    double& out_error,
    const std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>& voxelmap = nullptr)
  {
    out_error = std::numeric_limits<double>::max();
    const auto& vmap = voxelmap ? voxelmap : map_voxelmap_;
    gtsam::NonlinearFactorGraph g;
    gtsam::Values v;
    v.insert(X(0), gtsam::Pose3(init.matrix()));
    g.add(gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
      gtsam::Pose3::Identity(), X(0),
      std::static_pointer_cast<const gtsam_points::GaussianVoxelMap>(vmap),
      cloud));
    gtsam::LevenbergMarquardtParams lm;
    lm.maxIterations = 50;
    lm.relativeErrorTol = 1e-4;
    lm.absoluteErrorTol = 1e-5;
    try {
      auto r = gtsam::LevenbergMarquardtOptimizer(g, v, lm).optimize();
      auto opt = Eigen::Isometry3d(r.at<gtsam::Pose3>(X(0)).matrix());
      const double delta = (opt.translation() - init.translation()).norm();
      if (delta > jump_limit) {
        logger->warn("VGICP jumped too far ({:.3f}m > {:.1f}m), rejected", delta, jump_limit);
        return init;
      }
      out_error = g.error(r);
      return opt;
    } catch (...) { return init; }
  }

  // X(0) = T_savedmap_glimworld (optimized directly)
  Eigen::Isometry3d vgicp_world(
    const std::shared_ptr<gtsam_points::PointCloudCPU>& cloud,
    const Eigen::Isometry3d& init,
    double jump_limit = 5.0)
  {
    double err;
    return vgicp_world_scored(cloud, init, jump_limit, err);
  }

  // Global yaw search: rotate the (rough) init pose about the saved-map Z axis
  // through several candidate yaw offsets, run VGICP from each, keep the lowest
  // error. Used only for the first alignment so a wrong starting heading can be
  // recovered instead of trusting the supplied yaw.
  Eigen::Isometry3d vgicp_world_global_yaw(
    const std::shared_ptr<gtsam_points::PointCloudCPU>& cloud,
    const Eigen::Isometry3d& init)
  {
    Eigen::Isometry3d best = init;
    double best_err = std::numeric_limits<double>::max();

    // Search a window centered on the init yaw, not the full circle. The seed
    // heading is roughly correct (a few tens of degrees off at most), so a
    // dense local sweep is both more accurate and avoids locking onto a wrong
    // far-off orientation.
    const int    n     = yaw_search_steps_ > 0 ? yaw_search_steps_ : 8;
    const double range = yaw_search_range_deg_ * M_PI / 180.0;
    const double step  = (n > 1) ? (2.0 * range / (n - 1)) : 0.0;

    // Pivot for the yaw rotation: the cloud centroid expressed in the saved-map
    // frame (init * centroid_glimworld). Rotating about this point keeps the
    // cloud in place and only changes its heading, instead of swinging it away.
    Eigen::Vector4d c = Eigen::Vector4d::Zero();
    for (size_t i = 0; i < cloud->size(); i++) c += cloud->points[i];
    if (cloud->size() > 0) c /= static_cast<double>(cloud->size());
    c[3] = 1.0;
    const Eigen::Vector3d pivot_pt = (init * c).head<3>();

    for (int k = 0; k < n; k++) {
      const double dyaw = -range + step * k;

      Eigen::Isometry3d Rz = Eigen::Isometry3d::Identity();
      Rz.linear() = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Eigen::Isometry3d pivot = Eigen::Isometry3d::Identity();
      pivot.translation() = pivot_pt;
      const Eigen::Isometry3d seed = pivot * Rz * pivot.inverse() * init;

      // Coarse pre-pass widens the convergence basin, then fine refines.
      double err_c;
      Eigen::Isometry3d coarse = vgicp_world_scored(cloud, seed, 1e9, err_c, map_voxelmap_coarse_);
      double err_f;
      Eigen::Isometry3d cand = vgicp_world_scored(cloud, coarse, 1e9, err_f);

      auto yaw_of = [](const Eigen::Isometry3d& T) {
        return std::atan2(T.linear()(1, 0), T.linear()(0, 0)) * 180.0 / M_PI;
      };

      // The init translation is trusted (only yaw is uncertain). A candidate
      // whose result drifts far from the init position has locked onto a wrong,
      // repeated structure elsewhere in the map - reject it even if its error is
      // low, since a low error far from init is a false match in a small room.
      const double res_from_init = (cand.translation() - init.translation()).norm();
      const bool rejected = res_from_init > yaw_search_pos_tol_;

      logger->info("  yaw {:+.1f}deg | seed_yaw={:.1f} res_yaw={:.1f} | "
                   "res_from_init={:.3f} | coarse={:.1f} fine={:.1f}{}",
        dyaw * 180.0 / M_PI, yaw_of(seed), yaw_of(cand),
        res_from_init, err_c, err_f, rejected ? " [REJECTED far]" : "");

      if (!rejected && err_f < best_err) { best_err = err_f; best = cand; }
    }
    logger->info("Yaw search best fine error={:.1f}", best_err);
    return best;
  }

  // -----------------------------------------------------------------------
  // Load saved map (direct binary read, no GlobalMapping::load)
  // -----------------------------------------------------------------------
  void load_map() {
    std::vector<Eigen::Vector4d> all_pts;
    all_pts.reserve(20000000);

    // Load GPS origin sidecar if present. The updated-map header (if any) may
    // override this below; both describe the same origin.
    {
      GpsOrigin o;
      if (load_gps_origin(map_path_ + "/gps_origin.json", o)) {
        origin_lat_ = o.lat; origin_lon_ = o.lon; origin_alt_ = o.alt;
        origin_ecef_ = wgs84_to_ecef(o.lat, o.lon, o.alt);
        R_enu_ecef_  = enu_rotation(o.lat, o.lon);
        origin_gps_set_ = true;
        logger->info("GPS origin from sidecar: lat={:.7f} lon={:.7f} alt={:.2f}",
          o.lat, o.lon, o.alt);
      }
    }

    // Prefer a previously updated map (single binary point dump). It already
    // contains saved-map + replacements from earlier runs.
    const std::string upath = updated_map_file();
    bool loaded_updated = false;
    if (enable_map_update_ && !upath.empty()) {
      loaded_updated = load_updated_map(upath, all_pts);
      if (loaded_updated)
        logger->info("Loaded updated map: {} pts from {}", all_pts.size(), upath);
    }

    if (!loaded_updated) {
      std::ifstream ifs(map_path_ + "/graph.txt");
      if (!ifs) throw std::runtime_error("Cannot open " + map_path_ + "/graph.txt");
      int num_submaps = 0;
      std::string tok;
      ifs >> tok >> num_submaps;

      for (int i = 0; i < num_submaps; i++) {
        auto sm = glim::SubMap::load((boost::format("%s/%06d") % map_path_ % i).str());
        if (!sm || !sm->frame || !sm->frame->has_points()) continue;
        for (size_t j = 0; j < sm->frame->size(); j++)
          all_pts.push_back(sm->T_world_origin * sm->frame->points[j]);
        if (i % 200 == 0)
          logger->info("Loading {}/{} ({} pts)", i, num_submaps, all_pts.size());
      }
    }

    // Build voxelmap for VGICP
    auto merged = std::make_shared<gtsam_points::PointCloudCPU>();
    merged->add_points(all_pts);

    auto ds = voxel_downsample(*merged, vgicp_res_ * 2.0);
    add_isotropic_covs(*ds, 0.01);
    merged.reset();

    map_voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(vgicp_res_);
    map_voxelmap_->insert(*ds);

    // Coarse voxelmap for broad-phase registration: a larger voxel widens the
    // convergence basin so big initial yaw/position errors get pulled in before
    // the fine pass refines the result.
    {
      const double coarse_res = vgicp_res_ * coarse_res_scale_;
      auto full = std::make_shared<gtsam_points::PointCloudCPU>();
      full->add_points(all_pts);
      auto ds_c = voxel_downsample(*full, coarse_res);
      add_isotropic_covs(*ds_c, 0.01);
      map_voxelmap_coarse_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(coarse_res);
      map_voxelmap_coarse_->insert(*ds_c);
      logger->info("Coarse voxelmap: {} pts (res={:.2f})", ds_c->size(), coarse_res);
    }

    // Initialize the output voxel store from the loaded points so the saved
    // structure is present until new observations replace individual voxels.
    if (enable_map_update_) {
      std::lock_guard<std::mutex> lock(output_voxels_mutex_);
      output_voxels_.clear();
      for (const auto& p : all_pts) {
        const int64_t k = voxel_key((float)p[0], (float)p[1], (float)p[2], map_update_voxel_);
        auto& cell = output_voxels_[k];
        cell.weight = 1.0f;
        if ((int)cell.points.size() < map_update_max_points_)
          cell.points.push_back({(float)p[0], (float)p[1], (float)p[2]});
      }
      logger->info("Output voxel store initialized: {} voxels", output_voxels_.size());
    }

    // Store all points for entire_map publishing (original density)
    {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      saved_map_pts_.reserve(all_pts.size());
      for (const auto& p : all_pts)
        saved_map_pts_.push_back({(float)p[0], (float)p[1], (float)p[2]});
    }

    logger->info("Map loaded: {} pts, voxelmap {} pts (res={:.1f})",
      all_pts.size(), ds->size(), vgicp_res_);

    // Publish loaded map immediately (even before any submap arrives)
    if (!kill_switch_) publish_entire_map();
  }

  std::string updated_map_file() const {
    if (!updated_map_path_.empty()) return updated_map_path_;
    if (map_path_.empty()) return "";
    return map_path_ + "/updated_map.bin";
  }

  // Binary format (little-endian):
  //   char     magic[4]   = "GLMU"
  //   uint32   version    = 1
  //   uint64   point_count
  //   uint8    has_gps                 0 = origin GPS unset, 1 = set
  //   uint8    pad[7]
  //   double   origin_lat, origin_lon, origin_alt   (valid only if has_gps)
  //   double   reserved[4]                           (heading/ECEF, future use)
  //   float    x, y, z   * point_count
  // Only the origin GPS is stored; any voxel's absolute lat/lon is recoverable
  // from it, so per-voxel GPS is unnecessary.
  struct UpdatedMapHeader {
    char     magic[4];
    uint32_t version;
    uint64_t point_count;
    uint8_t  has_gps;
    uint8_t  pad[7];
    double   origin_lat;
    double   origin_lon;
    double   origin_alt;
    double   reserved[4];
  };

  bool load_updated_map(const std::string& path, std::vector<Eigen::Vector4d>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    UpdatedMapHeader h{};
    f.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (!f) return false;
    if (h.magic[0] != 'G' || h.magic[1] != 'L' || h.magic[2] != 'M' || h.magic[3] != 'U') {
      logger->warn("Updated map has bad magic, ignoring");
      return false;
    }
    if (h.version != 1) {
      logger->warn("Updated map version {} unsupported, ignoring", h.version);
      return false;
    }
    if (h.point_count == 0 || h.point_count > 200000000ULL) return false;

    if (h.has_gps) {
      origin_lat_ = h.origin_lat;
      origin_lon_ = h.origin_lon;
      origin_alt_ = h.origin_alt;
      origin_gps_set_ = true;
      origin_ecef_ = wgs84_to_ecef(origin_lat_, origin_lon_, origin_alt_);
      R_enu_ecef_ = enu_rotation(origin_lat_, origin_lon_);
      logger->info("Map origin GPS: lat={:.7f} lon={:.7f} alt={:.2f}",
        origin_lat_, origin_lon_, origin_alt_);
    }

    out.reserve(out.size() + h.point_count);
    for (uint64_t i = 0; i < h.point_count; i++) {
      float xyz[3];
      f.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
      if (!f) return i > 0;
      out.push_back(Eigen::Vector4d(xyz[0], xyz[1], xyz[2], 1.0));
    }
    return true;
  }

  void save_updated_map() {
    if (!enable_map_update_) return;
    const std::string path = updated_map_file();
    if (path.empty()) return;

    std::vector<Eigen::Vector3f> pts;
    {
      std::lock_guard<std::mutex> lock(output_voxels_mutex_);
      for (const auto& kv : output_voxels_)
        pts.insert(pts.end(), kv.second.points.begin(), kv.second.points.end());
    }

    UpdatedMapHeader h{};
    h.magic[0] = 'G'; h.magic[1] = 'L'; h.magic[2] = 'M'; h.magic[3] = 'U';
    h.version = 1;
    h.point_count = pts.size();
    h.has_gps = origin_gps_set_ ? 1 : 0;
    h.origin_lat = origin_lat_;
    h.origin_lon = origin_lon_;
    h.origin_alt = origin_alt_;

    const std::string tmp = path + ".tmp";
    std::ofstream f(tmp, std::ios::binary);
    if (!f) { logger->warn("Cannot write updated map: {}", path); return; }
    f.write(reinterpret_cast<const char*>(&h), sizeof(h));
    for (const auto& p : pts) {
      float xyz[3] = {p[0], p[1], p[2]};
      f.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
    }
    f.close();
    std::rename(tmp.c_str(), path.c_str());
    logger->info("Updated map saved: {} pts (has_gps={}) -> {}",
      h.point_count, (int)h.has_gps, path);
  }


  // -----------------------------------------------------------------------
  // Utility
  // -----------------------------------------------------------------------
  static std::shared_ptr<gtsam_points::PointCloudCPU> voxel_downsample(
    const gtsam_points::PointCloudCPU& cloud, double leaf)
  {
    std::unordered_map<size_t, bool> seen;
    seen.reserve(cloud.size());
    std::vector<Eigen::Vector4d> pts;
    pts.reserve(cloud.size() / 4);
    for (size_t i = 0; i < cloud.size(); i++) {
      const auto& p = cloud.points[i];
      int ix = (int)std::floor(p[0] / leaf);
      int iy = (int)std::floor(p[1] / leaf);
      int iz = (int)std::floor(p[2] / leaf);
      size_t key = ((size_t)(ix + 32768) * 65536 + (size_t)(iy + 32768)) * 65536
                 + (size_t)(iz + 32768);
      if (seen.insert({key, true}).second) pts.push_back(p);
    }
    auto out = std::make_shared<gtsam_points::PointCloudCPU>();
    out->add_points(pts);
    return out;
  }

  static void add_isotropic_covs(gtsam_points::PointCloudCPU& c, double s) {
    std::vector<Eigen::Matrix4d> covs(c.size(), Eigen::Matrix4d::Zero());
    for (auto& m : covs) m.topLeftCorner<3, 3>() = s * Eigen::Matrix3d::Identity();
    c.add_covs(covs);
  }

  // -----------------------------------------------------------------------
  // Members
  // -----------------------------------------------------------------------
  std::shared_ptr<spdlog::logger> logger;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_pub_;  // /glim_ros/localized_curr_pose
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             traj_pub_;       // /glim_ros/localized_trajectory
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transform_pub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;     // 10 Hz curr_pose republish
  rclcpp::TimerBase::SharedPtr traj_timer_;     //  1 Hz trajectory publish
  rclcpp::TimerBase::SharedPtr map_pub_timer_;  // dynamic-rate entire_map publish

  std::mutex traj_mutex_;
  nav_msgs::msg::Path trajectory_;

  std::string map_path_, output_path_, map_frame_, lidar_frame_;
  double vgicp_res_, prior_inf_scale_trans_, prior_inf_scale_rot_;
  Eigen::Matrix3d mount_R_;

  // Map-constraint fusion
  bool   enable_map_constraint_ = false;
  double map_prior_prec_trans_  = 1e2;
  double map_prior_prec_rot_    = 1e2;

  // Local yaw search
  bool   enable_yaw_search_    = true;
  int    yaw_search_steps_     = 9;
  double yaw_search_range_deg_ = 40.0;
  int    yaw_search_at_submap_ = 3;
  double yaw_search_pos_tol_   = 0.5;
  int    freeze_after_         = 5;
  bool   transform_frozen_     = false;
  double post_freeze_correction_limit_ = 1.0;  // max drift correction accepted after freeze (m)
  bool   yaw_search_done_      = false;
  int    yaw_accum_count_      = 0;

  // GLIM X(i) index tracking: X index is assigned by insert order, not submap->id
  std::atomic<int> glim_insert_counter_{0};
  std::mutex x_index_mutex_;
  std::unordered_map<int, int> submap_to_x_index_;

  // Default initial pose (from config_localizer.json, used in headless mode)
  bool has_default_init_ = false;
  double default_init_x_ = 0.0, default_init_y_ = 0.0;
  double default_init_z_ = 0.0, default_init_yaw_ = 0.0;

  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> map_voxelmap_;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> map_voxelmap_coarse_;
  double coarse_res_scale_ = 2.5;

  // Live map update: voxel-hashed point storage for the output map.
  // Live map update. The output map is a voxel hash where each cell holds points
  // plus an occupancy weight in [0,1]. Each trusted observation nudges the weight
  // toward 1 for cells it hits and toward 0 for empty cells in view (decay), with
  // a fixed learning rate. Cells below a threshold are dropped (slowly carved),
  // so empty space clears gradually instead of snapping to nothing. Saved-map
  // structure stays until repeatedly observed empty.
  bool   enable_map_update_ = true;
  double map_update_voxel_  = 0.5;
  double map_update_clear_radius_ = 15.0;
  double map_update_rate_   = 0.05;   // learning rate per observation
  double map_update_min_weight_ = 0.1;   // drop cell below this
  int    map_update_min_submap_pts_ = 500;  // skip update if submap sparser than this
  int    map_update_max_points_ = 300;   // points kept per voxel (approx global density)
  std::string updated_map_path_;
  std::mutex output_voxels_mutex_;
  struct VoxelCell {
    std::vector<Eigen::Vector3f> points;
    float weight = 1.0f;
  };
  std::unordered_map<int64_t, VoxelCell> output_voxels_;

  // Map origin in WGS84 (ENU origin). Stored once in the updated-map header;
  // every voxel's absolute lat/lon is recoverable from this, so per-voxel GPS
  // is not stored. Filled in when GPS integration lands.
  bool   origin_gps_set_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_enu_ecef_  = Eigen::Matrix3d::Identity();

  // GPS fallback / init
  bool   enable_gps_   = false;
  std::string gps_topic_ = "/gps/fix";
  bool   gps_init_pose_ = true;
  bool   gps_fallback_  = true;
  double gps_fail_err_  = 5000.0;
  double gps_fail_jump_ = 3.0;
  double gps_fresh_sec_ = 2.0;
  Eigen::Vector3d gps_lever_arm_ = Eigen::Vector3d::Zero();
  std::mutex gps_mutex_;
  Eigen::Vector3d gps_enu_ = Eigen::Vector3d::Zero();
  int64_t gps_stamp_ns_ = 0;
  bool   has_gps_ = false;
  Eigen::Vector3d gps_heading_ref_ = Eigen::Vector3d::Zero();
  double gps_heading_ = 0.0;
  bool   gps_heading_valid_ = false;   // ref point initialized
  bool   gps_heading_set_ = false;     // heading estimated at least once
  double gps_heading_min_move_ = 0.5;  // min XY move (m) to update heading

  static int64_t voxel_key(float x, float y, float z, double leaf) {
    const int64_t ix = (int64_t)std::floor(x / leaf) + 1048576;
    const int64_t iy = (int64_t)std::floor(y / leaf) + 1048576;
    const int64_t iz = (int64_t)std::floor(z / leaf) + 1048576;
    return (ix * 2097152LL + iy) * 2097152LL + iz;
  }

  static void voxel_center(int64_t key, double leaf, float& cx, float& cy, float& cz) {
    const int64_t iz = key % 2097152LL;
    key /= 2097152LL;
    const int64_t iy = key % 2097152LL;
    key /= 2097152LL;
    const int64_t ix = key;
    cx = (float)((ix - 1048576) * leaf + 0.5 * leaf);
    cy = (float)((iy - 1048576) * leaf + 0.5 * leaf);
    cz = (float)((iz - 1048576) * leaf + 0.5 * leaf);
  }

  // Entire map point storage (all in saved map world frame)
  std::mutex map_pts_mutex_;
  std::vector<Eigen::Vector3f> saved_map_pts_;   // loaded saved map
  std::vector<Eigen::Vector3f> glim_submap_pts_; // GLIM submaps after optimization
  std::vector<Eigen::Vector3f> new_submap_pts_;  // newly localized submaps

  glim::ConcurrentVector<glim::SubMap::ConstPtr> input_submap_queue_;
  glim::ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors_;

  std::atomic<bool> kill_switch_;
  std::atomic<bool> map_loaded_;
  std::thread backend_thread_;
  std::chrono::steady_clock::time_point last_map_pub_time_;

  std::mutex glim_pose_mutex_;
  Eigen::Isometry3d latest_glim_pose_ = Eigen::Isometry3d::Identity();
  bool has_glim_pose_;

  std::mutex mutex_;
  bool initialized_;
  int scan_count_;
  int last_processed_submap_id_ = -1;
  Eigen::Isometry3d last_T_savedmap_submap_ = Eigen::Isometry3d::Identity();

  std::mutex last_pose_mutex_;
  geometry_msgs::msg::PoseStamped last_pose_msg_;
  bool has_last_pose_ = false;
  std::atomic<bool> pose_fresh_{true};   // false when odom has stalled
  std::atomic<bool> localization_failed_{false};  // true when failed without GPS
  std::atomic<int64_t> latest_sensor_stamp_ns_{0};  // latest IMU stamp (live clock)
  std::atomic<int64_t> last_odom_stamp_ns_{0};      // latest processed odom stamp
  double pose_max_lag_sec_ = 1.0;
  std::string imu_topic_ = "/imu";
  bool pending_init_;
  Eigen::Isometry3d T_pending_savedmap_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_savedmap_glimworld_     = Eigen::Isometry3d::Identity();

  // true when map_path_ is set; false = new-mapping mode (GLIM world = map frame)
  bool localization_mode_ = false;
};

}  // namespace glim_localizer

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localizer::LocalizerExt();
}