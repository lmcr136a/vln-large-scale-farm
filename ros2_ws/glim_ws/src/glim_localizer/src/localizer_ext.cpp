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
#include <mutex>
#include <unordered_map>
#include <random>
#include <cmath>

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
    } catch (...) {
      logger->warn("config_localizer.json not found, using defaults");
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
      logger->warn("map_path not set — new-mapping mode (no localization)");
      return;
    }

    backend_thread_ = std::thread([this] { backend_task(); });
    logger->info("Loading saved map in background: {}", map_path_);
  }

  ~LocalizerExt() override {
    kill_switch_ = true;
    if (backend_thread_.joinable()) backend_thread_.join();
    curr_pose_pub_.reset();
    traj_pub_.reset();
    map_pub_.reset();
    transform_pub_.reset();
  }

  virtual std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
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

    return {sub, odom_sub};
  }

private:
  // -----------------------------------------------------------------------
  // GLIM callbacks
  // -----------------------------------------------------------------------

  void on_insert_submap_cb(const glim::SubMap::ConstPtr& submap) {
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
      logger->debug("Injecting {} localization priors into GLIM", factors.size());
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

    // Find the most recently processed submap and update T_savedmap_glimworld
    // This corrects drift caused by GLIM loop closure changing T_world_origin
    if (!submaps.empty() && last_processed_submap_id_ >= 0) {
      for (const auto& sm : submaps) {
        if (sm && sm->id == last_processed_submap_id_) {
          // T_savedmap_glimworld = T_savedmap_submap_result * T_glimworld_submap_new^-1
          std::lock_guard<std::mutex> lk(mutex_);
          T_savedmap_glimworld_ = last_T_savedmap_submap_ * sm->T_world_origin.inverse();
          T_sg = T_savedmap_glimworld_;
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

    // Overwrite last_pose_msg_ — 10Hz timer will publish immediately
    {
      std::lock_guard<std::mutex> lock(last_pose_mutex_);
      last_pose_msg_ = pose_msg;
      has_last_pose_ = true;
    }
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

      // Apply default pose once map is ready and no pose has been set yet
      if (!initialized_ && has_default_init_) {
        bool glim_ready = false;
        Eigen::Isometry3d glim_pose;
        {
          std::lock_guard<std::mutex> lock(glim_pose_mutex_);
          glim_ready = has_glim_pose_;
          glim_pose = latest_glim_pose_;
        }
        if (glim_ready) {
          std::lock_guard<std::mutex> lk(mutex_);
          if (!initialized_) {
            const double yaw = default_init_yaw_ * M_PI / 180.0;
            Eigen::Isometry3d T_savedmap_lidar = Eigen::Isometry3d::Identity();
            T_savedmap_lidar.translation() = Eigen::Vector3d(
              default_init_x_, default_init_y_, default_init_z_);
            T_savedmap_lidar.linear() =
              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * mount_R_;
            T_savedmap_glimworld_ = T_savedmap_lidar * glim_pose.inverse();
            initialized_ = true;
            logger->info("Default initial pose applied: ({:.2f},{:.2f},{:.2f}, yaw={:.1f}deg)",
              default_init_x_, default_init_y_, default_init_z_, default_init_yaw_);
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

    // VGICP: find T_savedmap_glimworld
    const Eigen::Isometry3d T_result = vgicp_world(cloud, T_sg);

    { std::lock_guard<std::mutex> lock(mutex_); T_savedmap_glimworld_ = T_result; }

    // Save for on_update_submaps to correct after loop closure
    last_processed_submap_id_ = submap->id;
    last_T_savedmap_submap_ = T_result * submap->T_world_origin;

    // Robot position in saved map frame
    const Eigen::Isometry3d T_savedmap_submap = T_result * submap->T_world_origin;

    if (scan_count_ < 5) {
      logger->info("#{} [LOCAL]:  ({:.2f}, {:.2f}, {:.2f})", scan_count_ + 1,
        T_savedmap_submap.translation().x(),
        T_savedmap_submap.translation().y(),
        T_savedmap_submap.translation().z());
    } else {
      logger->info("#{} [GLOBAL]: ({:.2f}, {:.2f}, {:.2f})", scan_count_ + 1,
        T_savedmap_submap.translation().x(),
        T_savedmap_submap.translation().y(),
        T_savedmap_submap.translation().z());
    }

    // Note: PriorFactor injection removed intentionally.
    // GLIM runs freely for its own odometry/loop closure.
    // T_savedmap_glimworld tracks the transform between frames independently.

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

    // Add new submap points to entire_map
    {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      for (size_t j = 0; j < submap->frame->size(); j++) {
        const Eigen::Vector4d p = T_savedmap_submap * submap->frame->points[j];
        new_submap_pts_.push_back({(float)p[0], (float)p[1], (float)p[2]});
      }
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

  // 10 Hz: republish last known pose with best_effort QoS for minimum latency
  void publish_curr_pose() {
    if (!curr_pose_pub_) return;
    std::lock_guard<std::mutex> lock(last_pose_mutex_);
    if (!has_last_pose_) return;
    curr_pose_pub_->publish(last_pose_msg_);
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
    {
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

  // X(0) = T_savedmap_glimworld (optimized directly)
  Eigen::Isometry3d vgicp_world(
    const std::shared_ptr<gtsam_points::PointCloudCPU>& cloud,
    const Eigen::Isometry3d& init)
  {
    gtsam::NonlinearFactorGraph g;
    gtsam::Values v;
    v.insert(X(0), gtsam::Pose3(init.matrix()));
    g.add(gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
      gtsam::Pose3::Identity(), X(0),
      std::static_pointer_cast<const gtsam_points::GaussianVoxelMap>(map_voxelmap_),
      cloud));
    gtsam::LevenbergMarquardtParams lm;
    lm.maxIterations = 50;
    lm.relativeErrorTol = 1e-4;
    lm.absoluteErrorTol = 1e-5;
    try {
      auto r = gtsam::LevenbergMarquardtOptimizer(g, v, lm).optimize();
      auto opt = Eigen::Isometry3d(r.at<gtsam::Pose3>(X(0)).matrix());
      if ((opt.translation() - init.translation()).norm() > 5.0) {
        logger->warn("VGICP jumped too far, rejected");
        return init;
      }
      return opt;
    } catch (...) { return init; }
  }

  // -----------------------------------------------------------------------
  // Load saved map (direct binary read, no GlobalMapping::load)
  // -----------------------------------------------------------------------
  void load_map() {
    std::ifstream ifs(map_path_ + "/graph.txt");
    if (!ifs) throw std::runtime_error("Cannot open " + map_path_ + "/graph.txt");
    int num_submaps = 0;
    std::string tok;
    ifs >> tok >> num_submaps;

    std::vector<Eigen::Vector4d> all_pts;
    all_pts.reserve(20000000);

    for (int i = 0; i < num_submaps; i++) {
      auto sm = glim::SubMap::load((boost::format("%s/%06d") % map_path_ % i).str());
      if (!sm || !sm->frame || !sm->frame->has_points()) continue;
      for (size_t j = 0; j < sm->frame->size(); j++)
        all_pts.push_back(sm->T_world_origin * sm->frame->points[j]);
      if (i % 200 == 0)
        logger->info("Loading {}/{} ({} pts)", i, num_submaps, all_pts.size());
    }

    // Build voxelmap for VGICP
    auto merged = std::make_shared<gtsam_points::PointCloudCPU>();
    merged->add_points(all_pts);

    auto ds = voxel_downsample(*merged, vgicp_res_ * 2.0);
    add_isotropic_covs(*ds, 0.01);
    merged.reset();

    map_voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(vgicp_res_);
    map_voxelmap_->insert(*ds);

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

  // Default initial pose (from config_localizer.json, used in headless mode)
  bool has_default_init_ = false;
  double default_init_x_ = 0.0, default_init_y_ = 0.0;
  double default_init_z_ = 0.0, default_init_yaw_ = 0.0;

  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> map_voxelmap_;

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