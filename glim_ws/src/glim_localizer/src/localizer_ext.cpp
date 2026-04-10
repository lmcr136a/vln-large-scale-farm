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

#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
    output_path_           = "/tmp/dump";
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
      const double roll  = cfg.param<double>("localizer", "mount_roll_deg",  180.0);
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

    if (map_path_.empty()) {
      logger->warn("map_path not set, localization disabled");
      return;
    }

    backend_thread_ = std::thread([this] { backend_task(); });
    logger->info("Loading saved map in background: {}", map_path_);
  }

  ~LocalizerExt() override {
    kill_switch_ = true;
    if (backend_thread_.joinable()) backend_thread_.join();
    pose_pub_.reset();
    map_pub_.reset();
  }

  virtual std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/localized_pose", 10);
    map_pub_  = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/glim_ros/entire_map", rclcpp::QoS(1).transient_local());

    auto sub = std::make_shared<glim::TopicSubscription<geometry_msgs::msg::PoseStamped>>(
      "/initial_pose",
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        on_initial_pose(msg);
      });
    return {sub};
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
    // Called after GLIM global optimization - rebuild GLIM pts in saved map frame
    Eigen::Isometry3d T_sg;
    { std::lock_guard<std::mutex> lock(mutex_); T_sg = T_savedmap_glimworld_; }
    if (!initialized_) return;

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
      if (submaps.empty()) {
        // Publish entire_map every 10 seconds even without new submaps
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_map_pub_time_).count() >= 10) {
          if (!kill_switch_) publish_entire_map();
          last_map_pub_time_ = now;
        }
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

    // Update viewer: move saved_map drawable to align with GLIM world frame
    if (!kill_switch_) {
      const Eigen::Isometry3d T_glimworld_savedworld = T_result.inverse();
      auto viewer = guik::LightViewer::instance();
      if (viewer) {
        auto found = viewer->find_drawable("saved_map");
        if (found.first)
          found.first->add("model_matrix",
            T_glimworld_savedworld.matrix().cast<float>().eval());
        auto coord = glk::Primitives::coordinate_system();
        viewer->update_drawable("localized_robot", coord,
          guik::VertexColor(submap->T_world_origin.matrix().cast<float>().eval()));
      }
    }

    // Publish /localized_pose (robot in saved map frame)
    if (pose_pub_ && !kill_switch_) {
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
        pose_pub_->publish(msg);
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
  void publish_entire_map() {
    if (!map_pub_) return;

    std::vector<Eigen::Vector3f> all_pts;
    {
      std::lock_guard<std::mutex> lock(map_pts_mutex_);
      all_pts.reserve(saved_map_pts_.size() + glim_submap_pts_.size() + new_submap_pts_.size());
      all_pts.insert(all_pts.end(), saved_map_pts_.begin(),    saved_map_pts_.end());
      all_pts.insert(all_pts.end(), glim_submap_pts_.begin(),  glim_submap_pts_.end());
      all_pts.insert(all_pts.end(), new_submap_pts_.begin(),   new_submap_pts_.end());
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = map_frame_;
    msg.height = 1;
    msg.width  = all_pts.size();
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(all_pts.size());

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    for (const auto& p : all_pts) {
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
    map_voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(vgicp_res_);
    map_voxelmap_->insert(*ds);

    // Store for entire_map publishing
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  std::string map_path_, output_path_, map_frame_, lidar_frame_;
  double vgicp_res_, prior_inf_scale_trans_, prior_inf_scale_rot_;
  Eigen::Matrix3d mount_R_;

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
  bool pending_init_;
  Eigen::Isometry3d T_pending_savedmap_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_savedmap_glimworld_     = Eigen::Isometry3d::Identity();
};

}  // namespace glim_localizer

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localizer::LocalizerExt();
}