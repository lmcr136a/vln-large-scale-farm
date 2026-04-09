/**
 * localizer_ext.cpp - GLIM extension module for localization against a saved map.
 *
 * Follows the gnss_global pattern:
 *   - Inherits ExtensionModuleROS2 to reuse GLIM's ROS node (no own node creation)
 *   - on_insert_submap -> VGICP(submap vs saved map) -> PriorFactor injected into GLIM
 *   - on_smoother_update -> inject factors into GLIM factor graph
 *
 * Key transform:
 *   T_savedmap_glimworld = T_savedmap_lidar * T_glimworld_lidar^-1
 *
 * mount_R_ (Rx(180deg) for upside-down LiDAR):
 *   submap->T_world_origin includes Rx(180deg) from IMU gravity correction.
 *   T_savedmap_lidar (from user click) only has yaw.
 *   Adding mount_R_ to T_savedmap_lidar makes the Rx(180deg) cancel:
 *     (R_yaw * Rx180) * Rx180^-1 = R_yaw  (pure yaw only, correct)
 */

#include <deque>
#include <atomic>
#include <thread>
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
    has_glim_pose_(false), pending_init_(false) {

    // --- Default parameters ---
    map_path_              = "";
    output_path_           = "/tmp/dump";
    map_frame_             = "map";
    lidar_frame_           = "livox_frame";
    vgicp_res_             = 1.0;
    prior_inf_scale_trans_ = 1e3;
    prior_inf_scale_rot_   = 1e2;
    // Upside-down LiDAR: Rx(180deg) - cancels with submap T_world_origin rotation
    mount_R_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // --- Load config_localizer.json ---
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

    // Environment variable fallback
    if (map_path_.empty()) {
      const char* e = std::getenv("LOCALIZER_MAP_PATH");
      if (e) map_path_ = e;
    }

    // --- GLIM callbacks (gnss_global pattern) ---
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&LocalizerExt::on_insert_submap_cb, this, _1));
    glim::GlobalMappingCallbacks::on_smoother_update.add(
      std::bind(&LocalizerExt::on_smoother_update, this, _1, _2, _3));

    if (map_path_.empty()) {
      logger->warn("map_path not set, localization disabled");
      return;
    }

    // Load map in background thread to avoid blocking GLIM initialization
    backend_thread_ = std::thread([this] { backend_task(); });
    logger->info("Loading saved map in background: {}", map_path_);
  }

  ~LocalizerExt() override {
    kill_switch_ = true;
    if (backend_thread_.joinable()) backend_thread_.join();
    pose_pub_.reset();
  }

  // Reuse GLIM's ROS node - no own node creation to avoid rclcpp context issues
  virtual std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/localized_pose", 10);
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
    // Track latest GLIM pose without TF (use submap T_world_origin directly)
    {
      std::lock_guard<std::mutex> lock(glim_pose_mutex_);
      latest_glim_pose_ = submap->T_world_origin;
      has_glim_pose_ = true;
    }

    // If initial_pose arrived before any submap, recalculate T_savedmap_glimworld
    // using the first real submap pose (which includes IMU gravity correction)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_init_) {
        // T_savedmap_glimworld = T_savedmap_lidar(with mount_R) * T_glimworld_submap^-1
        // mount_R cancels with the Rx(180) in T_world_origin:
        //   (R_yaw * Rx180) * Rx180^-1 = R_yaw  (pure yaw, correct)
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

  void on_smoother_update(
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

  // -----------------------------------------------------------------------
  // Initial pose handler
  // T_savedmap_glimworld = T_savedmap_lidar * T_glimworld_lidar^-1
  // -----------------------------------------------------------------------
  void on_initial_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    if (!map_loaded_) { logger->warn("Map not loaded yet"); return; }

    // T_savedmap_lidar: user click position in saved map frame
    // mount_R_ (Rx180) added so it cancels with submap T_world_origin rotation
    Eigen::Isometry3d T_savedmap_lidar = Eigen::Isometry3d::Identity();
    T_savedmap_lidar.translation() = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    T_savedmap_lidar.linear() = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix()
      * mount_R_;

    // T_glimworld_lidar: GLIM pose at click time (from latest submap T_world_origin)
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
      // Store and wait for first submap to arrive
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

    while (!kill_switch_) {
      const auto submaps = input_submap_queue_.get_all_and_clear();
      if (submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (!initialized_) continue;
      for (const auto& sm : submaps) process_submap(sm);
    }
  }

  // -----------------------------------------------------------------------
  // Submap processing: VGICP -> PriorFactor -> viewer update
  // -----------------------------------------------------------------------
  void process_submap(const glim::SubMap::ConstPtr& submap) {
    if (!submap || !submap->frame || !submap->frame->has_points()) return;

    Eigen::Isometry3d T_sg;
    { std::lock_guard<std::mutex> lock(mutex_); T_sg = T_savedmap_glimworld_; }

    // Transform submap points from sensor local frame to GLIM world frame
    // so we can compare directly with saved map (also in world frame)
    auto cloud = submap_to_glimworld(*submap);

    // VGICP: find T_savedmap_glimworld that maps GLIM world pts to saved map
    const Eigen::Isometry3d T_result = vgicp_world(cloud, T_sg);

    // Update T_savedmap_glimworld
    { std::lock_guard<std::mutex> lock(mutex_); T_savedmap_glimworld_ = T_result; }

    // T_savedmap_submap: robot position in saved map frame
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

    // Add weak PriorFactor to keep GLIM consistent with saved map
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
    info.topLeftCorner<3, 3>()     = prior_inf_scale_rot_   * Eigen::Matrix3d::Identity();
    info.bottomRightCorner<3, 3>() = prior_inf_scale_trans_ * Eigen::Matrix3d::Identity();
    output_factors_.push_back(
      gtsam::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(submap->id),
        gtsam::Pose3(submap->T_world_origin.matrix()),
        gtsam::noiseModel::Gaussian::Information(info)));

    // Update viewer: move saved_map drawable to align with new GLIM map
    // model_matrix = T_glimworld_savedworld = T_savedmap_glimworld^-1
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

    // Publish /localized_pose (robot position in saved map frame)
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
    scan_count_++;
  }

  // -----------------------------------------------------------------------
  // Convert submap points from sensor local frame to GLIM world frame
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

  // VGICP: source = GLIM world frame points, X(0) = T_savedmap_glimworld
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
    } catch (...) {
      return init;
    }
  }

  // -----------------------------------------------------------------------
  // Load saved map: read submap binaries directly (no GlobalMapping::load)
  // -----------------------------------------------------------------------
  void load_map() {
    std::ifstream ifs(map_path_ + "/graph.txt");
    if (!ifs) throw std::runtime_error("Cannot open " + map_path_ + "/graph.txt");
    int num_submaps = 0;
    std::string tok;
    ifs >> tok >> num_submaps;

    std::vector<Eigen::Vector4d> all_pts;
    all_pts.reserve(20000000);
    const int stride = std::max(1, num_submaps / 200);

    for (int i = 0; i < num_submaps; i++) {
      auto sm = glim::SubMap::load((boost::format("%s/%06d") % map_path_ % i).str());
      if (!sm || !sm->frame || !sm->frame->has_points()) continue;

      for (size_t j = 0; j < sm->frame->size(); j++)
        all_pts.push_back(sm->T_world_origin * sm->frame->points[j]);

      if (i % stride == 0) {
        auto ds = std::dynamic_pointer_cast<gtsam_points::PointCloudCPU>(sm->frame);
        if (!ds) {
          ds = std::make_shared<gtsam_points::PointCloudCPU>();
          ds->add_points(std::vector<Eigen::Vector4d>(
            sm->frame->points, sm->frame->points + sm->frame->size()));
        }
        if (!ds->has_covs()) add_isotropic_covs(*ds, 0.01);
      }

      if (i % 200 == 0)
        logger->info("Loading {}/{} ({} pts)", i, num_submaps, all_pts.size());
    }

    // Build global merged voxelmap
    auto merged = std::make_shared<gtsam_points::PointCloudCPU>();
    merged->add_points(all_pts);
    auto ds = voxel_downsample(*merged, vgicp_res_ * 2.0);
    add_isotropic_covs(*ds, 0.01);
    map_voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(vgicp_res_);
    map_voxelmap_->insert(*ds);
    logger->info("Map loaded: {} pts total, voxelmap {} pts (res={:.1f})",
      all_pts.size(), ds->size(), vgicp_res_);
  }

  // -----------------------------------------------------------------------
  // Helpers
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
  // Member variables
  // -----------------------------------------------------------------------
  std::shared_ptr<spdlog::logger> logger;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::string map_path_, output_path_, map_frame_, lidar_frame_;
  double vgicp_res_, prior_inf_scale_trans_, prior_inf_scale_rot_;
  Eigen::Matrix3d mount_R_;  // Upside-down LiDAR correction: Rx(180deg)

  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> map_voxelmap_;

  glim::ConcurrentVector<glim::SubMap::ConstPtr> input_submap_queue_;
  glim::ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors_;

  std::atomic<bool> kill_switch_;
  std::atomic<bool> map_loaded_;
  std::thread backend_thread_;

  // Track latest GLIM pose via submap callback (no TF)
  std::mutex glim_pose_mutex_;
  Eigen::Isometry3d latest_glim_pose_ = Eigen::Isometry3d::Identity();
  bool has_glim_pose_;

  std::mutex mutex_;
  bool initialized_;
  int scan_count_;

  // Pending initial pose: stored when user clicks before first submap arrives
  bool pending_init_;
  Eigen::Isometry3d T_pending_savedmap_lidar_ = Eigen::Isometry3d::Identity();

  // Core transform: maps GLIM world frame -> saved map world frame
  Eigen::Isometry3d T_savedmap_glimworld_ = Eigen::Isometry3d::Identity();
};

}  // namespace glim_localizer

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localizer::LocalizerExt();
}