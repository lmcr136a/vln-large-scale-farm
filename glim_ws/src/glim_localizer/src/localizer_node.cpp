/**
 * localizer_node.cpp
 *
 * GLIM이 생성한 submap을 saved global map과 VGICP로 정합.
 *
 * 핵심 아이디어:
 *   T_savedmap_glimworld 를 유지.
 *   새 submap 들어오면:
 *     T_init = T_savedmap_glimworld × T_glimworld_submap  (초기 추정)
 *     VGICP → T_savedmap_submap  (보정된 포즈)
 *     T_savedmap_glimworld = T_savedmap_submap × T_glimworld_submap⁻¹  (업데이트)
 *
 * raw scan 비교보다 훨씬 정확:
 *   - submap은 이미 deskewed + covariance 계산됨 (GLIM이 처리)
 *   - map-to-map 비교
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <mutex>
#include <unordered_map>

using gtsam::symbol_shorthand::X;

class LocalizerNode : public rclcpp::Node {
public:
  LocalizerNode() : Node("localizer_node"), initialized_(false), scan_count_(0) {
    // ── params ─────────────────────────────────────────────────────
    map_path_        = declare_parameter<std::string>("map_path", "");
    output_path_     = declare_parameter<std::string>("output_map_path", "");
    output_topic_    = declare_parameter<std::string>("output_pose_topic", "/localized_pose");
    map_frame_       = declare_parameter<std::string>("map_frame", "map");
    lidar_frame_     = declare_parameter<std::string>("lidar_frame", "livox_frame");
    save_interval_   = declare_parameter<int>("save_interval", 30);
    vgicp_voxel_res_ = declare_parameter<double>("vgicp_voxel_resolution", 1.0);

    // /initial_pose 구독 → T_savedmap_glimworld 초기화
    init_pose_topic_ = declare_parameter<std::string>("init_pose_topic", "/initial_pose");

    if (output_path_.empty()) output_path_ = "/tmp/dump";
    boost::filesystem::create_directories(output_path_);

    // ── map 로드 ───────────────────────────────────────────────────
    load_map();

    // ── ROS I/O ───────────────────────────────────────────────────
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    // initial_pose: T_savedmap_lidar (UI에서 클릭)
    init_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      init_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        on_initial_pose(msg);
      });

    // GLIM submap callback: 새 submap → saved map과 VGICP
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      [this](const glim::SubMap::ConstPtr& submap) {
        on_new_submap(submap);
      });

    // periodic save
    if (save_interval_ > 0) {
      save_timer_ = create_wall_timer(
        std::chrono::seconds(save_interval_),
        [this] { save_map(); });
    }

    RCLCPP_INFO(get_logger(), "LocalizerNode ready. Waiting for initial pose on [%s]",
      init_pose_topic_.c_str());
  }

  ~LocalizerNode() { save_map(); }

private:
  // ── Map 로드 (직접 binary 읽기 — voxelmap rebuild 없이) ─────────
  void load_map() {
    RCLCPP_INFO(get_logger(), "Loading map: %s", map_path_.c_str());

    std::ifstream graph_ifs(map_path_ + "/graph.txt");
    if (!graph_ifs) throw std::runtime_error("Cannot open graph.txt");
    int num_submaps = 0;
    std::string tok;
    graph_ifs >> tok >> num_submaps;

    std::vector<Eigen::Vector4d> all_pts;
    all_pts.reserve(30000000);

    for (int i = 0; i < num_submaps; i++) {
      const std::string sp = (boost::format("%s/%06d") % map_path_ % i).str();
      auto submap = glim::SubMap::load(sp);
      if (!submap || !submap->frame || !submap->frame->has_points()) continue;

      for (size_t j = 0; j < submap->frame->size(); j++) {
        all_pts.push_back(submap->T_world_origin * submap->frame->points[j]);
      }

      if (i % 200 == 0)
        RCLCPP_INFO(get_logger(), "Loading: %d/%d (%zu pts)", i, num_submaps, all_pts.size());
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu pts from %d submaps", all_pts.size(), num_submaps);

    // voxelmap 빌드 (2× resolution으로 다운샘플)
    auto merged = std::make_shared<gtsam_points::PointCloudCPU>();
    merged->add_points(all_pts);
    auto ds = voxel_downsample(*merged, vgicp_voxel_res_ * 2.0);
    add_isotropic_covs(*ds, 0.01);

    map_voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(vgicp_voxel_res_);
    map_voxelmap_->insert(*ds);
    RCLCPP_INFO(get_logger(), "Voxelmap built: %zu pts (res=%.1f)", ds->size(), vgicp_voxel_res_);

    // GlobalMapping — 새 submap 저장용 (빈 상태)
    glim::GlobalMappingParams params;
    params.enable_optimization = false;
    params.registration_error_factor_type = "VGICP";
    params.enable_gpu = false;
    global_mapping_ = std::make_shared<glim::GlobalMapping>(params);
  }

  // ── Initial pose → T_savedmap_glimworld 초기화 ──────────────────
  // UI에서: T_savedmap_lidar (saved map frame에서 lidar 위치)
  // GLIM은 자체 world frame에서 시작 → 아직 T_glimworld_lidar 모름
  // → 일단 T_savedmap_glimworld = T_savedmap_lidar 로 설정
  //   (첫 submap이 들어오면 T_glimworld_submap으로 보정됨)
  void on_initial_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    T.linear() = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();

    // mounting rotation 적용 (upside-down LiDAR)
    T.linear() = T.linear() * mount_R_;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    T_savedmap_glimworld_ = T;  // 첫 submap 전까지의 근사값
    initialized_ = true;

    RCLCPP_INFO(get_logger(), "Initial pose set: (%.2f, %.2f, %.2f)",
      T.translation().x(), T.translation().y(), T.translation().z());
  }

  // ── 새 GLIM submap → saved map과 VGICP ──────────────────────────
  void on_new_submap(const glim::SubMap::ConstPtr& submap) {
    if (!initialized_) return;
    if (!submap || !submap->frame || !submap->frame->has_points()) return;

    // T_glimworld_submap: GLIM이 추정한 submap pose (GLIM world frame)
    const Eigen::Isometry3d T_glimworld_submap = submap->T_world_origin;

    // T_init = T_savedmap_glimworld × T_glimworld_submap
    Eigen::Isometry3d T_savedmap_glimworld_cur;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      T_savedmap_glimworld_cur = T_savedmap_glimworld_;
    }
    const Eigen::Isometry3d T_init = T_savedmap_glimworld_cur * T_glimworld_submap;

    // submap frame은 이미 covariance 있음 (GLIM이 계산) → 바로 VGICP
    auto cloud = submap_to_gtsam(*submap);

    // VGICP: submap points (local frame) → saved map voxelmap
    // Unary: T_savedmap_submap 만 최적화
    const Eigen::Isometry3d T_savedmap_submap = vgicp_align(cloud, T_init);

    // T_savedmap_glimworld 업데이트
    // T_savedmap_glimworld = T_savedmap_submap × T_glimworld_submap⁻¹
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      T_savedmap_glimworld_ = T_savedmap_submap * T_glimworld_submap.inverse();
    }

    // submap 중간 frame의 stamp 사용
    double stamp = submap->frames.empty()
      ? node_->now().seconds()
      : submap->frames[submap->frames.size() / 2]->stamp;

    publish_pose(T_savedmap_submap, rclcpp::Time(static_cast<uint64_t>(stamp * 1e9)));

    // 새 submap을 GlobalMapping에 insert (저장용)
    auto sm = std::const_pointer_cast<glim::SubMap>(submap);
    sm->T_world_origin = T_savedmap_submap;  // corrected pose
    global_mapping_->insert_submap(sm);
    scan_count_++;

    RCLCPP_INFO(get_logger(), "Submap #%d localized: (%.2f, %.2f, %.2f)",
      scan_count_,
      T_savedmap_submap.translation().x(),
      T_savedmap_submap.translation().y(),
      T_savedmap_submap.translation().z());
  }

  // ── submap → gtsam_points (covariance 재사용) ───────────────────
  std::shared_ptr<gtsam_points::PointCloudCPU> submap_to_gtsam(
    const glim::SubMap& submap)
  {
    // submap->frame은 이미 gtsam_points::PointCloudCPU
    // covariance가 있으면 그대로 사용, 없으면 isotropic으로 추가
    auto cloud = std::dynamic_pointer_cast<gtsam_points::PointCloudCPU>(submap.frame);
    if (!cloud) {
      // base class → copy
      cloud = std::make_shared<gtsam_points::PointCloudCPU>();
      std::vector<Eigen::Vector4d> pts(
        submap.frame->points, submap.frame->points + submap.frame->size());
      cloud->add_points(pts);
    }

    if (!cloud->has_covs()) {
      add_isotropic_covs(*cloud, 0.01);
    }
    return cloud;
  }

  // ── VGICP alignment ──────────────────────────────────────────────
  // Unary form: saved map 고정, T_savedmap_submap만 최적화
  Eigen::Isometry3d vgicp_align(
    const std::shared_ptr<gtsam_points::PointCloudCPU>& cloud,
    const Eigen::Isometry3d& init_pose)
  {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(init_pose.matrix()));

    auto vmap = std::static_pointer_cast<const gtsam_points::GaussianVoxelMap>(map_voxelmap_);
    graph.add(gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
      gtsam::Pose3::Identity(), X(0), vmap, cloud));

    gtsam::LevenbergMarquardtParams lm;
    lm.maxIterations = 30;
    lm.relativeErrorTol = 1e-3;
    lm.absoluteErrorTol = 1e-4;

    try {
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, values, lm).optimize();
      return Eigen::Isometry3d(result.at<gtsam::Pose3>(X(0)).matrix());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "VGICP failed: %s — using initial estimate", e.what());
      return init_pose;
    }
  }

  // ── Voxel downsample ─────────────────────────────────────────────
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
      if (seen.insert({key, true}).second)
        pts.push_back(p);
    }
    auto out = std::make_shared<gtsam_points::PointCloudCPU>();
    out->add_points(pts);
    return out;
  }

  static void add_isotropic_covs(gtsam_points::PointCloudCPU& cloud, double scale) {
    std::vector<Eigen::Matrix4d> covs(cloud.size(), Eigen::Matrix4d::Zero());
    for (auto& c : covs)
      c.topLeftCorner<3,3>() = scale * Eigen::Matrix3d::Identity();
    cloud.add_covs(covs);
  }

  // ── Publish pose + TF ────────────────────────────────────────────
  void publish_pose(const Eigen::Isometry3d& pose, const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose = tf2::toMsg(pose);
    pose_pub_->publish(msg);

    const Eigen::Quaterniond q(pose.rotation());
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = lidar_frame_;
    tf.transform.translation.x = pose.translation().x();
    tf.transform.translation.y = pose.translation().y();
    tf.transform.translation.z = pose.translation().z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  void save_map() {
    if (!global_mapping_ || scan_count_ == 0) return;
    RCLCPP_INFO(get_logger(), "Saving %d submaps to %s...", scan_count_, output_path_.c_str());
    global_mapping_->save(output_path_);
  }

  // ── members ──────────────────────────────────────────────────────
  std::string map_path_, output_path_, init_pose_topic_, output_topic_;
  std::string map_frame_, lidar_frame_;
  double vgicp_voxel_res_;
  int save_interval_, scan_count_;

  // mounting rotation: upside-down LiDAR → Rx(180°)
  Eigen::Matrix3d mount_R_ = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();

  std::shared_ptr<glim::GlobalMapping> global_mapping_;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> map_voxelmap_;

  std::mutex pose_mutex_;
  bool initialized_;
  Eigen::Isometry3d T_savedmap_glimworld_ = Eigen::Isometry3d::Identity();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_pose_sub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp::Node> node_ = std::shared_ptr<rclcpp::Node>(this, [](auto*){});
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizerNode>());
  rclcpp::shutdown();
}