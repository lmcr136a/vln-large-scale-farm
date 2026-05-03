/**
 * init_pose_viewer.cpp — GLIM extension module
 *
 * ExtensionModuleROS2 패턴 사용 (자체 rclcpp::Node 생성 없음)
 * GLIM의 node를 create_subscriptions(node)로 받아서 사용.
 */

#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/sub_map.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <glk/texture.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <imgui.h>
#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <mutex>
#include <cmath>
#include <string>
#include <algorithm>

namespace glim_localizer {

class InitPoseViewer : public glim::ExtensionModuleROS2 {
public:
  InitPoseViewer() {
    resolution_  = 1.0f;
    grid_every_  = 5.0f;
    z_min_       = -100.0f;
    z_max_       =  100.0f;
    image_w_ = image_h_ = 0;
    origin_x_ = origin_y_ = 0.0;
    clicked_x_ = clicked_y_ = 0.0;
    yaw_deg_ = 0.0f;
    pose_selected_ = false;
    map_dirty_ = false;
    map_loaded_ = false;

    glim::GlobalMappingCallbacks::on_insert_submap.add(
      [this](const glim::SubMap::ConstPtr& submap) { on_submap(submap); });

    auto viewer = guik::LightViewer::instance();
    if (viewer) {
      viewer->register_ui_callback("init_pose_ui", [this] { draw_ui(); });
    }

    spdlog::info("[InitPoseViewer] loaded");
  }

  ~InitPoseViewer() override {
    if (load_thread_.joinable()) load_thread_.join();
  }

  // GLIM node 재사용 — 자체 node 생성 없음
  virtual std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/initial_pose", rclcpp::QoS(1).reliable());

    // Subscribe to localization transform to update saved_map drawable
    auto transform_sub = std::make_shared<glim::TopicSubscription<geometry_msgs::msg::PoseStamped>>(
      "/localizer/T_glimworld_savedworld",
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        pending_transform_ = Eigen::Isometry3d::Identity();
        pending_transform_.translation() = Eigen::Vector3d(
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        pending_transform_.linear() = Eigen::Quaterniond(
          msg->pose.orientation.w, msg->pose.orientation.x,
          msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();
        has_pending_transform_ = true;
      });
    return {transform_sub};
  }

private:
  void on_submap(const glim::SubMap::ConstPtr& submap) {
    if (!submap || !submap->frame) return;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (int i = 0; i < submap->frame->size(); i++) {
      const Eigen::Vector4d p = submap->T_world_origin * submap->frame->points[i];
      if (p[2] < z_min_ || p[2] > z_max_) continue;
      all_points_3d_.emplace_back(p[0], p[1], (float)p[2]);
    }
    map_dirty_ = true;
  }

  void load_saved_map(const std::string& map_path) {
    spdlog::info("[InitPoseViewer] Loading map: {}", map_path);

    std::ifstream graph_ifs(map_path + "/graph.txt");
    if (!graph_ifs) { spdlog::error("Cannot open graph.txt"); return; }
    int num_submaps = 0;
    std::string tok;
    graph_ifs >> tok >> num_submaps;

    std::vector<Eigen::Vector3f> pts_world;
    pts_world.reserve(5000000);

    for (int i = 0; i < num_submaps; i++) {
      const std::string sp = (boost::format("%s/%06d") % map_path % i).str();
      auto submap = glim::SubMap::load(sp);
      if (!submap || !submap->frame || !submap->frame->has_points()) continue;
      for (size_t j = 0; j < submap->frame->size(); j++) {
        const Eigen::Vector4d pw = submap->T_world_origin * submap->frame->points[j];
        pts_world.push_back({(float)pw[0], (float)pw[1], (float)pw[2]});
      }
      if (i % 200 == 0)
        spdlog::info("[InitPoseViewer] {}/{} ({} pts)", i, num_submaps, pts_world.size());
    }

    // Store points — viewer update happens in draw_ui() (main/GL thread)
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      loaded_map_pts_ = std::move(pts_world);
      all_points_3d_.clear();
      for (const auto& p : loaded_map_pts_) {
        if (p[2] >= z_min_ && p[2] <= z_max_)
          all_points_3d_.emplace_back(p[0], p[1], p[2]);
      }
      map_dirty_ = true;
      map_ready_ = true;  // signal draw_ui to upload to GL
    }
    map_loaded_ = true;
    spdlog::info("[InitPoseViewer] Map loaded: {} pts", loaded_map_pts_.size());
  }

  void draw_ui() {
    auto viewer = guik::LightViewer::instance();
    if (!viewer) return;
    ImGui::SetNextWindowSize(ImVec2(500, 660), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(20, 60), ImGuiCond_FirstUseEver);
    ImGui::Begin("Initial Pose UI");

    ImGui::Separator(); ImGui::Text("Saved Map");
    static char map_path_buf[512] = "";
    // Pre-fill from config_localizer.json on first draw
    static bool path_initialized = false;
    if (!path_initialized) {
      try {
        const glim::Config cfg(glim::GlobalConfig::get_config_path("config_localizer"));
        const std::string mp = cfg.param<std::string>("localizer", "map_path", "");
        if (!mp.empty()) std::strncpy(map_path_buf, mp.c_str(), sizeof(map_path_buf) - 1);
      } catch (...) {}
      path_initialized = true;
    }
    ImGui::SetNextItemWidth(340);
    ImGui::InputText("##map_path", map_path_buf, sizeof(map_path_buf));
    ImGui::SameLine();
    if (ImGui::Button("Load Map")) {
      if (strlen(map_path_buf) > 0) {
        load_thread_ = std::thread([this, path = std::string(map_path_buf)] {
          load_saved_map(path);
        });
      }
    }

    ImGui::Spacing();
    ImGui::Separator(); ImGui::Text("Top-down View");
    if (ImGui::SliderFloat("Resolution (m/px)", &resolution_, 0.05f, 5.0f, "%.2f"))
      map_dirty_ = true;
    ImGui::SetNextItemWidth(240);
    if (ImGui::DragFloatRange2("Z range", &z_min_, &z_max_, 0.5f, -200.0f, 200.0f, "%.0f", "%.0f"))
      map_dirty_ = true;
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) map_dirty_ = true;

    // Apply localization transform to saved_map drawable (from localizer_ext)
    if (has_pending_transform_) {
      auto found = viewer->find_drawable("saved_map");
      if (found.first)
        found.first->add("model_matrix",
          pending_transform_.matrix().cast<float>().eval());
      has_pending_transform_ = false;
    }

    if (map_dirty_) { build_projection(); map_dirty_ = false; }

    // Upload loaded map points to GL (must happen on main/GL thread)
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (map_ready_) {
        auto buf = std::make_shared<glk::PointCloudBuffer>(loaded_map_pts_);
        viewer->update_drawable("saved_map", buf, guik::Rainbow().set_point_scale(1.5f));
        map_ready_ = false;
      }
    }

    if (proj_texture_ && image_w_ > 0) {
      ImVec2 cursor = ImGui::GetCursorScreenPos();
      ImGui::Image(reinterpret_cast<ImTextureID>(proj_texture_->id()),
                   ImVec2(image_w_, image_h_), ImVec2(0, 1), ImVec2(1, 0));

      if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
        ImVec2 mp = ImGui::GetMousePos();
        clicked_x_ = origin_x_ + (mp.x - cursor.x) * resolution_;
        clicked_y_ = origin_y_ + (image_h_ - (mp.y - cursor.y)) * resolution_;
        pose_selected_ = true;
      }

      ImDrawList* dl = ImGui::GetWindowDrawList();
      for (float wx = std::ceil(origin_x_ / grid_every_) * grid_every_;
           wx < origin_x_ + image_w_ * resolution_; wx += grid_every_) {
        float sx = cursor.x + (wx - (float)origin_x_) / resolution_;
        dl->AddLine({sx, cursor.y}, {sx, cursor.y + image_h_}, IM_COL32(100,200,255,70), 1.0f);
      }
      for (float wy = std::ceil(origin_y_ / grid_every_) * grid_every_;
           wy < origin_y_ + image_h_ * resolution_; wy += grid_every_) {
        float sy = cursor.y + ((float)origin_y_ + image_h_ * resolution_ - wy) / resolution_;
        dl->AddLine({cursor.x, sy}, {cursor.x + image_w_, sy}, IM_COL32(100,200,255,70), 1.0f);
      }
      if (origin_x_ < 0 && origin_x_ + image_w_ * resolution_ > 0) {
        float sx = cursor.x + (float)(-origin_x_) / resolution_;
        dl->AddLine({sx, cursor.y}, {sx, cursor.y + image_h_}, IM_COL32(255,80,80,200), 2.0f);
      }
      if (origin_y_ < 0 && origin_y_ + image_h_ * resolution_ > 0) {
        float sy = cursor.y + (float)(origin_y_ + image_h_ * resolution_) / resolution_;
        dl->AddLine({cursor.x, sy}, {cursor.x + image_w_, sy}, IM_COL32(80,255,80,200), 2.0f);
      }
      if (pose_selected_) {
        float sx = cursor.x + (float)(clicked_x_ - origin_x_) / resolution_;
        float sy = cursor.y + (float)(origin_y_ + image_h_ * resolution_ - clicked_y_) / resolution_;
        float yr = yaw_deg_ * M_PI / 180.0f;
        dl->AddCircleFilled({sx, sy}, 6.0f, IM_COL32(255,200,0,255));
        dl->AddLine({sx, sy}, {sx + 22.0f * std::cos(-yr), sy + 22.0f * std::sin(-yr)},
                    IM_COL32(255,200,0,255), 2.5f);
      }
    } else {
      ImGui::TextDisabled("(Load map to see projection)");
    }

    ImGui::Separator();
    if (pose_selected_) ImGui::Text("Selected: X=%.2f  Y=%.2f  z=%.2f",
      clicked_x_, clicked_y_, estimate_ground_z(clicked_x_, clicked_y_));
    else ImGui::TextDisabled("Click map to select position");

    ImGui::SetNextItemWidth(240);
    ImGui::SliderFloat("Yaw (deg, CCW from +X)", &yaw_deg_, -180.0f, 180.0f);
    ImGui::SameLine();
    if (ImGui::Button("0##y")) yaw_deg_ = 0.0f;

    ImGui::Spacing();
    if (!pose_selected_) ImGui::BeginDisabled();
    if (ImGui::Button("Set Initial Pose", ImVec2(200, 32))) publish_pose();
    if (!pose_selected_) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Clear##pose")) pose_selected_ = false;

    ImGui::End();
  }

  double estimate_ground_z(double cx, double cy, double radius = 5.0) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    std::vector<float> zs;
    for (const auto& [x, y, z] : all_points_3d_) {
      double dx = x - cx, dy = y - cy;
      if (dx*dx + dy*dy < radius*radius) zs.push_back(z);
    }
    if (zs.empty()) return 0.0;
    std::sort(zs.begin(), zs.end());
    int p5 = std::max(0, (int)(zs.size() * 0.05));
    int p10 = std::max(1, (int)(zs.size() * 0.10));
    double sum = 0;
    for (int i = p5; i < p10; i++) sum += zs[i];
    return sum / (p10 - p5) + 1.0;
  }

  void build_projection() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (all_points_3d_.empty()) return;
    double min_x=1e9, min_y=1e9, max_x=-1e9, max_y=-1e9;
    for (auto& [x,y,z] : all_points_3d_) {
      min_x=std::min(min_x,x); max_x=std::max(max_x,x);
      min_y=std::min(min_y,y); max_y=std::max(max_y,y);
    }
    const double m = 5.0;
    origin_x_ = min_x-m; origin_y_ = min_y-m;
    image_w_ = std::min((int)((max_x-min_x+2*m)/resolution_)+1, 2048);
    image_h_ = std::min((int)((max_y-min_y+2*m)/resolution_)+1, 2048);

    std::vector<int> hits(image_w_*image_h_, 0);
    for (auto& [x,y,z] : all_points_3d_) {
      int px=(int)((x-origin_x_)/resolution_), py=(int)((y-origin_y_)/resolution_);
      if (px>=0 && px<image_w_ && py>=0 && py<image_h_) hits[py*image_w_+px]++;
    }
    int mh = *std::max_element(hits.begin(), hits.end());
    if (mh==0) return;
    std::vector<uint8_t> rgba(image_w_*image_h_*4, 255);
    for (int i=0; i<image_w_*image_h_; i++) {
      uint8_t v=(uint8_t)((1.0f-std::min(1.0f,hits[i]/(float)std::max(mh/5,1)))*255);
      rgba[i*4]=rgba[i*4+1]=rgba[i*4+2]=v; rgba[i*4+3]=255;
    }
    proj_texture_ = std::make_shared<glk::Texture>(
      Eigen::Vector2i(image_w_, image_h_), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  }

  void publish_pose() {
    if (!pose_pub_) { spdlog::warn("[InitPoseViewer] publisher not ready"); return; }
    const double yr = yaw_deg_ * M_PI / 180.0;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yr, Eigen::Vector3d::UnitZ()));
    const double z = estimate_ground_z(clicked_x_, clicked_y_);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = clicked_x_;
    msg.pose.position.y = clicked_y_;
    msg.pose.position.z = z;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_pub_->publish(msg);
    spdlog::info("[InitPoseViewer] Pose: ({:.2f}, {:.2f}, z={:.2f}, yaw={:.1f}deg)",
      clicked_x_, clicked_y_, z, (double)yaw_deg_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::thread load_thread_;

  // Localization transform from localizer_ext (updated on GL thread in draw_ui)
  Eigen::Isometry3d pending_transform_ = Eigen::Isometry3d::Identity();
  std::atomic<bool> has_pending_transform_{false};

  float resolution_, grid_every_, z_min_, z_max_;
  int image_w_, image_h_;
  double origin_x_, origin_y_;
  std::shared_ptr<glk::Texture> proj_texture_;
  bool map_dirty_, map_loaded_;

  std::mutex cloud_mutex_;
  std::vector<std::tuple<double, double, float>> all_points_3d_;
  std::vector<Eigen::Vector3f> loaded_map_pts_;  // stored for GL upload on main thread
  bool map_ready_ = false;  // set by load thread, consumed by draw_ui

  double clicked_x_, clicked_y_;
  float yaw_deg_;
  bool pose_selected_;
};

}  // namespace glim_localizer

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localizer::InitPoseViewer();
}