#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "gps_optimization/GNSS_Processing.hpp"

// GTSAM libraries
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

// PCL libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <deque>
#include <map>
#include <mutex>

class GpsFusionNode : public rclcpp::Node 
{
public:
    GpsFusionNode() : Node("gps_fusion_node"), 
                      origin_set_(false), 
                      last_node_index_(0),
                      first_pose_saved_(false),
                      gps_factor_count_(0)
    {
        // Load parameters
        this->declare_parameter("odom_topic", "/Odometry");
        this->declare_parameter("gps_topic", "/ublox_driver/receiver_lla");
        this->declare_parameter("cloud_registered_topic", "/cloud_registered");
        this->declare_parameter("laser_map_gps_fused_topic", "/Laser_map_gps_fused");
        this->declare_parameter("path_gps_fused_topic", "/path_gps_fused");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("camera_init_frame", "camera_init");
        this->declare_parameter("map_update_interval", 10.0);
        this->declare_parameter("max_scan_history", 1000);
        this->declare_parameter("gps_noise_x", 2.0);
        this->declare_parameter("gps_noise_y", 2.0);
        this->declare_parameter("gps_noise_z", 4.0);
        this->declare_parameter("odom_noise_x", 0.1);
        this->declare_parameter("odom_noise_y", 0.1);
        this->declare_parameter("odom_noise_z", 0.1);
        this->declare_parameter("odom_noise_roll", 0.1);
        this->declare_parameter("odom_noise_pitch", 0.1);
        this->declare_parameter("odom_noise_yaw", 0.1);

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string gps_topic = this->get_parameter("gps_topic").as_string();
        std::string cloud_topic = this->get_parameter("cloud_registered_topic").as_string();
        laser_map_gps_fused_topic_ = this->get_parameter("laser_map_gps_fused_topic").as_string();
        path_gps_fused_topic_ = this->get_parameter("path_gps_fused_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        camera_init_frame_ = this->get_parameter("camera_init_frame").as_string();
        double map_update_interval = this->get_parameter("map_update_interval").as_double();
        max_scan_history_ = this->get_parameter("max_scan_history").as_int();

        // GPS noise model
        double gps_noise_x = this->get_parameter("gps_noise_x").as_double();
        double gps_noise_y = this->get_parameter("gps_noise_y").as_double();
        double gps_noise_z = this->get_parameter("gps_noise_z").as_double();
        gps_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(3) << gps_noise_x * gps_noise_x, 
                                 gps_noise_y * gps_noise_y, 
                                 gps_noise_z * gps_noise_z).finished()
        );

        // Odometry noise model
        double odom_noise_x = this->get_parameter("odom_noise_x").as_double();
        double odom_noise_y = this->get_parameter("odom_noise_y").as_double();
        double odom_noise_z = this->get_parameter("odom_noise_z").as_double();
        double odom_noise_roll = this->get_parameter("odom_noise_roll").as_double();
        double odom_noise_pitch = this->get_parameter("odom_noise_pitch").as_double();
        double odom_noise_yaw = this->get_parameter("odom_noise_yaw").as_double();
        odom_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << odom_noise_roll * odom_noise_roll,
                                 odom_noise_pitch * odom_noise_pitch,
                                 odom_noise_yaw * odom_noise_yaw,
                                 odom_noise_x * odom_noise_x,
                                 odom_noise_y * odom_noise_y,
                                 odom_noise_z * odom_noise_z).finished()
        );

        // Subscribers
        sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, 10, 
            std::bind(&GpsFusionNode::gpsCallback, this, std::placeholders::_1)
        );
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 100, 
            std::bind(&GpsFusionNode::odomCallback, this, std::placeholders::_1)
        );

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 100,
            std::bind(&GpsFusionNode::cloudCallback, this, std::placeholders::_1)
        );

        // Publishers
        pub_laser_map_gps_fused_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            laser_map_gps_fused_topic_, 10);
        pub_path_gps_fused_ = this->create_publisher<nav_msgs::msg::Path>(
            path_gps_fused_topic_, 10);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // GTSAM ISAM2 initialization
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam_ = std::make_shared<gtsam::ISAM2>(parameters);

        // Timer for periodic map reconstruction
        map_update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(map_update_interval),
            std::bind(&GpsFusionNode::reconstructMap, this)
        );

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "GPS Fusion Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Odom topic: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  GPS topic: %s", gps_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Cloud topic: %s", cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Map output: %s", laser_map_gps_fused_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Path output: %s", path_gps_fused_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Map update interval: %.1f seconds", map_update_interval);
        RCLCPP_INFO(this->get_logger(), "  GPS noise: [%.1f, %.1f, %.1f]", gps_noise_x, gps_noise_y, gps_noise_z);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    struct ScanData {
        rclcpp::Time timestamp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        int node_index;
    };

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check GPS status
        if (msg->status.status < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "GPS fix not available (status: %d)", msg->status.status);
            return;
        }

        // Initialize GPS origin on first valid GPS message
        if (!origin_set_) {
            gnss_handler_.InitOriginPosition(msg->latitude, msg->longitude, msg->altitude);
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "🌍 GPS ORIGIN SET");
            RCLCPP_INFO(this->get_logger(), "  Latitude:  %.8f", msg->latitude);
            RCLCPP_INFO(this->get_logger(), "  Longitude: %.8f", msg->longitude);
            RCLCPP_INFO(this->get_logger(), "  Altitude:  %.2f m", msg->altitude);
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "");
            return;
        }

        // Convert GPS to ENU coordinates
        gnss_handler_.UpdateXYZ(msg->latitude, msg->longitude, msg->altitude);
        
        // Store GPS measurement with CURRENT TIME (타임스탬프 동기화 문제 해결)
        gps_queue_.push_back({
            this->now(),  // msg->header.stamp 대신 현재 시간 사용!
            gtsam::Point3(gnss_handler_.local_E, gnss_handler_.local_N, gnss_handler_.local_U)
        });

        // Keep queue size reasonable
        if (gps_queue_.size() > 100) {
            gps_queue_.pop_front();
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "📡 GPS: E=%.2f, N=%.2f, U=%.2f (queue: %lu)", 
                            gnss_handler_.local_E, gnss_handler_.local_N, gnss_handler_.local_U,
                            gps_queue_.size());
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!origin_set_ || last_node_index_ == 0) {
            return;
        }

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Store scan with current node index
        scan_history_.push_back({
            msg->header.stamp,
            cloud,
            last_node_index_
        });

        // Limit history size
        if (scan_history_.size() > static_cast<size_t>(max_scan_history_)) {
            scan_history_.pop_front();
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                            "💾 Stored scan %lu with %lu points (node %d)", 
                            scan_history_.size(), cloud->size(), last_node_index_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!origin_set_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Waiting for GPS origin...");
            return;
        }

        // Convert ROS Odometry to GTSAM Pose3
        gtsam::Pose3 current_pose = rosPoseToGtsamPose(msg->pose.pose);
        rclcpp::Time current_time = this->now(); 

        // First odometry: initialize with prior factor
        if (last_node_index_ == 0) {
            gtsam::noiseModel::Diagonal::shared_ptr prior_noise = 
                gtsam::noiseModel::Diagonal::Variances(
                    (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished()
                );
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, current_pose, prior_noise));
            initial_estimate_.insert(0, current_pose);
            
            if (!first_pose_saved_) {
                first_camera_init_pose_ = current_pose;
                first_pose_saved_ = true;
                RCLCPP_INFO(this->get_logger(), "");
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "🎯 FIRST POSE INITIALIZED");
                RCLCPP_INFO(this->get_logger(), "  Position: [%.2f, %.2f, %.2f]",
                           current_pose.translation().x(),
                           current_pose.translation().y(),
                           current_pose.translation().z());
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "");
            }
            
            last_pose_ = current_pose;
            last_time_ = current_time;
            last_node_index_ = 1;
            return;
        }

        // Add between factor (odometry constraint)
        gtsam::Pose3 relative_pose = last_pose_.inverse() * current_pose;
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            last_node_index_ - 1, last_node_index_, relative_pose, odom_noise_
        ));
        initial_estimate_.insert(last_node_index_, current_pose);

        // Add GPS factor if available
        bool gps_added = false;
        if (!gps_queue_.empty()) {
            auto closest_gps = findClosestGPS(current_time);
            if (closest_gps.has_value()) {
                graph_.add(gtsam::GPSFactor(
                    last_node_index_, closest_gps.value(), gps_noise_
                ));
                
                // GPS와 현재 pose 간 거리 계산
                gtsam::Point3 gps_point = closest_gps.value();
                gtsam::Point3 odom_point = current_pose.translation();
                double distance = (gps_point - odom_point).norm();
                
                gps_factor_count_++;
                gps_added = true;
                
                RCLCPP_INFO(this->get_logger(), 
                           "✅ GPS FACTOR ADDED! Node=%d, GPS=[%.1f,%.1f,%.1f], "
                           "Odom=[%.1f,%.1f,%.1f], Dist=%.1fm, Total GPS factors=%d",
                           last_node_index_,
                           gps_point.x(), gps_point.y(), gps_point.z(),
                           odom_point.x(), odom_point.y(), odom_point.z(),
                           distance, gps_factor_count_);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "❌ GPS not added: No close measurement (queue size: %lu)",
                                    gps_queue_.size());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                "❌ GPS not added: Queue is empty");
        }

        // Update ISAM2
        isam_->update(graph_, initial_estimate_);
        isam_->update();

        // Clear for next iteration
        graph_.resize(0);
        initial_estimate_.clear();

        // Get optimized result
        gtsam::Values result = isam_->calculateEstimate();
        gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(last_node_index_);

        // Store optimized pose
        optimized_poses_[last_node_index_] = optimized_pose;

        // Publish path (모든 최적화된 pose 포함)
        publishPath();

        // Broadcast TF
        broadcastTF(optimized_pose, current_time);

        // Update for next iteration
        last_pose_ = current_pose;
        last_time_ = current_time;
        last_node_index_++;

        if (gps_added) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "🔧 Node %d optimized (GPS factor ratio: %d/%d = %.1f%%)", 
                                last_node_index_ - 1, 
                                gps_factor_count_, last_node_index_,
                                100.0 * gps_factor_count_ / last_node_index_);
        }
    }

    void reconstructMap()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (scan_history_.empty() || optimized_poses_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                "No scans (%lu) or poses (%lu) for map reconstruction",
                                scan_history_.size(), optimized_poses_.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), 
                   "🗺️  Reconstructing map from %lu scans with %lu optimized poses...",
                   scan_history_.size(), optimized_poses_.size());

        // Create global map
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);

        int reconstructed_scans = 0;
        for (const auto& scan : scan_history_) {
            auto it = optimized_poses_.find(scan.node_index);
            if (it == optimized_poses_.end()) {
                continue;
            }

            gtsam::Pose3 optimized_pose = it->second;

            // Convert GTSAM pose to Eigen transform
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 3>(0, 0) = optimized_pose.rotation().matrix().cast<float>();
            transform.block<3, 1>(0, 3) = optimized_pose.translation().cast<float>();

            // Transform scan to map frame
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*scan.cloud, *transformed_cloud, transform);

            *global_map += *transformed_cloud;
            reconstructed_scans++;
        }

        RCLCPP_INFO(this->get_logger(), 
                   "✅ Map reconstructed: %lu points from %d/%lu scans", 
                   global_map->size(), reconstructed_scans, scan_history_.size());

        // Publish reconstructed map
        if (!global_map->empty()) {
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*global_map, output);
            output.header.stamp = this->now();
            output.header.frame_id = map_frame_;
            pub_laser_map_gps_fused_->publish(output);
        }
    }

    void publishPath()
    {
        if (optimized_poses_.empty()) {
            return;
        }

        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = map_frame_;

        // 모든 최적화된 pose를 순서대로 추가
        for (const auto& [node_index, optimized_pose] : optimized_poses_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = map_frame_;
            
            pose_stamped.pose.position.x = optimized_pose.translation().x();
            pose_stamped.pose.position.y = optimized_pose.translation().y();
            pose_stamped.pose.position.z = optimized_pose.translation().z();
            
            gtsam::Quaternion quat = optimized_pose.rotation().toQuaternion();
            pose_stamped.pose.orientation.w = quat.w();
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            
            path.poses.push_back(pose_stamped);
        }

        pub_path_gps_fused_->publish(path);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                            "📍 Published path with %lu poses", path.poses.size());
    }

    void broadcastTF(const gtsam::Pose3& optimized_pose, const rclcpp::Time& stamp)
    {
        if (!first_pose_saved_) {
            return;
        }

        // map → camera_init = optimized_pose × last_pose_.inverse()
        gtsam::Pose3 map_to_camera_init = optimized_pose * last_pose_.inverse();

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = map_frame_;
        transform_stamped.child_frame_id = camera_init_frame_;

        transform_stamped.transform.translation.x = map_to_camera_init.translation().x();
        transform_stamped.transform.translation.y = map_to_camera_init.translation().y();
        transform_stamped.transform.translation.z = map_to_camera_init.translation().z();

        gtsam::Quaternion quat = map_to_camera_init.rotation().toQuaternion();
        transform_stamped.transform.rotation.w = quat.w();
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    gtsam::Pose3 rosPoseToGtsamPose(const geometry_msgs::msg::Pose& pose)
    {
        gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        gtsam::Point3 translation(pose.position.x, pose.position.y, pose.position.z);
        return gtsam::Pose3(rotation, translation);
    }

    std::optional<gtsam::Point3> findClosestGPS(const rclcpp::Time& target_time)
    {
        if (gps_queue_.empty()) {
            return std::nullopt;
        }

        auto closest = std::min_element(
            gps_queue_.begin(), gps_queue_.end(),
            [&target_time](const GPSMeasurement& a, const GPSMeasurement& b) {
                auto diff_a = std::abs((rclcpp::Time(a.stamp) - target_time).seconds());
                auto diff_b = std::abs((rclcpp::Time(b.stamp) - target_time).seconds());
                return diff_a < diff_b;
            }
        );

        // 시간 허용치를 5초로 늘림 (타임스탬프 동기화 문제 대응)
        double time_diff = std::abs((rclcpp::Time(closest->stamp) - target_time).seconds());
        if (time_diff < 5.0) {
            return closest->position;
        }

        return std::nullopt;
    }

    // Member variables
    GnssProcess gnss_handler_;
    bool origin_set_;
    int last_node_index_;
    gtsam::Pose3 last_pose_;
    rclcpp::Time last_time_;
    bool first_pose_saved_;
    gtsam::Pose3 first_camera_init_pose_;
    int gps_factor_count_;  // GPS Factor 추가 카운트

    // GTSAM
    std::shared_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;

    // Optimized poses storage
    std::map<int, gtsam::Pose3> optimized_poses_;

    // Scan history
    std::deque<ScanData> scan_history_;
    int max_scan_history_;

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_map_gps_fused_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_gps_fused_;

    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Frame names and topics
    std::string map_frame_;
    std::string camera_init_frame_;
    std::string laser_map_gps_fused_topic_;
    std::string path_gps_fused_topic_;

    // GPS queue
    struct GPSMeasurement {
        builtin_interfaces::msg::Time stamp;
        gtsam::Point3 position;
    };
    std::deque<GPSMeasurement> gps_queue_;

    // Timer for map reconstruction
    rclcpp::TimerBase::SharedPtr map_update_timer_;

    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
