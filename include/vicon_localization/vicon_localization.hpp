#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace vicon_localization
{
    class ViconLocalization : public nav2_util::LifecycleNode
    {
    public:
        explicit ViconLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    private:
        // Node callbacks
        void viconPose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void periodic_loop();

        // Node configuration parameters
        std::string viconPose_topic_;
        std::string global_frame_id_;
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string vicon_world_frame_;
        std::string vicon_robot_frame_;

        double localization_publishing_rate_{20.0};
        double lookup_timeout_sec_{0.05};
        double transform_tolerance_{0.20};
        double max_pose_age_sec_{0.5};
        bool tf_broadcast_{true};

        // Node variables
        std::atomic_bool active_{false};
        std::mutex sample_mutex_;
        geometry_msgs::msg::Twist prev_computed_twist_;
        geometry_msgs::msg::PoseWithCovarianceStamped prev_pose_;
        std::deque<geometry_msgs::msg::PoseStamped> viconPose_samples_;

        // Publishers and subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr viconPose_sub_;

        rclcpp::TimerBase::SharedPtr cycle_timer_;
        rclcpp_lifecycle::LifecyclePublisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<
            nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Utility functions
        bool computeTwist(const geometry_msgs::msg::PoseWithCovarianceStamped& prev,
                          const geometry_msgs::msg::PoseWithCovarianceStamped& curr, geometry_msgs::msg::Twist& twist);
    };
} // namespace vicon_localization
