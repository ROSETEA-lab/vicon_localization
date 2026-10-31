#include "vicon_localization/vicon_localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace vicon_localization
{
    ViconLocalization::ViconLocalization(const rclcpp::NodeOptions& options)
        : nav2_util::LifecycleNode("vicon_localization", "", options)
    {
        // Declaring configuration parameters
        declare_parameter<std::string>("vicon_pose_topic", "/vicon/pose");

        declare_parameter<std::string>("global_frame_id", "map");
        declare_parameter<std::string>("odom_frame_id", "odom");
        declare_parameter<std::string>("base_frame_id", "base_link");
        declare_parameter<std::string>("vicon_world_frame", "vicon_world");
        declare_parameter<std::string>("vicon_robot_frame", "vicon_robot");

        declare_parameter<double>("localization_publishing_rate", 20.0);
        declare_parameter<double>("lookup_timeout_sec", 0.05);

        declare_parameter<double>("transform_tolerance", 0.20);
        declare_parameter<double>("max_pose_age_sec", 0.5);
        declare_parameter<bool>("tf_broadcast", true);
    }

    nav2_util::CallbackReturn ViconLocalization::on_configure(const rclcpp_lifecycle::State&)
    {
        // Retrieve configuration parameters
        try
        {
            viconPose_topic_ = get_parameter("vicon_pose_topic").as_string();

            global_frame_id_ = get_parameter("global_frame_id").as_string();
            odom_frame_id_ = get_parameter("odom_frame_id").as_string();
            base_frame_id_ = get_parameter("base_frame_id").as_string();
            vicon_world_frame_ = get_parameter("vicon_world_frame").as_string();
            vicon_robot_frame_ = get_parameter("vicon_robot_frame").as_string();

            localization_publishing_rate_ = get_parameter("localization_publishing_rate").as_double();
            lookup_timeout_sec_ = get_parameter("lookup_timeout_sec").as_double();

            transform_tolerance_ = get_parameter("transform_tolerance").as_double();
            max_pose_age_sec_ = get_parameter("max_pose_age_sec").as_double();
            tf_broadcast_ = get_parameter("tf_broadcast").as_bool();

            // Check parameter consistency
            if (viconPose_topic_.empty() || global_frame_id_.empty() || odom_frame_id_.empty() ||
                base_frame_id_.empty() || vicon_world_frame_.empty() || vicon_robot_frame_.empty() ||
                global_frame_id_ == odom_frame_id_ || odom_frame_id_ == base_frame_id_ ||
                global_frame_id_ == base_frame_id_ || vicon_world_frame_ == vicon_robot_frame_ ||
                vicon_robot_frame_ == base_frame_id_ ||
                vicon_robot_frame_ == odom_frame_id_ ||
                vicon_robot_frame_ == global_frame_id_ ||
                vicon_world_frame_ == odom_frame_id_ || vicon_world_frame_ == base_frame_id_)
            {
                throw std::invalid_argument("Invalid / ambiguous TF frame or topic names");
            }
            if (!std::isfinite(localization_publishing_rate_) || localization_publishing_rate_ <= 0.0 ||
                localization_publishing_rate_ > 1000.0 ||
                !std::isfinite(lookup_timeout_sec_) || lookup_timeout_sec_<0.0 ||
                    !std::isfinite(transform_tolerance_) || transform_tolerance_<0.0 ||
                        !std::isfinite(max_pose_age_sec_) || max_pose_age_sec_ <= 0.0)
            {
                throw std::invalid_argument("Invalid loop_rate or timing parameters");
            }

            // Create TF listeners and publishers / subscriptions
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vicon_localization_pose", rclcpp::QoS(10));
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("vicon_localization_odom", rclcpp::QoS(10));
            viconPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                viconPose_topic_, rclcpp::SensorDataQoS(), std::bind(&ViconLocalization::viconPose_callback, this, std::placeholders::_1));

            // Initialize node variables
            active_.store(false);

            {
                std::lock_guard<std::mutex> lock(sample_mutex_);
                viconPose_samples_.clear();
            }

            RCLCPP_INFO(get_logger(), "Configured %s at %.2f Hz; TF %s->%s; world=%s marker=%s",
                viconPose_topic_.c_str(), localization_publishing_rate_, global_frame_id_.c_str(),
                odom_frame_id_.c_str(), vicon_world_frame_.c_str(), vicon_robot_frame_.c_str());

            return nav2_util::CallbackReturn::SUCCESS;
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(get_logger(), "Configuration failed: %s", ex.what());

            // Set the node as inactive
            active_.store(false);

            // Release all the resources
            cycle_timer_.reset();
            viconPose_sub_.reset();
            pose_pub_.reset();
            odom_pub_.reset();
            tf_broadcaster_.reset();
            tf_listener_.reset();
            tf_buffer_.reset();
            std::lock_guard<std::mutex> lock(sample_mutex_);
            viconPose_samples_.clear();

            return nav2_util::CallbackReturn::FAILURE;
        }
    }

    nav2_util::CallbackReturn ViconLocalization::on_activate(const rclcpp_lifecycle::State&)
    {
        // At every new activation initialize samples buffer and prev_pose_ variable
        {
            std::lock_guard<std::mutex> lock(sample_mutex_);
            viconPose_samples_.clear();
        }

        prev_pose_ =  geometry_msgs::msg::PoseWithCovarianceStamped{};
        prev_computed_twist_ = geometry_msgs::msg::Twist{};

        // Activate publishers
        pose_pub_->on_activate();
        odom_pub_->on_activate();

        // Set node as active
        active_.store(true);

        // Create timer for periodic loop
        cycle_timer_ = create_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::duration<double>(1.0 / localization_publishing_rate_)),
                                    std::bind(&ViconLocalization::periodic_loop, this));

        // Crete the lifecycle node bond
        createBond();

        RCLCPP_INFO(get_logger(), "Activated: periodic publish loop at %.2f Hz", localization_publishing_rate_);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn ViconLocalization::on_deactivate(const rclcpp_lifecycle::State&)
    {
        // Set the node as not activa
        active_.store(false);

        // Reset the timers and publishers/subscriptions
        cycle_timer_.reset();
        pose_pub_->on_deactivate();
        odom_pub_->on_deactivate();

        // Release the bond of the lifecycle node
        destroyBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn ViconLocalization::on_cleanup(const rclcpp_lifecycle::State&)
    {
        // Set the node as inactive
        active_.store(false);

        // Release all the resources
        cycle_timer_.reset();
        viconPose_sub_.reset();
        pose_pub_.reset();
        odom_pub_.reset();
        tf_broadcaster_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        std::lock_guard<std::mutex> lock(sample_mutex_);
        viconPose_samples_.clear();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn ViconLocalization::on_shutdown(const rclcpp_lifecycle::State&)
    {
        // Set the node as inactive
        active_.store(false);

        // Release all the resources
        cycle_timer_.reset();
        viconPose_sub_.reset();
        pose_pub_.reset();
        odom_pub_.reset();
        tf_broadcaster_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        std::lock_guard<std::mutex> lock(sample_mutex_);
        viconPose_samples_.clear();

        // Release the bond of the lifecycle node
        destroyBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    void ViconLocalization::viconPose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        // If the node is not active do nothing
        if (!active_.load())
        {
            return;
        }

        // Check the consistency of the received message
        if (msg->header.frame_id != vicon_world_frame_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Vicon pose rejected: expected header.frame_id '%s' and nonzero ROS timestamp", vicon_world_frame_.c_str());
            return;
        }

        // Check the consistency of the received pose
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const double qnorm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(qnorm2) || qnorm2 < 1.0e-12)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid Vicon pose");
            return;
        }

        // Check the time consistency of the received pose
        const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (age > max_pose_age_sec_ || age < -max_pose_age_sec_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Vicon pose rejected: clock skew/age %.3f seconds", age);
            return;
        }

        std::lock_guard<std::mutex> lock(sample_mutex_);

        // If the pose is older than the last one discard it...
        if (!viconPose_samples_.empty())
        {
            const rclcpp::Time last_stamp(viconPose_samples_.back().header.stamp, get_clock()->get_clock_type());
            if (stamp <= last_stamp)
            {
                return;
            }
        }
        // ... else store it in the buffer
        viconPose_samples_.push_back(*msg);

        // Keep the buffer size limited to 200 samples, removing the oldes samples
        if (viconPose_samples_.size() > 200)
        {
            viconPose_samples_.pop_front();
        }
    }

    void ViconLocalization::periodic_loop()
    {
        // If the node is not active do nothing
        if (!active_.load())
        {
            return;
        }

        // Create a local copy of the pose buffer, if it is empy end the loop
        std::deque<geometry_msgs::msg::PoseStamped> samples;
        {
            std::lock_guard<std::mutex> lock(sample_mutex_);
            if (viconPose_samples_.empty())
            {
                return;
            }

            samples = viconPose_samples_;
        }

        // Compute and publish the localization data
        try
        {
            // Extract static TF for world->map and base_link->vicon_robot frames
            const auto world_map_msg = tf_buffer_->lookupTransform(vicon_world_frame_, global_frame_id_,
                rclcpp::Time(0, 0, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(lookup_timeout_sec_));
            const auto base_vicon_msg = tf_buffer_->lookupTransform(base_frame_id_, vicon_robot_frame_,
                rclcpp::Time(0, 0, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(lookup_timeout_sec_));

            // Select the newest Vicon sample which has odom->base TF at its EXACT timestamp
            geometry_msgs::msg::PoseStamped sample;
            geometry_msgs::msg::TransformStamped odom_base_at_vicon_msg;
            bool found = false;

            // Go through the buffer, from the newest to the oldest sample
            for (auto it = samples.rbegin(); it != samples.rend(); ++it)
            {
                // Sample stamp and age
                const rclcpp::Time candidate_stamp(it->header.stamp, get_clock()->get_clock_type());
                const double age = (now() - candidate_stamp).seconds();

                // If the age is greater than max pose age skip all the samples, the others are older
                if (age > max_pose_age_sec_)
                {
                    break;
                }

                // If the current sample is in the future, consider an older one
                if (age < 0.0)
                {
                    continue;
                }

                // Otherwise this is the correct sample
                try
                {
                    // Look for a odom->base_link TF in the buffer at the same stamp
                    odom_base_at_vicon_msg = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, candidate_stamp,
                        rclcpp::Duration::from_seconds(0.0));

                    // If a TF exists, this is the matching we are looking for
                    sample = *it;
                    found = true;

                    break;
                }
                catch (const tf2::TransformException&)
                {
                    // A delayed odometry update may make an older Vicon sample usable
                }
            }

            // If no samples is found, exit without publishing any pose/TF
            if (!found)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "No fresh Vicon sample can be matched to odom->base TF; skipping cycle");
                return;
            }

            // Extract world->map and base_link->vicon_robot transformations from the TF messages
            tf2::Transform world_map, base_vicon, odom_base_at_vicon;
            tf2::fromMsg(world_map_msg.transform, world_map);
            tf2::fromMsg(base_vicon_msg.transform, base_vicon);
            tf2::fromMsg(odom_base_at_vicon_msg.transform, odom_base_at_vicon);

            // Extract world->vicon_robot transformation from vicon sample
            tf2::Quaternion rotation(sample.pose.orientation.x, sample.pose.orientation.y, sample.pose.orientation.z,
                sample.pose.orientation.w);
            rotation.normalize();
            const tf2::Transform world_vicon(rotation,
                tf2::Vector3(sample.pose.position.x, sample.pose.position.y, sample.pose.position.z));

            // T_map_base = inverse(T_world_map) * T_world_vicon * inverse(T_base_vicon)
            const auto map_base_at_vicon = world_map.inverse() * world_vicon * base_vicon.inverse();
            // T_map_odom = T_map_base(t) * inverse(T_odom_base(t))
            const auto map_odom = map_base_at_vicon * odom_base_at_vicon.inverse();

            // Publish the estimated pose on vicon_localization_pose
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

            pose_msg.header.frame_id = global_frame_id_;
            pose_msg.header.stamp = sample.header.stamp;

            pose_msg.pose.pose.position.x = map_base_at_vicon.getOrigin().x();
            pose_msg.pose.pose.position.y = map_base_at_vicon.getOrigin().y();
            pose_msg.pose.pose.position.z = map_base_at_vicon.getOrigin().z();

            pose_msg.pose.pose.orientation = tf2::toMsg(map_base_at_vicon.getRotation());

            for (size_t i=0; i<6; ++i)
            {
                pose_msg.pose.covariance[i*6+i] = 0.0;
            }
            pose_pub_->publish(pose_msg);

            // Publish the estimated odometry on vicon_localization_odometry
            nav_msgs::msg::Odometry odom_msg;

            odom_msg.header.frame_id = global_frame_id_;
            odom_msg.header.stamp = sample.header.stamp;

            odom_msg.child_frame_id = base_frame_id_;

            odom_msg.pose.pose.position.x = map_base_at_vicon.getOrigin().x();
            odom_msg.pose.pose.position.y = map_base_at_vicon.getOrigin().y();
            odom_msg.pose.pose.position.z = map_base_at_vicon.getOrigin().z();

            odom_msg.pose.pose.orientation = tf2::toMsg(map_base_at_vicon.getRotation());

            for (size_t i=0; i<6; ++i)
            {
                odom_msg.pose.covariance[i*6+i] = 0.0;
            }

            geometry_msgs::msg::Twist computed_twist;
            if ((prev_pose_.header.stamp.sec == 0 && prev_pose_.header.stamp.nanosec == 0) ||
                !computeTwist(prev_pose_, pose_msg, computed_twist))
            {
                // Set twist to previous one
                odom_msg.twist.twist = prev_computed_twist_;

                // Update pose history
                prev_pose_ = pose_msg;
            }
            else
            {
                odom_msg.twist.twist = computed_twist;

                // Update pose and twist history
                prev_pose_ = pose_msg;
                prev_computed_twist_ = computed_twist;
            }

            for (size_t i=0; i<6; ++i)
            {
                odom_msg.twist.covariance[i*6+i] = 0.0;
            }

            odom_pub_->publish(odom_msg);

            // Publish the estimated map->odom TF (a little bit in the future so that it is valid for more time)
            if (tf_broadcast_)
            {
                geometry_msgs::msg::TransformStamped tf_msg;

                tf_msg.header.frame_id = global_frame_id_;
                tf_msg.child_frame_id = odom_frame_id_;
                tf_msg.header.stamp = rclcpp::Time(sample.header.stamp, get_clock()->get_clock_type()) +
                    rclcpp::Duration::from_seconds(transform_tolerance_);
                tf_msg.transform = tf2::toMsg(map_odom);

                tf_broadcaster_->sendTransform(tf_msg);
            }
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No consistent TF for Vicon/odometry: %s", ex.what());
        }
    }

    bool ViconLocalization::computeTwist(const geometry_msgs::msg::PoseWithCovarianceStamped& prev,
        const geometry_msgs::msg::PoseWithCovarianceStamped& curr, geometry_msgs::msg::Twist& twist)
    {
        // Verify that both poses use the same reference frame
        if (prev.header.frame_id != curr.header.frame_id)
        {
            return false;
        }

        // Compute time difference
        const double dt = (rclcpp::Time(curr.header.stamp) - rclcpp::Time(prev.header.stamp)).seconds();

        if (!std::isfinite(dt) || dt <= 0.0)
        {
            return false;
        }

        // Extract quaternions
        tf2::Quaternion q0, q1;

        tf2::fromMsg(prev.pose.pose.orientation, q0);
        tf2::fromMsg(curr.pose.pose.orientation, q1);

        if (q0.length2() < 1e-12 || q1.length2() < 1e-12)
        {
            return false;
        }

        q0.normalize();
        q1.normalize();

        // Compute linear velocity in the global frame
        tf2::Vector3 v_global(
            (curr.pose.pose.position.x - prev.pose.pose.position.x) / dt,
            (curr.pose.pose.position.y - prev.pose.pose.position.y) / dt,
            (curr.pose.pose.position.z - prev.pose.pose.position.z) / dt
        );

        // Express linear velocity in the body frame
        tf2::Vector3 v_body = tf2::quatRotate(q1.inverse(), v_global);

        // Compute relative rotation
        tf2::Quaternion dq = q0.inverse() * q1;
        dq.normalize();

        // Select the shortest rotation
        if (dq.w() < 0.0)
        {
            dq = tf2::Quaternion(
                -dq.x(), -dq.y(), -dq.z(), -dq.w());
        }

        // Compute angular velocity
        const double s = std::sqrt(
            dq.x() * dq.x() +
            dq.y() * dq.y() +
            dq.z() * dq.z()
        );

        tf2::Vector3 omega(0.0, 0.0, 0.0);

        if (s > 1e-12)
        {
            const double angle = 2.0 * std::atan2(s, dq.w());
            const double k = angle / (s * dt);

            omega = tf2::Vector3(
                k * dq.x(),
                k * dq.y(),
                k * dq.z()
            );
        }

        // Populate the twist message
        twist = geometry_msgs::msg::Twist{};

        twist.linear.x = v_body.x();
        twist.linear.y = v_body.y();
        twist.linear.z = v_body.z();

        twist.angular.x = omega.x();
        twist.angular.y = omega.y();
        twist.angular.z = omega.z();

        return true;
    }
} // namespace vicon_localization
