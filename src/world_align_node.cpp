#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class WorldAlignNode final : public rclcpp::Node
{
public:
    WorldAlignNode()
        : rclcpp::Node{"libsurvive_world_align_node"},
          joy_topic_{declare_parameter<std::string>("joy_topic", "/libsurvive/joy")},
          parent_frame_{"libsurvive_world"},
          world_frame_{declare_parameter<std::string>("world_frame", "world")},
          lookup_timeout_{
              rclcpp::Duration::from_seconds(declare_parameter<double>("lookup_timeout_sec", 0.1))},
          tf_buffer_{get_clock()},
          tf_listener_{tf_buffer_},
          static_broadcaster_{std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)}
    {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { OnJoy(msg); });
        tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            tf2_ros::DynamicListenerQoS(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg); });
        auto_align_timer_ =
            create_wall_timer(std::chrono::milliseconds(100), [this]() { TryAutoAlign(); });

        RCLCPP_INFO(get_logger(),
                    "World align node started. joy_topic=%s parent_frame=%s world_frame=%s "
                    "button_index=%d",
                    joy_topic_.c_str(),
                    parent_frame_.c_str(),
                    world_frame_.c_str(),
                    button_index_);
    }

private:
    bool AlignFromTracker(const std::string &tracker_frame, const rclcpp::Time &stamp,
                          const char *reason, bool warn_on_failure)
    {
        try
        {
            const auto tf_msg =
                tf_buffer_.lookupTransform(parent_frame_, tracker_frame, stamp, lookup_timeout_);

            tf2::Transform t_lt;
            tf2::fromMsg(tf_msg.transform, t_lt);
            const tf2::Transform t_lw = TrackerToWorldTransform(t_lt);
            const tf2::Transform t_wl = t_lw.inverse();

            geometry_msgs::msg::TransformStamped world_to_lib;
            world_to_lib.header.stamp = tf_msg.header.stamp;
            world_to_lib.header.frame_id = world_frame_;
            world_to_lib.child_frame_id = parent_frame_;
            world_to_lib.transform = tf2::toMsg(t_wl);
            static_broadcaster_->sendTransform(world_to_lib);

            RCLCPP_INFO(
                get_logger(), "Aligned world frame using %s (%s)", tracker_frame.c_str(), reason);
            auto_align_done_ = true;
            if (auto_align_timer_)
            {
                auto_align_timer_->cancel();
            }
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            if (warn_on_failure)
            {
                RCLCPP_WARN(get_logger(), "Failed to align world frame: %s", ex.what());
            }
        }
        return false;
    }

    static tf2::Transform TrackerToWorldTransform(const tf2::Transform &tracker_wrt_libsurvive)
    {
        // Keep world z aligned with libsurvive z, and define heading from tracker-derived
        // direction projected onto the xy plane.
        constexpr double kMinNorm = 1e-6;

        tf2::Vector3 heading_wrt_libsurvive =
            tf2::quatRotate(tracker_wrt_libsurvive.getRotation(), tf2::Vector3(0.0, -1.0, 0.0));
        heading_wrt_libsurvive.setZ(0.0);
        if (heading_wrt_libsurvive.length2() < kMinNorm)
        {
            heading_wrt_libsurvive =
                tf2::quatRotate(tracker_wrt_libsurvive.getRotation(), tf2::Vector3(-1.0, 0.0, 0.0));
            heading_wrt_libsurvive.setZ(0.0);
        }
        if (heading_wrt_libsurvive.length2() < kMinNorm)
        {
            heading_wrt_libsurvive = tf2::Vector3(1.0, 0.0, 0.0);
        }
        heading_wrt_libsurvive.normalize();

        const double yaw = std::atan2(heading_wrt_libsurvive.y(), heading_wrt_libsurvive.x());
        tf2::Quaternion world_quat_wrt_libsurvive;
        world_quat_wrt_libsurvive.setRPY(0.0, 0.0, yaw);
        world_quat_wrt_libsurvive.normalize();
        return tf2::Transform(world_quat_wrt_libsurvive, tracker_wrt_libsurvive.getOrigin());
    }

    void OnJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->header.frame_id.empty() || button_index_ < 0 ||
            static_cast<size_t>(button_index_) >= msg->buttons.size())
        {
            return;
        }

        const bool pressed = (msg->buttons[button_index_] != 0);
        bool &prev_pressed = last_button_state_[msg->header.frame_id];
        if (!pressed || prev_pressed)
        {
            prev_pressed = pressed;
            return;
        }
        prev_pressed = true;

        const rclcpp::Time click_time =
            msg->header.stamp.nanosec == 0 && msg->header.stamp.sec == 0
                ? now()
                : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

        if (pending_manual_align_tracker_ != msg->header.frame_id ||
            click_time - pending_manual_align_last_click_time_ > manual_align_click_gate_)
        {
            pending_manual_align_tracker_ = msg->header.frame_id;
            pending_manual_align_click_count_ = 1;
        }
        else
        {
            ++pending_manual_align_click_count_;
        }
        pending_manual_align_last_click_time_ = click_time;

        if (pending_manual_align_click_count_ < manual_align_required_clicks_)
        {
            RCLCPP_INFO(get_logger(),
                        "Manual world align armed for %s: %d/%d clicks within %.1f sec",
                        pending_manual_align_tracker_.c_str(),
                        pending_manual_align_click_count_,
                        manual_align_required_clicks_,
                        manual_align_click_gate_.seconds());
            return;
        }

        pending_manual_align_tracker_.clear();
        pending_manual_align_click_count_ = 0;
        AlignFromTracker(msg->header.frame_id, click_time, "manual joy", true);
    }

    void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        if (auto_align_done_ || !auto_align_tracker_frame_.empty())
        {
            return;
        }

        for (const auto &transform : msg->transforms)
        {
            if (transform.header.frame_id != parent_frame_ || transform.child_frame_id.empty())
            {
                continue;
            }

            auto_align_tracker_frame_ = transform.child_frame_id;
            RCLCPP_INFO(get_logger(),
                        "Selected %s for initial automatic world alignment",
                        auto_align_tracker_frame_.c_str());
            return;
        }
    }

    void TryAutoAlign()
    {
        if (auto_align_done_ || auto_align_tracker_frame_.empty())
        {
            return;
        }

        const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
        AlignFromTracker(auto_align_tracker_frame_, latest_time, "initial auto align", false);
    }

    std::string joy_topic_;
    std::string parent_frame_;
    std::string world_frame_;
    rclcpp::Duration lookup_timeout_;
    const int button_index_{3};
    const int manual_align_required_clicks_{3};
    const rclcpp::Duration manual_align_click_gate_{rclcpp::Duration::from_seconds(1.0)};

    bool auto_align_done_{false};
    std::string auto_align_tracker_frame_;
    std::string pending_manual_align_tracker_;
    rclcpp::Time pending_manual_align_last_click_time_{0, 0, get_clock()->get_clock_type()};
    int pending_manual_align_click_count_{0};
    std::unordered_map<std::string, bool> last_button_state_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::TimerBase::SharedPtr auto_align_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldAlignNode>());
    rclcpp::shutdown();
    return 0;
}
