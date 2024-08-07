#include <memory>
#include <cmath>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DetectObjectsNode : public rclcpp::Node
{
public:
    DetectObjectsNode()
    : Node("detect_objects_node")
    {
        gnss_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sensing/gnss/pose", 10, std::bind(&DetectObjectsNode::gnss_callback, this, std::placeholders::_1));

        objects_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/aichallenge/objects", 10, std::bind(&DetectObjectsNode::objects_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt8>("/detect_objects", 10);
    }

private:
    void gnss_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose;
        evaluate_objects();
    }

    void objects_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        objects_ = msg->data;
        evaluate_objects();
    }

    void evaluate_objects()
    {
        if (!current_pose_ || objects_.empty()) {
            return;
        }

        double yaw = quaternion_to_yaw(current_pose_->orientation);
        std::pair<double, double> front_direction = {std::cos(yaw), std::sin(yaw)};

        int left_count = 0;
        int right_count = 0;

        for (size_t i = 0; i < objects_.size(); i += 4) {
            double obj_x = objects_[i];
            double obj_y = objects_[i + 1];
            double ojb_z = objects_[i + 2];
            double ojb_radius = objects_[i + 3];

            std::pair<double, double> vec_to_obj = {obj_x - current_pose_->position.x, obj_y - current_pose_->position.y};
            double distance_to_obj = std::sqrt(vec_to_obj.first * vec_to_obj.first + vec_to_obj.second * vec_to_obj.second);

            if (distance_to_obj <= 5.0) {
                std::pair<double, double> norm_vec_to_obj = normalize_vector(vec_to_obj);
                double cross_product_result = this->cross_product(front_direction, norm_vec_to_obj);
                if (cross_product_result > 0) {
                    left_count += 1;
                } else if (cross_product_result < 0) {
                    right_count += 1;
                }
            }
        }

        std_msgs::msg::UInt8 result;
        if (left_count == 0 && right_count == 0) {
            result.data = 0;
        } else if (left_count >= right_count) {
            result.data = 1;
        } else {
            result.data = 2;
        }

        // ROS2のログ機能を使用して結果を表示
        RCLCPP_INFO(this->get_logger(), "Detected objects result: %u", result.data);

        publisher_->publish(result);
    }

    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    std::pair<double, double> normalize_vector(const std::pair<double, double> &vec)
    {
        double length = std::sqrt(vec.first * vec.first + vec.second * vec.second);
        return {vec.first / length, vec.second / length};
    }

    double cross_product(const std::pair<double, double> &vec1, const std::pair<double, double> &vec2)
    {
        return vec1.first * vec2.second - vec1.second * vec2.first;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr objects_subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;

    std::optional<geometry_msgs::msg::Pose> current_pose_;
    std::vector<double> objects_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectObjectsNode>());
    rclcpp::shutdown();
    return 0;
}
