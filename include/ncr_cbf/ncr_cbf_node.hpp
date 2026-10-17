#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ncr_cbf/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <Eigen/Core>

class NcrCbfNode : public rclcpp::Node
{
public:
    NcrCbfNode();

private:
    // populate with parameters
    std::string location;
    std::vector<std::string> robot_names;
    int cbf_rate_hz;
    double pose_timeout_sec;
    double cmd_vel_timeout_sec;
    std::vector<std::string> robot_pair_names;
    std::map<
        std::string, std::size_t>
        robot_index_lookup;
    ncr_cbf::RoomGeometry room_geometry;
    std::size_t num_agents;
    std::vector<std::size_t> room_agents;
    std::vector<std::size_t> saturation_agents;
    std::vector<std::pair<std::size_t, std::size_t>> pairwise_pairs;
    std::map<
        std::size_t, double>
        robot_radii;
    std::map<
        std::size_t, double>
        room_kappas;
    std::map<std::size_t, ncr_cbf::RoomFaces> room_enabled_faces;
    std::map<
        std::pair<
            std::size_t, std::size_t>,
        double>
        pairwise_kappas;
    std::map<
        std::pair<
            std::size_t, std::size_t>,
        double>
        pairwise_safety_buffers;
    std::map<
        std::size_t, Eigen::Vector3d>
        componentwise_vel_bounds;

    // subscribers
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs;            // agent pose
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> unsafe_cmd_vel_subs; // unsafe control input

    // publishers
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> safe_cmd_vel_pubs; // safe control input

    // utils
    bool check_if_duplicates(std::vector<std::string> input_vector);
    void check_pair_formatting(std::string input_pair);
    void publish_zeros(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub);
    Eigen::Vector3d convert_to_body_frame(Eigen::Quaterniond quaternion, Eigen::Vector3d world_frame_vel);
    void cbf_loop();

    // parameters
    void load_individual_parameters(std::string robot_name, std::size_t robot_idx);
    void load_pairwise_parameters(std::string robot_pair_name);

    // variable storage
    bool init_pose_checked_;
    bool all_init_poses_recieved_;
    bool all_init_unsafe_cmd_vels_recieved_;
    std::map<std::string, bool> poses_recieved_;
    std::map<std::string, rclcpp::Time> pose_time_stamps_;
    std::map<std::string, Eigen::Vector3d> positions_;
    std::map<std::string, bool> unsafe_cmd_vels_recieved_;
    std::map<std::string, rclcpp::Time> unsafe_cmd_vel_time_stamps_;
    std::map<std::string, Eigen::Vector3d> unsafe_linear_cmd_vels_;
    std::map<std::string, Eigen::Vector3d> unsafe_angular_cmd_vels_;
    std::map<std::string, Eigen::Vector3d> safe_cmd_vels_;
    std::map<std::string, Eigen::Quaterniond> pose_quaternions_;

    // callbacks
    rclcpp::TimerBase::SharedPtr timer_callback;
    void pose_callback(std::string robot_name, const geometry_msgs::msg::PoseStamped &msg);
    void unsafe_cmd_vel_callback(std::string robot_name, const geometry_msgs::msg::TwistStamped &msg);
};
