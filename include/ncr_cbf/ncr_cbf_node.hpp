#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

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

    // parameters
    void load_individual_parameters(std::string robot_name, std::size_t robot_idx);
    void load_pairwise_parameters(std::string robot_pair_name);

    // variable storage
    std::map<std::string, rclcpp::Time> pose_stamps;
    std::map<std::string, rclcpp::Time> unsafe_cmd_vel_stamps;
    std::map<std::string, Eigen::Vector3d> positions;
    std::map<std::string, Eigen::Vector3d> unsafe_cmd_vels;
    std::map<std::string, Eigen::Vector3d> safe_cmd_vels;

    // callbacks
    void pose_callback(std::string robot_name, const geometry_msgs::msg::PoseStamped msg);
    void unsafe_cmd_vel_callback(std::string robot_name, const geometry_msgs::msg::TwistStamped msg);
};
