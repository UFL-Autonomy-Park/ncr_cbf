#include <ncr_cbf/ncr_cbf_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ncr_cbf/construct_constraints.hpp"
#include "ncr_cbf/room.hpp"
#include "ncr_cbf/solver_wrapper.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

NcrCbfNode::NcrCbfNode() : Node("ncr_cbf_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing NCR CBF node");

    // declare global parameters
    this->declare_parameter("location", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("robot_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    this->declare_parameter("cbf_rate_hz", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("pose_timeout_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("cmd_vel_timeout_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("robot_pair_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    this->declare_parameter("room_geometry.x_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("room_geometry.x_max", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("room_geometry.y_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("room_geometry.y_max", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("room_geometry.z_min", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("room_geometry.z_max", rclcpp::ParameterType::PARAMETER_DOUBLE);

    // get global parameters
    this->get_parameter("location", location);
    this->get_parameter("robot_names", robot_names);
    this->get_parameter("cbf_rate_hz", cbf_rate_hz);
    this->get_parameter("pose_timeout_sec", pose_timeout_sec);
    this->get_parameter("cmd_vel_timeout_sec", cmd_vel_timeout_sec);
    this->get_parameter("robot_pair_names", robot_pair_names);
    this->get_parameter("room_geometry.x_min", room_geometry.x_min);
    this->get_parameter("room_geometry.x_max", room_geometry.x_max);
    this->get_parameter("room_geometry.y_min", room_geometry.y_min);
    this->get_parameter("room_geometry.y_max", room_geometry.y_max);
    this->get_parameter("room_geometry.z_min", room_geometry.z_min);
    this->get_parameter("room_geometry.z_max", room_geometry.z_max);

    // verify cbf_rate_hz
    if (!(cbf_rate_hz > 0) || !std::isfinite(cbf_rate_hz))
    {
        throw std::invalid_argument("Invalid cbf_rate_hz\n`");
    }

    // init checks
    init_pose_checked_ = false;
    all_init_poses_recieved_ = false;
    all_init_unsafe_cmd_vels_recieved_ = false;
    for (auto name : robot_names)
    {
        poses_recieved_.emplace(name, false);
        unsafe_cmd_vels_recieved_.emplace(name, false);
    }

    // TO-DO: add check while uses known dimensions of autonomy park and indoor lab to rule out misconfigured inputs
    // e.g., if you put apark bounds in your config for the indoor lab 
    if (location != "indoor_lab" && location != "autonomy_park")
    {
        throw std::invalid_argument("Location must be `indoor_lab` or `autonomy_park\n`");
    }

    // set num_agents
    if (robot_names.size() < 1)
    {
        throw std::invalid_argument("Must specify at least 1 robot\n");
    }
    num_agents = robot_names.size();

    // check for duplicates in robot_names and robot_pair_names
    if (check_if_duplicates(robot_names))
    {
        throw std::invalid_argument("Duplicated entry in robot_names\n");
    }
    if (check_if_duplicates(robot_pair_names))
    {
        throw std::invalid_argument("Duplicated entry in robot_pair_names\n");
    }

    sort(robot_names.begin(), robot_names.end());
    auto names_it = unique(robot_names.begin(), robot_names.end());
    robot_names.erase(names_it, robot_names.end());

    // robot names to indices, populate lookup
    // NOTE: this is where the code is the most inconsistent. I wrote the CBF library to take maps keyed with ints, but then found when writing this node that using the strings was more intuitive
    // TO-DO: make the CBF library use strings, not ints! 
    for (std::size_t robot_idx = 0; robot_idx < robot_names.size(); ++robot_idx)
    {
        robot_index_lookup.emplace(robot_names[robot_idx], robot_idx);
        load_individual_parameters(robot_names[robot_idx], robot_idx);
    }
    RCLCPP_INFO(this->get_logger(), "Individual parameters loaded successfully");
    for (auto pair : robot_pair_names)
    {
        load_pairwise_parameters(pair);
    }
    RCLCPP_INFO(this->get_logger(), "Pairwise parameters loaded successfully");

    // subscribers
    for (const auto &robot_name : robot_names)
    {
        // pose
        std::string pose_topic;
        this->declare_parameter("robots." + robot_name + ".pose_topic", rclcpp::ParameterType::PARAMETER_STRING);
        if (!this->get_parameter("robots." + robot_name + ".pose_topic", pose_topic))
        {
            throw std::invalid_argument("Parameter robots." + robot_name + ".pose_topic" + " undefined or invalid\n");
        }
        pose_subs[robot_name] = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            [this, robot_name](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            { pose_callback(robot_name, *msg); });

        // unsafe cmd_vel
        std::string unsafe_cmd_vel_topic;
        this->declare_parameter("robots." + robot_name + ".unsafe_cmd_vel_topic", rclcpp::ParameterType::PARAMETER_STRING);
        if (!this->get_parameter("robots." + robot_name + ".unsafe_cmd_vel_topic", unsafe_cmd_vel_topic))
        {
            throw std::invalid_argument("Parameter robots." + robot_name + ".unsafe_cmd_vel_topic" + " undefined or invalid\n");
        }
        unsafe_cmd_vel_subs[robot_name] = create_subscription<geometry_msgs::msg::TwistStamped>(
            unsafe_cmd_vel_topic, 10, [this, robot_name](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
            { unsafe_cmd_vel_callback(robot_name, *msg); });
    }

    // publishers
    for (const auto &robot_name : robot_names)
    {
        // safe_cmd_vel
        std::string safe_cmd_vel_topic;
        this->declare_parameter("robots." + robot_name + ".safe_cmd_vel_topic", rclcpp::ParameterType::PARAMETER_STRING);
        if (!this->get_parameter("robots." + robot_name + ".safe_cmd_vel_topic", safe_cmd_vel_topic))
        {
            throw std::invalid_argument("Parameter robots." + robot_name + ".safe_cmd_vel_topic" + " undefined or invalid\n");
        }
        safe_cmd_vel_pubs[robot_name] = create_publisher<geometry_msgs::msg::Twist>(
            safe_cmd_vel_topic, 10);
    }

    // timer callback
    timer_callback = create_wall_timer(
        std::chrono::duration<double>(1.0 / cbf_rate_hz),
        [this]
        { cbf_loop(); });
}

bool NcrCbfNode::check_if_duplicates(std::vector<std::string> input_vector)
{
    std::vector<std::string> input_vector_copy = input_vector;

    // sort both
    sort(input_vector.begin(), input_vector.end());
    sort(input_vector_copy.begin(), input_vector_copy.end());

    // remove duplicates and compare to original
    auto it = unique(input_vector_copy.begin(), input_vector_copy.end());
    input_vector_copy.erase(it, input_vector_copy.end());

    if (input_vector_copy.size() == input_vector.size())
    {
        return false;
    }
    else
    {
        return true;
    }
}

void NcrCbfNode::check_pair_formatting(std::string input_pair)
{
    std::size_t underscore_count = 0;
    std::vector<std::size_t> underscore_idxs;
    bool last_underscore = false;

    while (not last_underscore)
    {
        std::size_t underscore_idx;
        if (underscore_count == 0)
        {
            underscore_idx = input_pair.find('_');
        }
        else
        {
            underscore_idx = input_pair.find('_', underscore_idxs.back() + 1);
        }
        if (underscore_idx == std::string::npos)
        {
            last_underscore = true;
        }
        else
        {
            underscore_count = underscore_count + 1;
            underscore_idxs.emplace_back(underscore_idx);
        }
    }

    std::size_t begin_cnt = count(underscore_idxs.begin(), underscore_idxs.end(), 0);
    std::size_t end_cnt = count(underscore_idxs.begin(), underscore_idxs.end(), input_pair.size() - 1);

    if (underscore_count == 0)
    {
        throw std::invalid_argument("Must be at least one underscore\n");
    }
    if (underscore_count > 1)
    {
        throw std::invalid_argument("Must only be one underscore\n");
    }
    if (begin_cnt > 0)
    {
        throw std::invalid_argument("Underscore must not be at beginning of string\n");
    }
    if (end_cnt > 0)
    {
        throw std::invalid_argument("Underscore must not be at the end of string\n");
    }
}

void NcrCbfNode::load_individual_parameters(std::string robot_name, std::size_t robot_idx)
{
    // using name and index, append values to each parameter map
    const std::string prefix = "robots." + robot_name + ".";

    // convert_to_body_frame
    const std::string body_string = prefix + "convert_to_body_frame";
    this->declare_parameter(body_string, rclcpp::ParameterType::PARAMETER_BOOL);
    bool body_bool;
    if (!this->get_parameter(body_string, body_bool))
    {
        throw std::invalid_argument("Parameter " + body_string + " undefined or invalid\n");
    }
    convert_to_body_frame_bools.emplace(robot_idx, body_bool);

    // radius
    const std::string radius_string = prefix + "radius";
    this->declare_parameter(radius_string, rclcpp::ParameterType::PARAMETER_DOUBLE);
    double robot_radius;
    if (!this->get_parameter(radius_string, robot_radius))
    {
        throw std::invalid_argument("Parameter " + radius_string + " undefined or invalid\n");
    }
    robot_radii.emplace(robot_idx, robot_radius);

    // room constraints
    bool room_bool{false};
    const std::string room_string = prefix + "room_constraints";
    this->declare_parameter(room_string + ".enabled", rclcpp::ParameterType::PARAMETER_BOOL);
    this->get_parameter(room_string + ".enabled", room_bool);
    if (room_bool)
    {
        // save to vector
        room_agents.emplace_back(robot_idx);
        // save to map
        double room_kappa;
        this->declare_parameter(room_string + ".kappa", rclcpp::ParameterType::PARAMETER_DOUBLE);
        if (!this->get_parameter(room_string + ".kappa", room_kappa))
        {
            throw std::invalid_argument("Parameter " + room_string + ".kappa" + " undefined or invalid\n");
        }
        room_kappas.emplace(robot_idx, room_kappa);

        const std::string faces_parameter = room_string + ".enabled_faces";
        this->declare_parameter(faces_parameter,
            std::vector<std::string>{"x_min", "x_max", "y_min", "y_max", "z_min", "z_max"});
        std::vector<std::string> face_names;
        this->get_parameter(faces_parameter, face_names);
        ncr_cbf::RoomFaces faces{};
        for (const auto &face_name : face_names)
        {
            bool recognized = false;
            for (std::size_t face_idx = 0; face_idx < ncr_cbf::room_face_names.size(); ++face_idx)
            {
                if (face_name == ncr_cbf::room_face_names[face_idx])
                {
                    if (faces[face_idx])
                    {
                        throw std::invalid_argument("Duplicate face " + face_name + " in " + faces_parameter);
                    }
                    faces[face_idx] = true;
                    recognized = true;
                    break;
                }
            }
            if (!recognized)
            {
                throw std::invalid_argument("Unknown face " + face_name + " in " + faces_parameter);
            }
        }
        room_enabled_faces.emplace(robot_idx, faces);
    }
    // saturation constraints
    bool sat_bool{false};
    const std::string sat_string = prefix + "saturation_constraints";
    this->declare_parameter(sat_string + ".enabled", rclcpp::ParameterType::PARAMETER_BOOL);
    this->get_parameter(sat_string + ".enabled", sat_bool);
    if (sat_bool)
    {
        // save to vector
        saturation_agents.emplace_back(robot_idx);
        // save to map
        std::vector<double> componentwise_vel_bound;
        this->declare_parameter(sat_string + ".componentwise_vel_bound", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        if (!this->get_parameter(sat_string + ".componentwise_vel_bound", componentwise_vel_bound))
        {
            throw std::invalid_argument("Parameter " + sat_string + ".componentwise_vel_bound" + " undefined or invalid\n");
        }
        if (componentwise_vel_bound.size() != 3)
        {
            throw std::invalid_argument("componentwise_vel_bound for " + robot_name + " must have an x, y, and z component\n");
        }
        componentwise_vel_bounds.emplace(robot_idx, Eigen::Vector3d(componentwise_vel_bound[0], componentwise_vel_bound[1], componentwise_vel_bound[2]));
    }
}

void NcrCbfNode::load_pairwise_parameters(std::string robot_pair_name)
{
    // using name and index, append values to each parameter map
    const std::string prefix = "robot_pairs." + robot_pair_name + ".";

    // check pair formatting
    check_pair_formatting(robot_pair_name);
    // split at underscore
    std::size_t underscore_idx = robot_pair_name.find('_');
    std::string robot_1 = robot_pair_name.substr(0, underscore_idx);
    std::string robot_2 = robot_pair_name.substr(underscore_idx + 1);
    // make sure both are valid robots
    std::size_t robot_1_cnt = count(robot_names.begin(), robot_names.end(), robot_1);
    std::size_t robot_2_cnt = count(robot_names.begin(), robot_names.end(), robot_2);
    if (robot_1_cnt == 0)
    {
        throw std::invalid_argument("Invalid robot name " + robot_1 + " in pair " + robot_pair_name + "\n");
    }
    if (robot_2_cnt == 0)
    {
        throw std::invalid_argument("Invalid robot name " + robot_2 + " in pair " + robot_pair_name + "\n");
    }

    // enforce index order
    std::size_t robot_1_idx = robot_index_lookup[robot_1];
    std::size_t robot_2_idx = robot_index_lookup[robot_2];
    if (robot_1_idx >= robot_2_idx)
    {
        throw std::invalid_argument("Invalid pair order: First robot must have a lower index than the second in the pair\n");
    }

    // assign variables
    std::pair<std::size_t, std::size_t> robot_pair_idxs = {robot_1_idx, robot_2_idx};
    // pairwise_pairs
    pairwise_pairs.emplace_back(robot_pair_idxs);
    // kappas
    double pairwise_kappa;
    this->declare_parameter(prefix + "kappa", rclcpp::ParameterType::PARAMETER_DOUBLE);
    if (!this->get_parameter(prefix + "kappa", pairwise_kappa))
    {
        throw std::invalid_argument("Parameter " + prefix + "kappa" + " undefined or invalid\n");
    }
    pairwise_kappas.emplace(robot_pair_idxs, pairwise_kappa);
    // safety_buffers
    double pairwise_safety_buffer;
    this->declare_parameter(prefix + "safety_buffer", rclcpp::ParameterType::PARAMETER_DOUBLE);
    if (!this->get_parameter(prefix + "safety_buffer", pairwise_safety_buffer))
    {
        throw std::invalid_argument("Parameter " + prefix + "safety_buffer" + " undefined or invalid\n");
    }
    pairwise_safety_buffers.emplace(robot_pair_idxs, pairwise_safety_buffer);
}

void NcrCbfNode::publish_zeros(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub)
{
    geometry_msgs::msg::Twist zero_twist;
    zero_twist.linear.x = 0.0;
    zero_twist.linear.y = 0.0;
    zero_twist.linear.z = 0.0;
    zero_twist.angular.x = 0.0;
    zero_twist.angular.y = 0.0;
    zero_twist.angular.z = 0.0;

    safe_cmd_vel_pub->publish(zero_twist);
}

Eigen::Vector3d NcrCbfNode::convert_to_body_frame(Eigen::Quaterniond quaternion, Eigen::Vector3d world_frame_vel)
{
    quaternion.normalize();
    Eigen::Vector3d body_frame_vel = quaternion.inverse() * world_frame_vel;
    return body_frame_vel;
}

void NcrCbfNode::cbf_loop()
{
    // init checks
    if (!all_init_poses_recieved_ || !all_init_unsafe_cmd_vels_recieved_)
    {
        // check if each pose has been receieved
        if (!all_init_poses_recieved_)
        {
            bool pose_candidate = true;
            for (auto robot_name : robot_names)
            {
                if (poses_recieved_[robot_name] == false)
                {
                    pose_candidate = false;
                }
            }
            all_init_poses_recieved_ = pose_candidate;
        }

        // check if each cmd_vel has been recieved
        if (!all_init_unsafe_cmd_vels_recieved_)
        {
            bool cmd_vel_candidate = true;
            for (auto robot_name : robot_names)
            {
                if (unsafe_cmd_vels_recieved_[robot_name] == false)
                {
                    cmd_vel_candidate = false;
                }
            }
            all_init_unsafe_cmd_vels_recieved_ = cmd_vel_candidate;
        }

        // publish zeros to all and return
        for (auto robot_name : robot_names)
        {
            publish_zeros(safe_cmd_vel_pubs[robot_name]);
        }
        return;
    }
    if (!init_pose_checked_)
    {
        // room constraints
        for (auto robot_name : robot_names)
        {
            bool room_bool = false;
            this->get_parameter("robots." + robot_name + ".room_constraints.enabled", room_bool);
            if (room_bool)
            {
                const std::size_t robot_idx = robot_index_lookup.at(robot_name);
                const auto &faces = room_enabled_faces.at(robot_idx);
                const auto room_constraints = ncr_cbf::constructRoomConstraints(
                    room_geometry, positions_.at(robot_name), robot_radii.at(robot_idx),
                    room_kappas.at(robot_idx), faces);
                for (std::size_t face_idx = 0; face_idx < faces.size(); ++face_idx)
                {
                    if (faces[face_idx] && room_constraints.upper_bounds(face_idx) <= 0.0)
                    {
                        RCLCPP_FATAL(this->get_logger(), "Room face %s violated on initialization for %s",
                            ncr_cbf::room_face_names[face_idx], robot_name.c_str());
                        throw std::runtime_error("Room face " + std::string(ncr_cbf::room_face_names[face_idx]) +
                            " violated on initialization for " + robot_name);
                    }
                }
            }
        }

        // pairwise constraints
        for (auto robot_pair_name : robot_pair_names)
        {
            // split at underscore
            std::size_t underscore_idx = robot_pair_name.find('_');
            std::string robot_1 = robot_pair_name.substr(0, underscore_idx);
            std::string robot_2 = robot_pair_name.substr(underscore_idx + 1);
            std::size_t robot_1_idx = robot_index_lookup[robot_1];
            std::size_t robot_2_idx = robot_index_lookup[robot_2];

            // compute distance between robots and check if it is smaller than sum of individual radii
            double robot_1_radius = robot_radii[robot_1_idx];
            double robot_2_radius = robot_radii[robot_2_idx];
            double distance = (positions_[robot_1] - positions_[robot_2]).norm();
            if (distance <= robot_1_radius + robot_2_radius)
            {
                RCLCPP_FATAL(this->get_logger(), "Pairwise constraints violated on initialization for %s", robot_pair_name.c_str());
                throw std::runtime_error("Pairwise constraints violated on initialization for " + robot_pair_name);
            }
        }

        init_pose_checked_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose validated against constraints");
        // publish zeros to all and return
        for (auto robot_name : robot_names)
        {
            publish_zeros(safe_cmd_vel_pubs[robot_name]);
        }
        return;
    }

    if (init_pose_checked_ && all_init_poses_recieved_ && all_init_unsafe_cmd_vels_recieved_)
    {
        // timeouts first, if one robot is stale send zeros to all
        for (auto robot_name : robot_names)
        {
            // pose
            if (this->now() - pose_time_stamps_[robot_name] >= rclcpp::Duration::from_seconds(pose_timeout_sec))
            {
                RCLCPP_WARN(this->get_logger(), "Stale pose data for %s", robot_name.c_str());
                for (auto robot_name : robot_names)
                {
                    publish_zeros(safe_cmd_vel_pubs[robot_name]);
                }
                return;
            }
            // cmd_vel
            if (this->now() - unsafe_cmd_vel_time_stamps_[robot_name] >= rclcpp::Duration::from_seconds(cmd_vel_timeout_sec))
            {
                RCLCPP_WARN(this->get_logger(), "Stale cmd_vel data for %s", robot_name.c_str());
                for (auto robot_name : robot_names)
                {
                    publish_zeros(safe_cmd_vel_pubs[robot_name]);
                }
                return;
            }
        }

        // construct global constraints
        std::map<std::size_t, Eigen::Vector3d> agent_positions;
        std::map<std::size_t, Eigen::Vector3d> unsafe_linear_velocities;

        for (const auto &[robot_name, index] : robot_index_lookup)
        {
            agent_positions.emplace(index, positions_.at(robot_name));
            unsafe_linear_velocities.emplace(index, unsafe_linear_cmd_vels_.at(robot_name));
        }

        auto constraints = ncr_cbf::constructGlobalConstraints(
            num_agents, room_agents, saturation_agents, pairwise_pairs,
            room_geometry, robot_radii, pairwise_safety_buffers,
            room_kappas, room_enabled_faces, pairwise_kappas, componentwise_vel_bounds,
            agent_positions);

        auto [safe_linear_velocities, status] =
            ncr_cbf::solveQP(unsafe_linear_velocities, constraints);

        if (status != ncr_cbf::SolverStatus::SOLVED)
        {
            RCLCPP_ERROR(this->get_logger(), "CBF solver failed with status %d; publishing zero velocities", static_cast<int>(status));
            // publish zeros and return
            for (auto robot_name : robot_names)
            {
                publish_zeros(safe_cmd_vel_pubs[robot_name]);
            }
            return;
        }

        // check if convert_to_body_frame and publish safe cmd_vel
        for (const auto &[robot_name, index] : robot_index_lookup)
        {
            Eigen::Vector3d safe_linear_velocity = safe_linear_velocities[index];
            Eigen::Vector3d safe_angular_velocity = unsafe_angular_cmd_vels_[robot_name]; // passing this through for now
            if (convert_to_body_frame_bools[index])
            {
                safe_linear_velocity = convert_to_body_frame(pose_quaternions_[robot_name], safe_linear_velocity);
                safe_angular_velocity = convert_to_body_frame(pose_quaternions_[robot_name], safe_angular_velocity);
            }
            geometry_msgs::msg::Twist safe_cmd_vel;
            safe_cmd_vel.linear.x = safe_linear_velocity[0];
            safe_cmd_vel.linear.y = safe_linear_velocity[1];
            safe_cmd_vel.linear.z = safe_linear_velocity[2];
            safe_cmd_vel.angular.x = safe_angular_velocity[0];
            safe_cmd_vel.angular.y = safe_angular_velocity[1];
            safe_cmd_vel.angular.z = safe_angular_velocity[2];

            safe_cmd_vel_pubs[robot_name]->publish(safe_cmd_vel);
        }
    }
}

// callbacks
void NcrCbfNode::pose_callback(std::string robot_name, const geometry_msgs::msg::PoseStamped &msg)
{
    if (!all_init_poses_recieved_)
    {
        poses_recieved_.insert_or_assign(robot_name, true);
    }
    rclcpp::Time last_pose_time = this->now();
    pose_time_stamps_.insert_or_assign(robot_name, last_pose_time);
    Eigen::Vector3d agent_position(
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z);
    Eigen::Quaterniond agent_orientation(
        msg.pose.orientation.w,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z);
    positions_.insert_or_assign(robot_name, agent_position);
    pose_quaternions_.insert_or_assign(robot_name, agent_orientation);
}

void NcrCbfNode::unsafe_cmd_vel_callback(std::string robot_name, const geometry_msgs::msg::TwistStamped &msg)
{
    if (!all_init_unsafe_cmd_vels_recieved_)
    {
        unsafe_cmd_vels_recieved_.insert_or_assign(robot_name, true);
    }
    rclcpp::Time last_cmd_vel_time = this->now();
    unsafe_cmd_vel_time_stamps_.insert_or_assign(robot_name, last_cmd_vel_time);
    Eigen::Vector3d agent_unsafe_linear_cmd_vel(
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z);
    Eigen::Vector3d agent_unsafe_angular_cmd_vel(
        msg.twist.angular.x,
        msg.twist.angular.y,
        msg.twist.angular.z);
    unsafe_linear_cmd_vels_.insert_or_assign(robot_name, agent_unsafe_linear_cmd_vel);
    unsafe_angular_cmd_vels_.insert_or_assign(robot_name, agent_unsafe_angular_cmd_vel);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NcrCbfNode>());
    rclcpp::shutdown();
    return 0;
}
