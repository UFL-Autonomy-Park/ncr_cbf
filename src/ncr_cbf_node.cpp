#include <ncr_cbf/ncr_cbf_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"

NcrCbfNode::NcrCbfNode() : Node("ncr_cbf_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing NCR CBF node");

    // declare global parameters
    this->declare_parameter("location", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("robot_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
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
    this->get_parameter("robot_pair_names", robot_pair_names);
    this->get_parameter("room_geometry.x_min", room_geometry.x_min);
    this->get_parameter("room_geometry.x_max", room_geometry.x_max);
    this->get_parameter("room_geometry.y_min", room_geometry.y_min);
    this->get_parameter("room_geometry.y_max", room_geometry.y_max);
    this->get_parameter("room_geometry.z_min", room_geometry.z_min);
    this->get_parameter("room_geometry.z_max", room_geometry.z_max);

    if (location != "indoor_lab" || location != "autonomy_park")
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

    // TO-DO: construct global constraints

    // TO-DO: subscribers
    for (const auto &robot_name : robot_names)
    {
        // pose
        std::string sub_topic;
        this->declare_parameter("robots." + robot_name + ".pose_topic", rclcpp::ParameterType::PARAMETER_STRING);
        if (!this->get_parameter("robots." + robot_name + ".pose_topic", sub_topic))
        {
            throw std::invalid_argument("Parameter robots." + robot_name + ".pose_topic" + " undefined or invalid\n");
        }
        // pose_subs[robot_name] = create_subscription<geometry_msgs::msg::PoseStamped>(sub_topic, 10, ); // YOU ARE HERE

        // unsafe cmd_vel
    }

    // TO-DO: publishers

    // TO-DO: variable storage
    // vars for timeouts. timeouts should include stale pose data, stale control input data, and stale solved cbf data
    // need to think about architecture here
    // pose data comes
    // control data should go out on timer and return zeros in any situation where it would be unsafe or the data has not completely populated (i.e., no active desired control input for agents or mocap times out)
    // should also consider how the bebop modes are handled
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
