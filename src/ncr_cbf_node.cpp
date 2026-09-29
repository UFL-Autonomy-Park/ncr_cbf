#include <ncr_cbf_node.hpp>
#include <stdexcept>

NcrCbfNode::NcrCbfNode() : Node("ncr_cbf_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing NCR CBF node");

    // declare global parameters
    this->declare_parameter("robot_names");
    this->declare_parameter("room_geometry.x_min");
    this->declare_parameter("room_geometry.x_max");
    this->declare_parameter("room_geometry.y_min");
    this->declare_parameter("room_geometry.y_max");
    this->declare_parameter("room_geometry.z_min");
    this->declare_parameter("room_geometry.z_max");

    // get global parameters
    this->get_parameter("robot_names",robot_names);
    this->get_parameter("room_geometry.x_min",room_geometry.x_min);
    this->get_parameter("room_geometry.x_max",room_geometry.x_max);
    this->get_parameter("room_geometry.y_min",room_geometry.y_min);
    this->get_parameter("room_geometry.y_max",room_geometry.y_max);
    this->get_parameter("room_geometry.z_min",room_geometry.z_min);
    this->get_parameter("room_geometry.z_max",room_geometry.z_max);

    // set num_agents
    if (robot_names.size() < 1) {
        throw std::invalid_argument("Must specify at least 1 robot");
    }
    num_agents = robot_names.size();
    // robot names to indices, populate lookup
    for (std::size_t robot_idx = 0; robot_idx < robot_names.size(); ++robot_idx) {
        robot_index_lookup.emplace(robot_names[robot_idx],robot_idx);
        load_parameters(robot_names[robot_idx],robot_idx);
    }
}

void NcrCbfNode::load_parameters(std::string robot_name, std::size_t robot_idx) {
    // using name and index, append values to each parameter map
    const std::string prefix = "robots." + robot_name + ".";

    // room constraints 
    bool room_bool{false};
    const std::string room_string = prefix + "room_constraints";
    this->declare_parameter(room_string + ".enabled");
    this->get_parameter(room_string + ".enabled", room_bool);
    if (room_bool) {
        // save to vector
        room_agents.emplace_back(robot_idx);
        // save to map
        double room_kappa;
        this->declare_parameter(room_string + ".kappa");
        if (!this->get_parameter(room_string + ".kappa", room_kappa))
        {
            throw std::invalid_argument("Parameter " + room_string + ".kappa" + " undefined or invalid")
        }
        room_kappas.emplace(robot_idx, room_kappa);
    }
    // saturation constraints
    bool sat_bool{false};
    const std::string sat_string = prefix + "saturation_constraints";
    this->declare_parameter(sat_string + ".enabled");
    this->get_parameter(sat_string + ".enabled", sat_bool);
    if (sat_bool) {
        // save to vector
        saturation_agents.emplace_back(robot_idx);
        // save to map
        std::vector<double> componentwise_vel_bound;
        this->declare_parameter(sat_string + ".componentwise_vel_bound");
        if (!this->get_parameter(sat_string + ".componentwise_vel_bound",componentwise_vel_bound)) {
            throw std::invalid_argument("Parameter " + sat_string + ".componentwise_vel_bound" + " undefined or invalid")
        }
        if (componentwise_vel_bound.size() !=  3) {
            throw std::invalid_argument("componentwise_vel_bound for " + robot_name + " must have an x, y, and z component");
        }
        componentwise_vel_bounds.emplace(robot_idx,Eigen::Vector3d(componentwise_vel_bound[0],componentwise_vel_bound[1],componentwise_vel_bound[2]));
    }

}