#include <ncr_cbf/types.hpp>

#include <Eigen/Core>

class NcrCbfNode : public rclcpp::Node
{
    public:
        NcrCbfNode();
    private:
        // populate with parameters
        std::vector<std::string> robot_names;
        std::map<
            std::string,std::size_t
        > robot_index_lookup;
        ncr_cbf::RoomGeometry room_geometry;
        std::size_t num_agents;
        std::vector<std::size_t> room_agents;
        std::vector<std::size_t> saturation_agents;
        std::vector<std::pair<std::size_t,std::size_t>> pairwise_pairs;
        std::map<
            std::size_t, double
        > robot_radii;
        std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        > safety_buffers;
        std::map<
            std::size_t, double
        > room_kappas;
        std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        > pairwise_kappas;
        std::map<
            std::size_t,Eigen::Vector3d
        > componentwise_vel_bounds;

        void load_parameters(std::string robot_name, std::size_t robot_idx);
};