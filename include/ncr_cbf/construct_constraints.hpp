#pragma once

#include <ncr_cbf/types.hpp>

#include <map>

namespace ncr_cbf{
    LinearConstraints assembleConstraints(
        std::size_t num_agents,
        const std::map<
            std::size_t,
            LinearConstraints
        >& room_constraints,
        const std::map<
            std::size_t,
            LinearConstraints
        >& saturation_constraints,
        const std::map<
            std::pair<std::size_t, std::size_t>,
            LinearConstraints
        >& pairwise_constraints
    );

    LinearConstraints constructGlobalConstraints(
        const std::size_t& num_agents,
        std::vector<std::size_t>& room_agents,
        std::vector<std::size_t>& saturation_agents,
        std::vector<std::pair<std::size_t,std::size_t>>& pairwise_pairs,
        const RoomGeometry& room_geometry,
        const std::map<
            std::size_t, double
        >& robot_radii,
        const std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        >& safety_buffers,
        const std::map<
            std::size_t, double
        >& room_kappas,
        const std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        >& pairwise_kappas,
        const std::map<
            std::size_t,Eigen::Vector3d
        >& componentwise_vel_bounds,
        const std::map<
            std::size_t,Eigen::Vector3d
        >& agent_positions
    );
} // namespace ncr_cbf