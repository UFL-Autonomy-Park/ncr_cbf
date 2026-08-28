#pragma once

#include <ncr_cbf/types.hpp>

#include <Eigen/Core>

namespace ncr_cbf{
    LinearConstraints constructPairwiseConstraint(
        const Eigen::Vector3d& agent_position_i,
        const Eigen::Vector3d& agent_position_j,
        double robot_radius_i,
        double robot_radius_j,
        double safety_buffer,
        double kappa
    );
} // namespace ncr_cbf