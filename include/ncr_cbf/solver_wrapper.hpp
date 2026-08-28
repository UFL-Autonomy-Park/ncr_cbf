#pragma once

#include <ncr_cbf/types.hpp>

#include <Eigen/Core>
#include <map>

namespace ncr_cbf{
    std::pair<
        std::map<
        std::size_t,Eigen::Vector3d
        >,SolverStatus
    > solveQP(
        const std::map<
            std::size_t,Eigen::Vector3d
        >& agent_desired_control_inputs,
        const LinearConstraints& global_constraints
    );
} // namespace ncr_cbf
