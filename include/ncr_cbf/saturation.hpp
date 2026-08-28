#pragma once

#include <ncr_cbf/types.hpp>

#include <Eigen/Core>

namespace ncr_cbf{
    LinearConstraints constructSaturationConstraint(
        const Eigen::Vector3d& componentwise_vel_bound
    );
} // namespace ncr_cbf