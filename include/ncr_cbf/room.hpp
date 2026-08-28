#pragma once

#include <ncr_cbf/types.hpp>

#include <Eigen/Core>

namespace ncr_cbf{
    LinearConstraints constructRoomConstraints(
        const RoomGeometry& room_geometry,
        const Eigen::Vector3d& agent_position,
        double robot_radius,
        double kappa
    );
} // namespace ncr_cbf