#pragma once

#include <Eigen/Core>
#include <array>

namespace ncr_cbf
{
    // Face order matches the rows of constructRoomConstraints.
    using RoomFaces = std::array<bool, 6>;
    inline constexpr std::array<const char *, 6> room_face_names{
        "x_min", "x_max", "y_min", "y_max", "z_min", "z_max"};
    inline constexpr RoomFaces all_room_faces{true, true, true, true, true, true};

    struct RoomGeometry
    {
        double x_min{0.0};
        double x_max{0.0};
        double y_min{0.0};
        double y_max{0.0};
        double z_min{0.0};
        double z_max{0.0};
    };
    struct LinearConstraints
    {
        Eigen::MatrixXd matrix;
        Eigen::VectorXd lower_bounds;
        Eigen::VectorXd upper_bounds;
    };
    enum class SolverStatus
    {
        SOLVED,
        MAX_ITER_REACHED,
        PRIMAL_INFEASIBLE,
        SOLVED_CLOSEST_PRIMAL_FEASIBLE ,
        DUAL_INFEASIBLE,
        NOT_RUN ,
    };
} // namespace ncr_cbf
