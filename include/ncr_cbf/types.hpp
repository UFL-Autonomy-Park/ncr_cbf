#pragma once

#include <Eigen/Core>

namespace ncr_cbf
{
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