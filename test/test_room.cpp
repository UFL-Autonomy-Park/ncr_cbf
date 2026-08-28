#include <ncr_cbf/room.hpp>

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    using namespace ncr_cbf;
    // room geom
    RoomGeometry room_geometry;
    room_geometry.x_min = -5.0;
    room_geometry.x_max = 5.0;
    room_geometry.y_min = -5.0;
    room_geometry.y_max = 5.0;
    room_geometry.z_min = -5.0;
    room_geometry.z_max = 5.0;

    // agent position, radius, kappa
    Eigen::Vector3d agent_position;
    agent_position <<
        4.0,
        0.0,
        1.0;
    double robot_radius = 2.0;
    double kappa = 2.0;

    // construct RoomConstraints
    LinearConstraints room_constraints = constructRoomConstraints(room_geometry,agent_position,robot_radius,kappa);

    // compare
    double tol = std::pow(10,-9);

    // implement checks
    // matrix has right dims
    assert (room_constraints.matrix.rows() == 6);
    assert (room_constraints.matrix.cols() == 3);
    // matrix entries are correct
    Eigen::MatrixXd matrix_ground_truth(6,3);
    matrix_ground_truth <<
        -1.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0,
        0.0, 0.0, 1.0;
    Eigen::MatrixXd matrix_diff = matrix_ground_truth - room_constraints.matrix;
    assert(matrix_diff.norm() < tol);

    // upper bounds tol
    Eigen::VectorXd upper_ground_truth = Eigen::VectorXd(6);
    upper_ground_truth <<
        14.0,
        -2.0,
        6.0,
        6.0,
        8.0,
        4.0;
    Eigen::VectorXd upper_compare = upper_ground_truth - room_constraints.upper_bounds;
    double upper_diff = upper_compare.norm();
    assert(upper_diff < tol);

    // lower bounds tol
    for (int idx = 0; idx < 6; idx++) {
        assert(std::isinf(room_constraints.lower_bounds(idx)));
        assert(room_constraints.lower_bounds(idx) < 0.0);
    }

    // passed
    std::cout << "test_room passed\n";
    return 0;
}

