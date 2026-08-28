#include <ncr_cbf/pairwise.hpp>

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    using namespace ncr_cbf;

    // agent positions, radii, buffer, kappa
    Eigen::Vector3d agent_position_i;
    agent_position_i <<
        4.0,
        0.0,
        1.0;
    Eigen::Vector3d agent_position_j;
    agent_position_j <<
        -4.0,
        0.5,
        3.0;
    double robot_radius_i = 0.5;
    double robot_radius_j = 0.33;
    double safety_buffer = 0.1;
    double kappa = 2.0;

    // construct RoomConstraints
    LinearConstraints pairwise_constraint = constructPairwiseConstraint(agent_position_i,agent_position_j,robot_radius_i,robot_radius_j,safety_buffer,kappa);

    // compare
    double tol = std::pow(10,-9);

    // implement checks
    // matrix has right dims
    assert (pairwise_constraint.matrix.rows() == 1);
    assert (pairwise_constraint.matrix.cols() == 6);
    // matrix entries are correct
    Eigen::MatrixXd matrix_ground_truth(1,6);
    matrix_ground_truth <<
        -16.0, 1.0, 4.0, 16.0, -1.0, -4.0;
    Eigen::MatrixXd matrix_diff = matrix_ground_truth - pairwise_constraint.matrix;
    assert(matrix_diff.norm() < tol);

    // upper bounds tol
    Eigen::VectorXd upper_ground_truth = Eigen::VectorXd(1);
    upper_ground_truth <<
        134.7702;
    Eigen::VectorXd upper_compare = upper_ground_truth - pairwise_constraint.upper_bounds;
    double upper_diff = upper_compare.norm();
    assert(upper_diff < tol);

    // lower bounds tol
    assert(std::isinf(pairwise_constraint.lower_bounds(0)));
    assert(pairwise_constraint.lower_bounds(0) < 0.0);

    // passed
    std::cout << "test_pairwise passed\n";
    return 0;
}

