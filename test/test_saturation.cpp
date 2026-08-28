#include <ncr_cbf/saturation.hpp>

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <map>

int main() {
    using namespace ncr_cbf;

    // linear constraints
    LinearConstraints saturation_constraint;
    LinearConstraints compare_constraint;

    // construct saturation
    Eigen::Vector3d componentwise_vel_bound;
    componentwise_vel_bound <<
        11.0,
        12.0,
        13.0;
    
    saturation_constraint = constructSaturationConstraint(componentwise_vel_bound);

    // construct compare
    compare_constraint.matrix = Eigen::MatrixXd::Zero(6,3);
    compare_constraint.matrix <<
        -1.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0,
        0.0, 0.0, 1.0;

    compare_constraint.upper_bounds = Eigen::VectorXd::Zero(6);
    compare_constraint.upper_bounds <<
        11.0,
        11.0,
        12.0,
        12.0,
        13.0,
        13.0;

    compare_constraint.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());

    // compare
    double tol = 1e-9; 
    Eigen::MatrixXd matrix_diff = saturation_constraint.matrix - compare_constraint.matrix;
    Eigen::VectorXd upper_diff = saturation_constraint.upper_bounds - compare_constraint.upper_bounds;

    assert(matrix_diff.norm() < tol);
    assert(upper_diff.norm() < tol);

    for (int idx = 0; idx < saturation_constraint.lower_bounds.size(); ++idx)
    {
        assert(std::isinf(saturation_constraint.lower_bounds(idx)));
        assert(saturation_constraint.lower_bounds(idx) < 0.0);
    }

    // passed
    std::cout << "test_saturation passed\n";
    return 0;
}

