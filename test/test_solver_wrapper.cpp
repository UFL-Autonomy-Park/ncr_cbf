#include <ncr_cbf/room.hpp>
#include <ncr_cbf/saturation.hpp>
#include <ncr_cbf/solver_wrapper.hpp>

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <map>

#include <Eigen/Core>

int main() {
    using namespace ncr_cbf;

    // tolerance
    double tol = std::pow(10,-4);

    // scenario 1: simple
    RoomGeometry simple_room_geometry;
    simple_room_geometry.x_min = -5.0;
    simple_room_geometry.x_max = 5.0;
    simple_room_geometry.y_min = -5.0;
    simple_room_geometry.y_max = 5.0;
    simple_room_geometry.z_min = -5.0;
    simple_room_geometry.z_max = 5.0;

    double simple_robot_radius = 0.1;
    double simple_room_kappa = 0.1;

    std::map<std::size_t, Eigen::Vector3d> simple_positions = {};
    simple_positions.insert({0,Eigen::Vector3d::Constant(1)});
    
    std::map<std::size_t, Eigen::Vector3d> simple_desired_control_inputs = {};
    simple_desired_control_inputs.insert({0,Eigen::Vector3d::Constant(0.1)});

    LinearConstraints simple_constraints;
    simple_constraints = constructRoomConstraints(simple_room_geometry,simple_positions.at(0),simple_robot_radius,simple_room_kappa);

    auto simple_qp_result = solveQP(simple_desired_control_inputs,simple_constraints);

    Eigen::VectorXd simple_control_input_diff = simple_desired_control_inputs.at(0) - simple_qp_result.first.at(0);

    assert(simple_control_input_diff.norm() < tol);
    assert(simple_qp_result.second == SolverStatus::SOLVED);

    // scenario 2: saturation constraint on first variable
    std::map<std::size_t, Eigen::Vector3d> saturation_desired_control_inputs = {};
    saturation_desired_control_inputs.insert({0,Eigen::Vector3d::Constant(1.0)});

    Eigen::Vector3d componentwise_vel_bound;
    componentwise_vel_bound <<
        0.1,
        20,
        20;

    LinearConstraints saturation_constraints;
    saturation_constraints = constructSaturationConstraint(
        componentwise_vel_bound
    );

    auto saturation_qp_result = solveQP(saturation_desired_control_inputs,saturation_constraints);

    Eigen::Vector3d saturation_expected_control_input;
    saturation_expected_control_input <<
        0.1,
        1.0,
        1.0;

    Eigen::VectorXd saturation_control_input_diff = saturation_expected_control_input - saturation_qp_result.first.at(0);

    assert(saturation_control_input_diff.norm() < tol);
    assert(saturation_qp_result.second == SolverStatus::SOLVED);

    std::cout << "test_solver_wrapper passed\n";
    return 0;
}