#include <ncr_cbf/construct_constraints.hpp>

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <map>

int main() {
    using namespace ncr_cbf;

    std::size_t num_agents = 3;

    // constraint values
    LinearConstraints compare_constraint;
    LinearConstraints room_constraint_0;
    LinearConstraints room_constraint_1;
    LinearConstraints saturation_constraint_1;
    LinearConstraints saturation_constraint_2;
    LinearConstraints pairwise_constraint_01;
    LinearConstraints pairwise_constraint_02;
    LinearConstraints pairwise_constraint_12;

    // room 
    Eigen::MatrixXd room_matrix_0 = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd room_matrix_1 = Eigen::MatrixXd::Zero(6,3);

    room_matrix_0 <<
        -2.0, 0.0, 0.0,
        2.0, 0.0, 0.0,
        0.0, -3.0, 0.0,
        0.0, 3.0, 0.0,
        0.0, 0.0, -4.0,
        0.0, 0.0, 4.0;
    room_matrix_1 <<
        -5.0, 0.0, 0.0,
        5.0, 0.0, 0.0,
        0.0, -6.0, 0.0,
        0.0, 6.0, 0.0,
        0.0, 0.0, -7.0,
        0.0, 0.0, 7.0;

    Eigen::MatrixXd joint_room_matrix = Eigen::MatrixXd::Zero(12,9);
    joint_room_matrix.block(0,0,6,3) = room_matrix_0;
    joint_room_matrix.block(6,3,6,3) = room_matrix_1;

    room_constraint_0.matrix = room_matrix_0;
    room_constraint_0.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());
    room_constraint_0.upper_bounds = Eigen::VectorXd::Zero(6);
    room_constraint_0.upper_bounds <<
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0;

    room_constraint_1.matrix = room_matrix_1;
    room_constraint_1.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());
    room_constraint_1.upper_bounds = Eigen::VectorXd::Zero(6);
    room_constraint_1.upper_bounds <<
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0;

    // saturation
    Eigen::MatrixXd saturation_matrix_1 = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd saturation_matrix_2 = Eigen::MatrixXd::Zero(6,3);

    saturation_matrix_1 <<
        -2.0, 0.0, 0.0,
        2.0, 0.0, 0.0,
        0.0, -3.0, 0.0,
        0.0, 3.0, 0.0,
        0.0, 0.0, -4.0,
        0.0, 0.0, 4.0;
    saturation_matrix_2 <<
        -5.0, 0.0, 0.0,
        5.0, 0.0, 0.0,
        0.0, -6.0, 0.0,
        0.0, 6.0, 0.0,
        0.0, 0.0, -7.0,
        0.0, 0.0, 7.0;

    Eigen::MatrixXd joint_saturation_matrix = Eigen::MatrixXd::Zero(12,9);
    joint_saturation_matrix.block(0,3,6,3) = saturation_matrix_1;
    joint_saturation_matrix.block(6,6,6,3) = saturation_matrix_2;

    saturation_constraint_1.matrix = saturation_matrix_1;
    saturation_constraint_1.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());
    saturation_constraint_1.upper_bounds = Eigen::VectorXd::Zero(6);
    saturation_constraint_1.upper_bounds <<
        1.0,
        1.0,
        3.0,
        3.0,
        5.0,
        5.0;

    saturation_constraint_2.matrix = saturation_matrix_2;
    saturation_constraint_2.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());
    saturation_constraint_2.upper_bounds = Eigen::VectorXd::Zero(6);
    saturation_constraint_2.upper_bounds <<
        7.0,
        7.0,
        9.0,
        9.0,
        11.0,
        11.0;

    // pairwise
    Eigen::MatrixXd pairwise_matrix_01 = Eigen::MatrixXd::Zero(1,6);
    Eigen::MatrixXd pairwise_matrix_02 = Eigen::MatrixXd::Zero(1,6);
    Eigen::MatrixXd pairwise_matrix_12 = Eigen::MatrixXd::Zero(1,6);

    pairwise_matrix_01 <<
        -1.0, -2.0, -3.0, 1.0, 2.0, 3.0;
    pairwise_matrix_02 <<
        -4.0, -5.0, -6.0, 4.0, 5.0, 6.0;
    pairwise_matrix_12 <<
        -7.0, -8.0, -9.0, 7.0, 8.0, 9.0;

    Eigen::MatrixXd joint_pairwise_matrix = Eigen::MatrixXd::Zero(3,3*num_agents);

    joint_pairwise_matrix.block(0,0,1,3) = pairwise_matrix_01.block(0,0,1,3);
    joint_pairwise_matrix.block(0,3,1,3) = pairwise_matrix_01.block(0,3,1,3);

    joint_pairwise_matrix.block(1,0,1,3) = pairwise_matrix_02.block(0,0,1,3);
    joint_pairwise_matrix.block(1,6,1,3) = pairwise_matrix_02.block(0,3,1,3);

    joint_pairwise_matrix.block(2,3,1,3) = pairwise_matrix_12.block(0,0,1,3);
    joint_pairwise_matrix.block(2,6,1,3) = pairwise_matrix_12.block(0,3,1,3);

    pairwise_constraint_01.matrix = pairwise_matrix_01;
    pairwise_constraint_01.lower_bounds = Eigen::VectorXd::Constant(1,-std::numeric_limits<double>::infinity());
    pairwise_constraint_01.upper_bounds = Eigen::VectorXd::Zero(1);
    pairwise_constraint_01.upper_bounds <<
        19.0;

    pairwise_constraint_02.matrix = pairwise_matrix_02;
    pairwise_constraint_02.lower_bounds = Eigen::VectorXd::Constant(1,-std::numeric_limits<double>::infinity());
    pairwise_constraint_02.upper_bounds = Eigen::VectorXd::Zero(1);
    pairwise_constraint_02.upper_bounds <<
        20.0;

    pairwise_constraint_12.matrix = pairwise_matrix_12;
    pairwise_constraint_12.lower_bounds = Eigen::VectorXd::Constant(1,-std::numeric_limits<double>::infinity());
    pairwise_constraint_12.upper_bounds = Eigen::VectorXd::Zero(1);
    pairwise_constraint_12.upper_bounds <<
        21.0;
    
    // compare
    compare_constraint.matrix = Eigen::MatrixXd::Zero(27,9);
    compare_constraint.matrix.block(0,0,12,9) = joint_room_matrix;
    compare_constraint.matrix.block(12,0,12,9) = joint_saturation_matrix;
    compare_constraint.matrix.block(24,0,3,9) = joint_pairwise_matrix;
    
    compare_constraint.upper_bounds = Eigen::VectorXd::Zero(27);
    compare_constraint.upper_bounds <<
        // room 0
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        // room 1
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        // saturation 1
        1.0,
        1.0,
        3.0,
        3.0,
        5.0,
        5.0,
        // saturation 2
        7.0,
        7.0,
        9.0,
        9.0,
        11.0,
        11.0,
        // pairwise 01
        19.0,
        // pairwise 02
        20.0,
        // pairwise 12
        21.0;
    compare_constraint.lower_bounds = Eigen::VectorXd::Constant(27,-std::numeric_limits<double>::infinity());

    // make maps
    // room
    std::map<std::size_t,LinearConstraints> room_constraints = {};
    room_constraints.insert({0,room_constraint_0});
    room_constraints.insert({1,room_constraint_1});

    // saturation
    std::map<std::size_t,LinearConstraints> saturation_constraints = {};
    saturation_constraints.insert({1,saturation_constraint_1});
    saturation_constraints.insert({2,saturation_constraint_2});
    
    // pairwise
    std::map<std::pair<std::size_t,std::size_t>,LinearConstraints> pairwise_constraints = {};
    pairwise_constraints.insert({std::make_pair(0,1),pairwise_constraint_01});
    pairwise_constraints.insert({std::make_pair(0,2),pairwise_constraint_02});
    pairwise_constraints.insert({std::make_pair(1,2),pairwise_constraint_12});

    // assembled
    LinearConstraints assembled_constraints = assembleConstraints(num_agents,room_constraints,saturation_constraints,pairwise_constraints);

    // compare
    double tol = 1e-9; 
    Eigen::MatrixXd matrix_diff = assembled_constraints.matrix - compare_constraint.matrix;
    Eigen::VectorXd upper_diff = assembled_constraints.upper_bounds - compare_constraint.upper_bounds;

    assert(matrix_diff.norm() < tol);
    assert(upper_diff.norm() < tol);

    for (int idx = 0; idx < assembled_constraints.lower_bounds.size(); ++idx)
    {
        assert(std::isinf(assembled_constraints.lower_bounds(idx)));
        assert(assembled_constraints.lower_bounds(idx) < 0.0);
    }

    // passed
    std::cout << "test_construct_constraints passed\n";
    return 0;
}

