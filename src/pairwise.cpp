#include <ncr_cbf/pairwise.hpp>

#include <stdexcept>
#include <cmath>
#include <limits>

namespace ncr_cbf{
    LinearConstraints constructPairwiseConstraint(
        const Eigen::Vector3d& agent_position_i,
        const Eigen::Vector3d& agent_position_j,
        double robot_radius_i,
        double robot_radius_j,
        double safety_buffer,
        double kappa
    )
    {
        // input validations
        // robot radius
        if (robot_radius_i < 0.0) {
            throw std::invalid_argument("robot_radius_i must be >= 0.0");
        }
        if (robot_radius_j < 0.0) {
            throw std::invalid_argument("robot_radius_j must be >= 0.0");
        }

        // safety buffer
        if (safety_buffer < 0.0) {
            throw std::invalid_argument("safety_buffer must be >= 0.0");
        }

        // CBF
        if (kappa <= 0.0) {
            throw std::invalid_argument("kappa must be > 0.0");
        }

        // is finite
        if (!agent_position_i.allFinite()) {
            throw std::invalid_argument("All values of agent_position_i must be finite");
        }
        if (!agent_position_j.allFinite()) {
            throw std::invalid_argument("All values of agent_position_j must be finite");
        }
        
        // output object init
        LinearConstraints constraints;
        constraints.matrix = Eigen::MatrixXd::Zero(1,6);
        constraints.lower_bounds = Eigen::VectorXd::Zero(1);
        constraints.upper_bounds = Eigen::VectorXd::Zero(1);
        
        // constraint matrix
        Eigen::Vector3d inter_agent_difference = agent_position_i - agent_position_j;
        double min_inter_agent_distance = robot_radius_i + robot_radius_j + safety_buffer;
        constraints.matrix.block(0,0,1,3) = -2.0 * inter_agent_difference.transpose();
        constraints.matrix.block(0,3,1,3) = 2.0 * inter_agent_difference.transpose();

        // u/l bounds
        constraints.upper_bounds(0) = kappa*(pow(inter_agent_difference.norm(),2) - pow(min_inter_agent_distance,2));
        constraints.lower_bounds(0) = -std::numeric_limits<double>::infinity();

        // return output
        return constraints;
    }
}