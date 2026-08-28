#include<ncr_cbf/solver_wrapper.hpp>

#include <proxsuite/proxqp/dense/dense.hpp>

#include <stdexcept>
#include <cmath>
#include <limits>
#include <Eigen/Core>

namespace ncr_cbf{
    std::pair<
        std::map<
            std::size_t,Eigen::Vector3d
        >,SolverStatus
    > solveQP(
        const std::map<
            std::size_t,Eigen::Vector3d
        >& agent_desired_control_inputs,
        const LinearConstraints& global_constraints
    ) {
        // construct input vector and perform checks
        std::size_t num_agents = agent_desired_control_inputs.size();

        // valid indices
        if (num_agents == 0) {
            throw std::invalid_argument("Number of agents must be >0");
        }
        for (auto const& agent_desired_control_input : agent_desired_control_inputs) {
            if (agent_desired_control_input.first >= num_agents) {
                throw std::invalid_argument("Invalid agent index " + std::to_string(agent_desired_control_input.first) + ". Must be < " + std::to_string(num_agents));
            }
        }

        // construct u_des
        Eigen::VectorXd u_des = Eigen::VectorXd::Zero(3*num_agents);
        for (auto const& agent_desired_control_input : agent_desired_control_inputs) {
            u_des.segment<3>(3*agent_desired_control_input.first) = agent_desired_control_input.second;
        }
        // finite control input
        if (!u_des.allFinite()) {
            throw std::invalid_argument("All values of agent_desired_control_inputs must be finite");
        }

        // dimension
        if (3*num_agents != global_constraints.matrix.cols()) {
            throw std::invalid_argument("Number of global constraints " + std::to_string(global_constraints.matrix.cols()) + " incompatible with " + std::to_string(3*num_agents) + " desired control inputs");
        }

        // construct QP
        // \frac12 \lVert u - u_{\text{des}} \rVert_2^2 = 
        // \frac12 u^\top u - u_{\text{des}}^\top u + \frac12 u_{\text{des}}^\top u_{\text{des}}
        // H = I, g = -u_{\text{des}}

        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3*num_agents,3*num_agents);
        Eigen::VectorXd g = -u_des;

        std::size_t qp_num_inputs = 3*num_agents;
        std::size_t qp_num_equality_constraints = 0;
        std::size_t qp_num_equations = global_constraints.matrix.rows();
        proxsuite::proxqp::dense::QP<double> qp(qp_num_inputs,qp_num_equality_constraints,qp_num_equations);

        qp.init(
            H,
            g,
            proxsuite::nullopt,
            proxsuite::nullopt,
            global_constraints.matrix,
            global_constraints.lower_bounds,
            global_constraints.upper_bounds
        );

        // solve QP, repackage outputs
        qp.solve();

        // control input
        std::map<
            std::size_t,Eigen::Vector3d
        > u_qp_output = {}; 
        std::size_t current_idx = 0;
        for (std::size_t agent_idx =0; agent_idx < num_agents; agent_idx++) {
            u_qp_output.insert({agent_idx,qp.results.x.segment<3>(current_idx)});
            current_idx += 3;
        }

        // solver status
        SolverStatus solver_status;
        if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
            solver_status = SolverStatus::SOLVED;
        }
        else if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED) {
            solver_status = SolverStatus::MAX_ITER_REACHED;
        }
        else if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE) {
            solver_status = SolverStatus::PRIMAL_INFEASIBLE;
        }
        else if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE) {
            solver_status = SolverStatus::SOLVED_CLOSEST_PRIMAL_FEASIBLE;
        }
        else if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE) {
            solver_status = SolverStatus::DUAL_INFEASIBLE;
        }
        else if (qp.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN) {
            solver_status = SolverStatus::NOT_RUN;
        }
        else {
            throw std::invalid_argument("Unknown QP solver status");
        }

        // package output
        return {u_qp_output, solver_status};;
    }
}