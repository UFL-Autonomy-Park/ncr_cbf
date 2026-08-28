#include <ncr_cbf/construct_constraints.hpp>
#include <ncr_cbf/room.hpp>
#include <ncr_cbf/pairwise.hpp>
#include <ncr_cbf/saturation.hpp>

#include <stdexcept>
#include <vector>
#include <algorithm>
#include <map>

namespace {
    template <typename T>
    void sortAndRemoveDuplicates(
        std::vector<T>& contains_duplicates
    )
    {
        std::sort(contains_duplicates.begin(),contains_duplicates.end());
        auto last = std::unique(contains_duplicates.begin(),contains_duplicates.end());
        contains_duplicates.erase(last,contains_duplicates.end());
    }
}

namespace ncr_cbf{
    LinearConstraints assembleConstraints(
        std::size_t num_agents,
        const std::map<
            std::size_t,
            LinearConstraints
        >& room_constraints,
        const std::map<
            std::size_t,
            LinearConstraints
        >& saturation_constraints,
        const std::map<
            std::pair<std::size_t, std::size_t>,
            LinearConstraints
        >& pairwise_constraints
    )
    {
        // input validations
        // num_agents
        if (num_agents < 1) {
            throw std::invalid_argument("There must be at least one active agent.");
        }

        // room
        // agent_idx < num_agents
        // matrix 6x3
        // lower/upper size 6
        for (auto it = room_constraints.begin(); it != room_constraints.end(); ++it) {
            if (it->first >= num_agents) {
                throw std::invalid_argument("Agent index " + std::to_string(it->first) + " must be less than " + std::to_string(num_agents) + ".");
            }

            if (it->second.matrix.rows() != 6) {
                throw std::invalid_argument("Room constraint with index " + std::to_string(it->first) + " has invalid row dimension.");
            }

            if (it->second.matrix.cols() != 3) {
                throw std::invalid_argument("Room constraint with index " + std::to_string(it->first) + " has invalid column dimension.");
            }

            if (it->second.lower_bounds.size() != 6) {
                throw std::invalid_argument("Room constraint with index " + std::to_string(it->first) + " has invalid lower_bounds dimension.");
            }

            if (it->second.upper_bounds.size() != 6) {
                throw std::invalid_argument("Room constraint with index " + std::to_string(it->first) + " has invalid upper_bounds dimension.");
            }
        }

        // saturation
        // agent_idx < num_agents
        // matrix 6x3
        // lower/upper size 6
        for (auto it = saturation_constraints.begin(); it != saturation_constraints.end(); ++it) {
            if (it->first >= num_agents) {
                throw std::invalid_argument("Agent index " + std::to_string(it->first) + " must be less than " + std::to_string(num_agents) + ".");
            }

            if (it->second.matrix.rows() != 6) {
                throw std::invalid_argument("Saturation constraint with index " + std::to_string(it->first) + " has invalid row dimension.");
            }

            if (it->second.matrix.cols() != 3) {
                throw std::invalid_argument("Saturation constraint with index " + std::to_string(it->first) + " has invalid column dimension.");
            }

            if (it->second.lower_bounds.size() != 6) {
                throw std::invalid_argument("Saturation constraint with index " + std::to_string(it->first) + " has invalid lower_bounds dimension.");
            }

            if (it->second.upper_bounds.size() != 6) {
                throw std::invalid_argument("Saturation constraint with index " + std::to_string(it->first) + " has invalid upper_bounds dimension.");
            }
        }

        // pairwise 
        // i < j
        // i,j < num_agents
        // matrix 1x6
        // lower/upper size 1
        for (auto it = pairwise_constraints.begin(); it != pairwise_constraints.end(); ++it) {
            std::string i_j_str = "(" + std::to_string(it->first.first) + "," + std::to_string(it->first.second) + ")";

            if (it->first.first >= it->first.second) {
                throw std::invalid_argument("Invalid pairwise constraint index (i,j): " + i_j_str + ". Require i < j");
            }

            if (it->first.first >= num_agents || it->first.second >= num_agents) {
                throw std::invalid_argument("Invalid pairwise constraint index (i,j): " + i_j_str + ". Require i,j < num_agents");
            }

            if (it->second.matrix.rows() != 1) {
                throw std::invalid_argument("Pairwise constraint with index " + i_j_str + " has invalid row dimension.");
            }

            if (it->second.matrix.cols() != 6) {
                throw std::invalid_argument("Pairwise constraint with index " + i_j_str + " has invalid column dimension.");
            }

            if (it->second.upper_bounds.size() != 1) {
                throw std::invalid_argument("Pairwise constraint with index " + i_j_str + " has invalid upper_bounds dimension.");
            }

            if (it->second.lower_bounds.size() != 1) {
                throw std::invalid_argument("Pairwise constraint with index " + i_j_str + " has invalid lower_bounds dimension.");
            }
        }
        
        // get dimensions
        std::size_t num_variables = 3 * num_agents;
        std::size_t num_room_rows = 6 * room_constraints.size();
        std::size_t num_saturation_rows = 6 * saturation_constraints.size();
        std::size_t num_pairwise_rows = pairwise_constraints.size();
        std::size_t total_rows = num_room_rows + num_saturation_rows + num_pairwise_rows;

        // output object init
        LinearConstraints constraints;
        constraints.matrix = Eigen::MatrixXd::Zero(total_rows, num_variables);
        constraints.lower_bounds = Eigen::VectorXd::Zero(total_rows);
        constraints.upper_bounds = Eigen::VectorXd::Zero(total_rows);  

        // room constraint loop
        std::size_t current_row = 0;
        for (auto it = room_constraints.begin(); it != room_constraints.end(); ++it) {
            constraints.matrix.block(current_row,3*it->first,6,3) = it->second.matrix;

            constraints.lower_bounds.segment(current_row,6) = it->second.lower_bounds;

            constraints.upper_bounds.segment(current_row,6) = it->second.upper_bounds;

            current_row += 6;
        }

        // saturation constraint loop
        for (auto it = saturation_constraints.begin(); it != saturation_constraints.end(); ++it) {
            constraints.matrix.block(current_row,3*it->first,6,3) = it->second.matrix;

            constraints.lower_bounds.segment(current_row,6) = it->second.lower_bounds;

            constraints.upper_bounds.segment(current_row,6) = it->second.upper_bounds;

            current_row += 6;
        }

        // pairwise loop
        for (auto it = pairwise_constraints.begin(); it != pairwise_constraints.end(); ++it) {
            constraints.matrix.block(current_row,3*it->first.first,1,3) = it->second.matrix.block(0,0,1,3);

            constraints.matrix.block(current_row,3*it->first.second,1,3) = it->second.matrix.block(0,3,1,3);

            constraints.lower_bounds(current_row) = it->second.lower_bounds(0);

            constraints.upper_bounds(current_row) = it->second.upper_bounds(0);

            ++current_row;
        }

        // return output
        return constraints;
    }

    LinearConstraints constructGlobalConstraints(
        const std::size_t& num_agents,
        std::vector<std::size_t> room_agents,
        std::vector<std::size_t> saturation_agents,
        std::vector<std::pair<std::size_t,std::size_t>>& pairwise_pairs,
        const RoomGeometry& room_geometry,
        const std::map<
            std::size_t, double
        >& robot_radii,
        const std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        >& safety_buffers,
        const std::map<
            std::size_t, double
        >& room_kappas,
        const std::map<
            std::pair<
                std::size_t, std::size_t
            >, double
        >& pairwise_kappas,
        const std::map<
            std::size_t,Eigen::Vector3d
        >& componentwise_vel_bounds,
        const std::map<
            std::size_t,Eigen::Vector3d
        >& agent_positions
    ) 
    {
        // constraint prep
        // sort vectors appropriately and remove duplicates
        // room
        sortAndRemoveDuplicates(room_agents);

        // saturation
        sortAndRemoveDuplicates(saturation_agents);

        // pairwise
        for (auto& pair : pairwise_pairs) {
            if (pair.first > pair.second){
                std::swap(pair.first, pair.second);
            }
        }
        sortAndRemoveDuplicates(pairwise_pairs);

        // constraint map construction
        // room constraints
        std::map<
            std::size_t,
            LinearConstraints
        > room_constraints = {};

        for (auto room_agent : room_agents) {
            LinearConstraints room_constraint;
            room_constraint = constructRoomConstraints(
                room_geometry,
                agent_positions.at(room_agent),
                robot_radii.at(room_agent),
                room_kappas.at(room_agent)
            );
            room_constraints.insert({room_agent,room_constraint});
        }

        // saturation constraints
        std::map<std::size_t,LinearConstraints> saturation_constraints = {};
        for (auto saturation_agent : saturation_agents) { 
            LinearConstraints saturation_constraint;
            saturation_constraint = constructSaturationConstraint(
                componentwise_vel_bounds.at(saturation_agent)
            );
            saturation_constraints.insert({saturation_agent,saturation_constraint});
        }

        // pairwise constraints
        std::map<std::pair<std::size_t,std::size_t>,LinearConstraints> pairwise_constraints = {};
        for (auto pairwise_pair : pairwise_pairs) {
            LinearConstraints pairwise_constraint;
            pairwise_constraint = constructPairwiseConstraint(
                agent_positions.at(pairwise_pair.first),
                agent_positions.at(pairwise_pair.second),
                robot_radii.at(pairwise_pair.first),
                robot_radii.at(pairwise_pair.second),
                safety_buffers.at(pairwise_pair),
                pairwise_kappas.at(pairwise_pair)
            );
            pairwise_constraints.insert({pairwise_pair,pairwise_constraint});
        }

        // global constraints
        LinearConstraints global_constraints;
        global_constraints = assembleConstraints(
            num_agents,
            room_constraints,
            saturation_constraints,
            pairwise_constraints
        );
        return global_constraints;
    }
}