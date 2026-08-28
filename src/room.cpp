#include <ncr_cbf/room.hpp>

#include <stdexcept>
#include <cmath>
#include <limits>

namespace ncr_cbf{
    LinearConstraints constructRoomConstraints(
        const RoomGeometry& room_geometry,
        const Eigen::Vector3d& agent_position,
        double robot_radius,
        double kappa
    )
    {
        // input validations
        // ensure min values are <=  max values
        if (room_geometry.x_min >= room_geometry.x_max) {
            throw std::invalid_argument("x_min must be < x_max");
        }

        if (room_geometry.y_min >= room_geometry.y_max) {
            throw std::invalid_argument("y_min must be < y_max");
        }

        if (room_geometry.z_min >= room_geometry.z_max) {
            throw std::invalid_argument("z_min must be < z_max");
        }

        // robot radius
        if (robot_radius < 0.0) {
            throw std::invalid_argument("robot_radius must be >= 0.0");
        }

        // CBF
        if (kappa <= 0.0) {
            throw std::invalid_argument("kappa must be > 0.0");
        }

        // is finite
        if (!agent_position.allFinite()) {
            throw std::invalid_argument("All values of agent_position must be finite");
        }
        
        // room size
        if (2.0*robot_radius >= room_geometry.x_max - room_geometry.x_min) {
            throw std::invalid_argument("Safe set has zero width or is empty");
        }

        if (2.0*robot_radius >= room_geometry.y_max - room_geometry.y_min) {
            throw std::invalid_argument("Safe set has zero width or is empty");
        }

        if (2.0*robot_radius >= room_geometry.z_max - room_geometry.z_min) {
            throw std::invalid_argument("Safe set has zero width or is empty");
        }

        // output object init
        // standard order x_min,x_max,y_min,y_max,z_min,z_max
        LinearConstraints constraints;
        constraints.matrix = Eigen::MatrixXd::Zero(6,3);
        constraints.lower_bounds = Eigen::VectorXd::Zero(6);
        constraints.upper_bounds = Eigen::VectorXd::Zero(6);
        
        // constraint matrix
        for (int idx = 0; idx<6; idx++) {
            // if int even, min. if odd, max
            int sign;
            if (idx % 2 == 0) {
                sign = -1;
            }
            else {
                sign = 1;
            }
            constraints.matrix(idx, idx/2) = sign; 
        }

        // u/l bounds
        Eigen::VectorXd r = Eigen::VectorXd::Constant(6,robot_radius); 
        Eigen::VectorXd b(6);
        b << 
            -room_geometry.x_min,
            room_geometry.x_max,
            -room_geometry.y_min,
            room_geometry.y_max,
            -room_geometry.z_min,
            room_geometry.z_max;
        constraints.upper_bounds = kappa*(b-constraints.matrix*agent_position-r);
        constraints.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());

        // return output
        return constraints;
    }
}