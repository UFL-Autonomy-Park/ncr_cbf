#include <ncr_cbf/saturation.hpp>

#include <stdexcept>
#include <cmath>
#include <limits>

namespace ncr_cbf{
    LinearConstraints constructSaturationConstraint(
        const Eigen::Vector3d& componentwise_vel_bound
    )
    {
        // input validations
        // is finite
        if (!componentwise_vel_bound.allFinite()) {
            throw std::invalid_argument("All values of componentwise_vel_bound must be finite");
        }

        // norm bounds
        if ((componentwise_vel_bound.array() <= 0.0).any()) {
            throw std::invalid_argument(
                "Each entry of componentwise_vel_bound must be > 0.0"
            );
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
        constraints.upper_bounds <<
            componentwise_vel_bound(0),
            componentwise_vel_bound(0),
            componentwise_vel_bound(1),
            componentwise_vel_bound(1),
            componentwise_vel_bound(2),
            componentwise_vel_bound(2);

        constraints.lower_bounds = Eigen::VectorXd::Constant(6,-std::numeric_limits<double>::infinity());

        // return output
        return constraints;
    }
}