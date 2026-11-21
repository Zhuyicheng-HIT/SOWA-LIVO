#include "sowa/v076_compat.h"

namespace doa_axis {

Result contract(Eigen::MatrixXd &H, Eigen::VectorXd &z,
                const Eigen::VectorXd &R_inv, const Eigen::Matrix3d &Rwb,
                const Parameters &parameters,
                const Eigen::Vector3d *reference_score) {
  return contractImpl(H, z, R_inv, Rwb, parameters, reference_score);
}

}  // namespace doa_axis
