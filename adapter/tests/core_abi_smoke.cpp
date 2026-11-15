#include "sowa/compat_api.h"
#include "sowa/core.h"
#include <cmath>
#include <cstring>
#include <iostream>

int main() {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 6);
  H.diagonal() << 1.0, 1.0, 1.0, 0.01, 1.0, 1.0;
  Eigen::VectorXd z = Eigen::VectorXd::Ones(6);
  const Eigen::VectorXd R_inv = Eigen::VectorXd::Ones(6);
  doa_axis::Parameters p;
  p.collect_diagnostics = true;
  const doa_axis::Result result = doa_axis::contract(
      H, z, R_inv, Eigen::Matrix3d::Identity(), p);
  std::cout << "applied=" << result.applied
            << " gamma_x=" << result.gamma(0)
            << " h_x=" << H(3,3) << " z_x=" << z(3)
            << " raw_x=" << result.raw_information_body(3,3)
            << " after_x=" << result.after_information_body(3,3)
            << " loss_min=" << result.information_loss_min_eigenvalue << '\n';
  const bool compat_ok = result.applied && std::abs(result.gamma(0) - 0.01) < 1e-10 &&
      std::abs(H(3,3) - 0.001) < 1e-10 &&
      std::abs(z(3) - 0.1) < 1e-10 &&
      std::abs(result.raw_information_body(3,3) - 0.0001) < 1e-10 &&
      std::abs(result.after_information_body(3,3) - 0.000001) < 1e-10 &&
      result.information_loss_min_eigenvalue > -1e-10;
  const Eigen::MatrixXd before_H = H;
  const Eigen::VectorXd before_z = z;
  Eigen::MatrixXd directions = Eigen::MatrixXd::Zero(6, 3);
  directions.bottomRows(3).setIdentity();
  Eigen::MatrixXd nuisance = Eigen::MatrixXd::Zero(6, 3);
  nuisance.topRows(3).setIdentity();
  sowa::Parameters disabled;
  const Eigen::VectorXd variance = Eigen::VectorXd::Ones(6);
  const sowa::Result noop = sowa::modulate(H, z, variance, directions, nuisance, disabled);
  const bool noop_ok = !noop.applied &&
      std::memcmp(H.data(), before_H.data(), H.size() * sizeof(double)) == 0 &&
      std::memcmp(z.data(), before_z.data(), z.size() * sizeof(double)) == 0;
  std::cout << "disabled_noop=" << noop_ok << '\n';
  return compat_ok && noop_ok ? 0 : 1;
}
