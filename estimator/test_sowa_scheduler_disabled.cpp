#include "doa_axis_scheduler.h"

#include <Eigen/Core>

#include <cstring>
#include <iostream>

int main() {
  doa_axis::Scheduler scheduler;
  Eigen::MatrixXd H = Eigen::MatrixXd::Random(13, 7);
  Eigen::VectorXd residual = Eigen::VectorXd::Random(13);
  const Eigen::MatrixXd H_before = H;
  const Eigen::VectorXd residual_before = residual;
  const Eigen::VectorXd precision = Eigen::VectorXd::Ones(13);
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  scheduler.apply(H, residual, precision, rotation, 0);
  const bool same_H = std::memcmp(
      H.data(), H_before.data(),
      static_cast<std::size_t>(H.size()) * sizeof(double)) == 0;
  const bool same_residual = std::memcmp(
      residual.data(), residual_before.data(),
      static_cast<std::size_t>(residual.size()) * sizeof(double)) == 0;
  if (!same_H || !same_residual) {
    std::cerr << "disabled SOWA scheduler changed the measurement block\n";
    return 1;
  }
  std::cout << "PASS: disabled scheduler returns before the numerical core and "
               "preserves H/residual bytes\n";
  return 0;
}
