#pragma once

#include <Eigen/Core>

namespace doa_axis {

using Matrix6 = Eigen::Matrix<double, 6, 6>;

struct Parameters {
  bool collect_diagnostics = true;
  double relative_cutoff = 0.01;
  double rank_relative_tolerance = 1e-10;
  double zero_information_relative_tolerance = 1e-12;
  bool visual_priority_enabled = false;
  double visual_priority_score_min = 0.55;
  double visual_priority_residual_max = 1.0;
  double visual_priority_gain_max = 1.50;
  bool visual_reference_allocation_enabled = false;
  double visual_reference_lidar_scale = 3.0;
  double visual_reference_floor = 0.10;
  bool visual_route_a_enabled = false;
  bool visual_route_a_only = false;
  double visual_route_a_min_orthogonal_scale = 0.25;
  bool visual_route_a_fixed_diagnostic = false;
  double visual_route_a_fixed_orthogonal_scale = 0.50;
  double visual_route_a_prior_variance_x = 0.0;
  double visual_route_a_calibration_gain = 1.0;
};

struct Result {
  bool applied = false;
  Eigen::Vector3d conditional_information = Eigen::Vector3d::Zero();
  Eigen::Vector3d score = Eigen::Vector3d::Zero();
  Eigen::Vector3d gamma = Eigen::Vector3d::Ones();
  Eigen::Vector3d after_conditional_information = Eigen::Vector3d::Zero();
  Eigen::Vector3d after_score = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_mode_information = Eigen::Vector3d::Zero();
  Eigen::Matrix3d raw_translation_schur = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d after_translation_schur = Eigen::Matrix3d::Zero();
  Matrix6 raw_information_body = Matrix6::Zero();
  Matrix6 after_information_body = Matrix6::Zero();
  Eigen::Matrix<double, 6, 1> raw_rhs_body = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> after_rhs_body = Eigen::Matrix<double, 6, 1>::Zero();
  double raw_translation_max_eigenvalue = 0.0;
  double residual_rms = 0.0;
  double raw_whitened_residual_rms = 0.0;
  double after_whitened_residual_rms = 0.0;
  double information_loss_min_eigenvalue = 0.0;
  double strong_axis_max_relative_column_change = 0.0;
  bool priority_active = false;
  double priority_gain = 1.0;
  bool route_a_applied = false;
  double route_a_orthogonal_scale = 1.0;
  double route_a_x_mode_information = 0.0;
  double route_a_information_to_prior_ratio = 0.0;
  double route_a_calibrated_ratio = 0.0;
  double signed_innovation_x = 0.0;
  double x_mode_information = 0.0;
  bool innovation_x_valid = false;
};

Result contract(Eigen::MatrixXd &H, Eigen::VectorXd &z,
                const Eigen::VectorXd &R_inv, const Eigen::Matrix3d &Rwb,
                const Parameters &parameters,
                const Eigen::Vector3d *reference_score = nullptr);

}  // namespace doa_axis
