#pragma once

// Measurement-space conditional-axis precision contraction. The geometric
// direction scores use only the nominally whitened sensor Jacobian. Optional
// Route A additionally compares conditional x information with the same-state
// body-x prior variance; no ROS, truth or trajectory error enters this core.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

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
  // Soft cross-sensor allocation for directions where LiDAR already has
  // reliable support. A visual row remains in the tightly coupled update,
  // but its effective gain is shared with the LiDAR support instead of being
  // left at a fixed 0.8 floor.
  bool visual_reference_allocation_enabled = false;
  double visual_reference_lidar_scale = 3.0;
  double visual_reference_floor = 0.10;
  // Route A: preserve the real whitened conditional x mode and contract its
  // measurement-space orthogonal complement. It is opt-in and visual-only.
  // Route A 保留真实白化条件前向模态，只软缩测量空间正交补；默认关闭。
  bool visual_route_a_enabled = false;
  bool visual_route_a_only = false;
  double visual_route_a_min_orthogonal_scale = 0.25;
  bool visual_route_a_fixed_diagnostic = false;
  double visual_route_a_fixed_orthogonal_scale = 0.50;
  // In dynamic mode, Ix_cond * var_prior_x is a dimensionless comparison
  // against the SAME state prior; no sensor-normalized scores are divided.
  double visual_route_a_prior_variance_x = 0.0;
  // Scale calibration uses H/base precision/prior only. This is the strength
  // of conditional x relative to the same prior, not y/z observability.
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
  double raw_translation_max_eigenvalue = 0;
  double residual_rms = 0;
  double raw_whitened_residual_rms = 0;
  double after_whitened_residual_rms = 0;
  double information_loss_min_eigenvalue = 0;
  double strong_axis_max_relative_column_change = 0;
  bool priority_active = false;
  double priority_gain = 1.0;
  bool route_a_applied = false;
  double route_a_orthogonal_scale = 1.0;
  double route_a_x_mode_information = 0.0;
  double route_a_information_to_prior_ratio = 0.0;
  double route_a_calibrated_ratio = 0.0;
  // Signed innovation of the residualized body-x translation mode. This is
  // diagnostic state only; it is computed from the current H, z and R_inv.
  // A cross-sensor gate may compare its sign without using truth.
  double signed_innovation_x = 0.0;
  double x_mode_information = 0.0;
  bool innovation_x_valid = false;
};

// Orthogonal projection uses an SVD of the nuisance columns, not an inverse
// normal equation. Column normalization makes rank decisions independent of
// radians versus metres; it does not change the exact nuisance column space.
inline Eigen::MatrixXd residualize(const Eigen::MatrixXd &target,
                                   const Eigen::MatrixXd &nuisance,
                                   double rank_tolerance) {
  if (!nuisance.cols() || !nuisance.rows()) return target;
  Eigen::MatrixXd scaled = nuisance;
  for (Eigen::Index c = 0; c < scaled.cols(); ++c) {
    double norm = scaled.col(c).norm();
    if (norm > 0) scaled.col(c) /= norm;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!svd.singularValues().size()) return target;
  int rank = 0;
  const double cutoff = rank_tolerance * svd.singularValues()(0);
  for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i)
    if (svd.singularValues()(i) > cutoff) ++rank;
  if (!rank) return target;
  const Eigen::MatrixXd basis = svd.matrixU().leftCols(rank);
  return (target - basis * (basis.transpose() * target)).eval();
}

inline Eigen::VectorXd conditionalResponse(const Eigen::MatrixXd &H, int axis,
                                           const Parameters &p) {
  const int index = 3 + axis;
  Eigen::MatrixXd nuisance(H.rows(), H.cols() - 1);
  for (int c = 0, j = 0; c < H.cols(); ++c) if (c != index) nuisance.col(j++) = H.col(c);
  return residualize(H.col(index), nuisance, p.rank_relative_tolerance);
}

inline void inspect(const Eigen::MatrixXd &H, const Parameters &p,
                    Eigen::Matrix3d &schur, Eigen::Vector3d &conditional,
                    Eigen::Vector3d &score, double &max_eigenvalue) {
  const Eigen::MatrixXd T = residualize(H.middleCols(3, 3), H.leftCols(3), p.rank_relative_tolerance);
  schur = T.transpose() * T;
  schur = (0.5 * (schur + schur.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(schur);
  max_eigenvalue = std::max(0.0, eig.eigenvalues().maxCoeff());
  const double zero = p.zero_information_relative_tolerance * max_eigenvalue;
  for (int axis = 0; axis < 3; ++axis) {
    const double info = conditionalResponse(H, axis, p).squaredNorm();
    conditional(axis) = info > zero ? info : 0.0;
  score(axis) = max_eigenvalue > 0 ? std::max(0.0, std::min(1.0,
      conditional(axis) / max_eigenvalue)) : 0;
  }
}

// Native state convention: delta_theta_body, delta_position_world. Rwb takes
// body FLU vectors to world. H and z are transformed TOGETHER in whitened
// measurement space; R_inv stays nominal. This preserves the native RHS sign,
// Kalman normal equations, and covariance update without column-wise scaling.
// 状态增量按机体系姿态、世界系位置排列；Rwb 将机体系 FLU 方向映射到世界系。
// 白化测量空间中的 H 与 z 同步变换，再还原至原测量尺度供原更新式使用。
inline Result contractImpl(Eigen::MatrixXd &H, Eigen::VectorXd &z,
                       const Eigen::VectorXd &R_inv,
                       const Eigen::Matrix3d &Rwb, const Parameters &p,
                       const Eigen::Vector3d *reference_score = nullptr) {
  if ((H.cols() != 6 && H.cols() != 7) || H.rows() != z.size() || z.size() != R_inv.size())
    throw std::invalid_argument("DOA axis dimensions");
  if (!(p.relative_cutoff > 0) || !(p.rank_relative_tolerance > 0) ||
      !(p.zero_information_relative_tolerance >= 0) ||
      !(p.visual_route_a_min_orthogonal_scale > 0.0 &&
        p.visual_route_a_min_orthogonal_scale <= 1.0) ||
      !(p.visual_route_a_fixed_orthogonal_scale > 0.0 &&
        p.visual_route_a_fixed_orthogonal_scale <= 1.0) ||
      !(p.visual_route_a_prior_variance_x >= 0.0) ||
      !(p.visual_route_a_calibration_gain > 0.0) ||
      !std::isfinite(p.visual_route_a_calibration_gain))
    throw std::invalid_argument("DOA axis parameters");
  if (!H.allFinite() || !z.allFinite() || !R_inv.allFinite() ||
      !Rwb.allFinite() || (R_inv.array() <= 0).any())
    throw std::invalid_argument("DOA axis non-finite or non-positive precision");
  Result out;
  if (!H.rows()) return out;
  Eigen::MatrixXd Hb = H;
  Hb.middleCols(3, 3) = H.middleCols(3, 3) * Rwb;
  const Eigen::VectorXd sqrt_precision = R_inv.array().sqrt();
  Eigen::MatrixXd A = sqrt_precision.asDiagonal() * Hb;
  Eigen::VectorXd b = sqrt_precision.asDiagonal() * z;
  Eigen::MatrixXd Araw;
  if (p.collect_diagnostics) Araw = A;
  out.raw_whitened_residual_rms = std::sqrt(b.squaredNorm() / b.size());
  if (p.collect_diagnostics) {
    const Eigen::MatrixXd Astate = A.leftCols(6);
    out.raw_information_body = Astate.transpose() * Astate;
    out.raw_rhs_body = Astate.transpose() * b;
    out.residual_rms = std::sqrt(z.squaredNorm() / z.size());
  }
  inspect(A, p, out.raw_translation_schur, out.conditional_information,
          out.score, out.raw_translation_max_eigenvalue);
  // Signed x innovation in the conditional measurement mode. The nuisance
  // projection removes the rotation/y/z response before evaluating u_x^T b,
  // so the result is a scalar in the same whitened measurement space as the
  // LiDAR and visual residuals. It is never normalized by truth or trajectory
  // error.
  const Eigen::VectorXd x_mode = conditionalResponse(A, 0, p);
  const double x_info = x_mode.squaredNorm();
  out.x_mode_information = x_info;
  const double x_zero = p.zero_information_relative_tolerance *
      std::max(1.0, out.raw_translation_max_eigenvalue);
  if (x_info > x_zero) {
    out.signed_innovation_x = x_mode.dot(b) / std::sqrt(x_info);
    out.innovation_x_valid = std::isfinite(out.signed_innovation_x);
  }
  out.gamma = (out.score.array() / p.relative_cutoff).min(1.0).max(0.0);
  // Cross-sensor allocation for the non-axial directions. LiDAR support is
  // measured from its own conditional information and only reduces visual
  // contribution in the same body direction. The floor keeps visual rows in
  // the tightly coupled update and avoids a hard sensor switch. X is left to
  // the visual priority branch because it is the corridor-degenerate axis.
  if (reference_score && p.visual_reference_allocation_enabled) {
    for (int axis = 0; axis < 3; ++axis) {
      if (axis == 0) continue;
      const double visual_support = std::max(0.0, std::min(1.0, out.gamma(axis)));
      const double lidar_support = std::max(0.0, std::min(1.0, (*reference_score)(axis)));
      const double denom = visual_support +
          p.visual_reference_lidar_scale * lidar_support;
      const double allocated = denom > 1e-12 ? visual_support / denom : visual_support;
      out.gamma(axis) = std::max(0.0, std::min(1.0,
          std::max(p.visual_reference_floor, allocated)));
    }
  }
  // Route A is a measurement-space operation. The normalized conditional
  // body-x response u is retained exactly, while every orthogonal measurement
  // component is scaled by s. Applying M=sI+(1-s)uu^T to BOTH A=W H and
  // b=W z is equivalent to a valid covariance update with R_inv unchanged:
  // M^T M <= I for 0<s<=1, so information cannot increase and the native
  // right-hand side remains consistent. Dynamic scale uses calibrated
  // conditional-x information relative to the same state prior. It measures
  // x-vs-prior influence, not y/z observability or a guarantee that scale
  // returns to one when a sensor recovers. No truth or trajectory error enters.
  // 同一矩阵同步作用于白化 Jacobian 和残差，信息不增；尺度来自条件信息与先验。
  if (p.visual_route_a_enabled && H.cols() == 7) {
    Eigen::VectorXd route_u = conditionalResponse(A, 0, p);
    const double route_info = route_u.squaredNorm();
    out.route_a_x_mode_information = route_info;
    const double route_zero = p.zero_information_relative_tolerance *
        std::max(1.0, out.raw_translation_max_eigenvalue);
    if (route_info > route_zero) {
      route_u /= std::sqrt(route_info);
      const double ratio = route_info * p.visual_route_a_prior_variance_x;
      out.route_a_information_to_prior_ratio = ratio;
      const double calibrated_ratio = p.visual_route_a_calibration_gain * ratio;
      out.route_a_calibrated_ratio = calibrated_ratio;
      const double confidence = calibrated_ratio / (1.0 + calibrated_ratio);
      const double floor = p.visual_route_a_min_orthogonal_scale;
      const double scale = p.visual_route_a_fixed_diagnostic ?
          p.visual_route_a_fixed_orthogonal_scale :
          std::sqrt(std::max(floor * floor,
              1.0 - (1.0 - floor * floor) * confidence));
      out.route_a_orthogonal_scale = scale;
      if (scale < 1.0 - 1e-15) {
        // Rank-one form: O(m*n), no dense m-by-m precision or covariance.
        const Eigen::RowVectorXd projected_H = route_u.transpose() * A;
        const double projected_z = route_u.dot(b);
        A *= scale;
        A.noalias() += (1.0 - scale) * route_u * projected_H;
        b *= scale;
        b.noalias() += (1.0 - scale) * route_u * projected_z;
        out.route_a_applied = true;
        out.applied = true;
      }
    }
  }
  if (p.visual_priority_enabled && H.cols() == 7 &&
      !p.visual_route_a_only &&
      out.score(0) >= p.visual_priority_score_min &&
      out.raw_whitened_residual_rms <= p.visual_priority_residual_max &&
      p.visual_priority_gain_max >= 1.0) {
    const double span = std::max(1e-12, 1.0 - p.visual_priority_score_min);
    const double t = std::max(0.0, std::min(1.0,
        (out.score(0) - p.visual_priority_score_min) / span));
    out.gamma(0) = std::max(out.gamma(0), 1.0 + (p.visual_priority_gain_max - 1.0) * t);
    out.priority_gain = out.gamma(0);
    out.priority_active = out.gamma(0) > 1.0;
  }

  // Raw scores fix the gate for this linearization. Each current residualized
  // mode is orthogonal to ALL other observed state columns. Rank-one left
  // contractions therefore preserve every other column, including strong axes.
  // Each step has singular values {sqrt(gamma), 1, ...}, hence information
  // can only decrease. No dense measurement covariance or n*n matrix is built.
  for (int axis = 0; axis < 3; ++axis) {
    if (H.cols() == 7 && (p.visual_route_a_only ||
        (p.visual_route_a_enabled && axis != 0))) continue;
    // Positive gain is reserved for the explicit high-confidence visual
    // x-priority path. Directional degradation remains contraction-only.
    const bool positive_priority = out.priority_active && axis == 0 && H.cols() == 7;
    if ((!positive_priority && out.gamma(axis) >= 1.0) ||
        out.conditional_information(axis) == 0) continue;
    Eigen::VectorXd u = conditionalResponse(A, axis, p);
    const double info = u.squaredNorm();
    out.target_mode_information(axis) = info;
    if (!(info > p.zero_information_relative_tolerance * out.raw_translation_max_eigenvalue)) continue;
    u /= std::sqrt(info);
    const double alpha = std::sqrt(out.gamma(axis)) - 1.0;
    const Eigen::RowVectorXd projected_H = u.transpose() * A;
    const double projected_z = u.dot(b);
    A.noalias() += alpha * u * projected_H;
    b.noalias() += alpha * u * projected_z;
    out.applied = true;
  }
  if (p.collect_diagnostics) {
    const Eigen::MatrixXd Astate_after = A.leftCols(6);
    out.after_information_body = Astate_after.transpose() * Astate_after;
    out.after_rhs_body = Astate_after.transpose() * b;
    out.after_whitened_residual_rms = std::sqrt(b.squaredNorm() / b.size());
    double ignored_max;
    inspect(A, p, out.after_translation_schur, out.after_conditional_information,
            out.after_score, ignored_max);
    Matrix6 loss = out.raw_information_body - out.after_information_body;
    loss = (0.5 * (loss + loss.transpose())).eval();
    out.information_loss_min_eigenvalue = Eigen::SelfAdjointEigenSolver<Matrix6>(loss).eigenvalues().minCoeff();
    for (int axis = 0; axis < 3; ++axis) if (out.gamma(axis) >= 1.0) {
      const double denom = std::max(1.0, Araw.col(3 + axis).norm());
      out.strong_axis_max_relative_column_change = std::max(out.strong_axis_max_relative_column_change,
          (A.col(3 + axis) - Araw.col(3 + axis)).norm() / denom);
    }
  }
  // Exact bitwise no-op if every mode is strong (or exactly zero). This is
  // necessary for a trustworthy same-binary enabled/disabled comparison.
  if (out.applied) {
    Hb = sqrt_precision.cwiseInverse().asDiagonal() * A;
    H.leftCols(3) = Hb.leftCols(3);
    H.middleCols(3, 3) = Hb.middleCols(3, 3) * Rwb.transpose();
    // H7 carries exposure as an additional nuisance column. Measurement-space
    // transforms must update it too; leaving it untouched would make the
    // returned Jacobian inconsistent with the transformed residual.
    if (H.cols() == 7) H.col(6) = Hb.col(6);
    z = sqrt_precision.cwiseInverse().asDiagonal() * b;
  }
  return out;
}

Result contract(Eigen::MatrixXd &H, Eigen::VectorXd &z,
                const Eigen::VectorXd &R_inv, const Eigen::Matrix3d &Rwb,
                const Parameters &p,
                const Eigen::Vector3d *reference_score = nullptr);
} // namespace doa_axis
