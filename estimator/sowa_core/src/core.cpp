#include "sowa/core.h"

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sowa {
namespace {

void validate(const Eigen::MatrixXd &jacobian,
              const Eigen::VectorXd &residual,
              const Eigen::MatrixXd &direction_map,
              const Eigen::MatrixXd &nuisance_map,
              const Parameters &parameters) {
  const Eigen::Index state_size = jacobian.cols();
  if (jacobian.rows() != residual.size() ||
      direction_map.rows() != state_size || nuisance_map.rows() != state_size)
    throw std::invalid_argument("DOA core dimension mismatch");
  if (!jacobian.allFinite() || !residual.allFinite() ||
      !direction_map.allFinite() || !nuisance_map.allFinite())
    throw std::invalid_argument("DOA core input contains non-finite values");
  if (direction_map.cols() == 0)
    throw std::invalid_argument("DOA core requires at least one target direction");
  if (!(parameters.relative_cutoff > 0.0) ||
      !(parameters.minimum_gain >= 0.0 && parameters.minimum_gain <= 1.0) ||
      !(parameters.rank_relative_tolerance > 0.0) ||
      !(parameters.zero_information_relative_tolerance >= 0.0))
    throw std::invalid_argument("DOA core parameter out of range");
  if (parameters.route_a_enabled &&
      (parameters.route_a_direction < 0 || parameters.route_a_direction >= direction_map.cols() ||
       !(parameters.route_a_min_orthogonal_scale > 0.0 &&
         parameters.route_a_min_orthogonal_scale <= 1.0) ||
       !(parameters.route_a_fixed_orthogonal_scale > 0.0 &&
         parameters.route_a_fixed_orthogonal_scale <= 1.0) ||
       !(parameters.route_a_prior_variance >= 0.0) ||
       !(parameters.route_a_calibration_gain > 0.0) ||
       !std::isfinite(parameters.route_a_calibration_gain)))
    throw std::invalid_argument("DOA core Route A parameter out of range");
  if (parameters.visual_priority_enabled &&
      (parameters.priority_direction < 0 ||
       parameters.priority_direction >= direction_map.cols() ||
       !(parameters.priority_score_min >= 0.0 &&
         parameters.priority_score_min <= 1.0) ||
       !(parameters.priority_residual_max >= 0.0) ||
       !(parameters.priority_gain_max >= 1.0) ||
       !std::isfinite(parameters.priority_gain_max)))
    throw std::invalid_argument("DOA core priority parameter out of range");

  Eigen::MatrixXd state_map(state_size,
                            direction_map.cols() + nuisance_map.cols());
  state_map.leftCols(direction_map.cols()) = direction_map;
  if (nuisance_map.cols())
    state_map.rightCols(nuisance_map.cols()) = nuisance_map;
  if (state_size > 0) {
    for (Eigen::Index col = 0; col < state_map.cols(); ++col) {
      const double norm = state_map.col(col).norm();
      if (norm > 0.0) state_map.col(col) /= norm;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(state_map);
    qr.setThreshold(parameters.rank_relative_tolerance);
    if (qr.rank() != state_size)
      throw std::invalid_argument("direction and nuisance maps do not span state");
  }
}

Eigen::MatrixXd orthogonalResidual(const Eigen::VectorXd &target,
                                  const Eigen::MatrixXd &nuisance,
                                  double rank_tolerance) {
  // Remove the part explainable by coupled directions and nuisance states.
  // 投影掉耦合方向和干扰状态能够解释的部分，留下目标方向独有的响应。
  if (nuisance.cols() == 0 || nuisance.rows() == 0) return target;

  Eigen::MatrixXd scaled = nuisance;
  for (Eigen::Index col = 0; col < scaled.cols(); ++col) {
    const double norm = scaled.col(col).norm();
    if (norm > 0.0) scaled.col(col) /= norm;
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(scaled);
  qr.setThreshold(rank_tolerance);
  const Eigen::Index rank = qr.rank();
  if (rank == 0) return target;
  const Eigen::MatrixXd basis =
      qr.householderQ() * Eigen::MatrixXd::Identity(nuisance.rows(), rank);
  return target - basis * (basis.transpose() * target);
}

Result modulateWhitened(Eigen::MatrixXd &jacobian,
                        Eigen::VectorXd &residual,
                        const Eigen::MatrixXd &direction_map,
                        const Eigen::MatrixXd &nuisance_map,
                        const Parameters &parameters) {
  Result result;
  // Preserve the exact baseline observation block when scheduling is off.
  // 关闭调度时直接返回，保证输入观测块与原始基线完全一致。
  if (!parameters.enabled) return result;
  result.diagnostics_collected = parameters.collect_diagnostics;

  validate(jacobian, residual, direction_map, nuisance_map, parameters);
  if (jacobian.rows() == 0 || jacobian.cols() == 0) return result;

  const Eigen::Index rows = jacobian.rows();
  const Eigen::Index directions = direction_map.cols();
  result.whitened_residual_rms = residual.norm() / std::sqrt(double(rows));
  const Eigen::MatrixXd original_jacobian = jacobian;
  const Eigen::VectorXd original_residual = residual;
  const Eigen::MatrixXd direction_jacobian = original_jacobian * direction_map;
  const Eigen::MatrixXd nuisance_jacobian = original_jacobian * nuisance_map;
  Eigen::MatrixXd modes(rows, directions);
  result.conditional_information = Eigen::VectorXd::Zero(directions);
  // Compare each mapped direction only after the other mapped directions and
  // nuisance responses have been projected out.
  // 逐方向计算条件信息时，先剔除其余方向与干扰状态的共同响应。
  for (Eigen::Index axis = 0; axis < directions; ++axis) {
    Eigen::MatrixXd other_directions(rows, directions - 1);
    for (Eigen::Index col = 0, other = 0; col < directions; ++col)
      if (col != axis) other_directions.col(other++) = direction_jacobian.col(col);

    Eigen::MatrixXd nuisance(rows, nuisance_jacobian.cols() + other_directions.cols());
    if (nuisance_jacobian.cols())
      nuisance.leftCols(nuisance_jacobian.cols()) = nuisance_jacobian;
    if (other_directions.cols())
      nuisance.rightCols(other_directions.cols()) = other_directions;

    modes.col(axis) = orthogonalResidual(direction_jacobian.col(axis), nuisance,
                                         parameters.rank_relative_tolerance);
    result.conditional_information(axis) = modes.col(axis).squaredNorm();
  }

  const Eigen::MatrixXd directional_information = modes.transpose() * modes;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(
      0.5 * (directional_information + directional_information.transpose()));
  if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite())
    throw std::runtime_error("DOA core directional eigensolver failed");
  const double maximum_information =
      std::max(0.0, spectrum.eigenvalues().maxCoeff());

  result.score = Eigen::VectorXd::Zero(directions);
  result.gain = Eigen::VectorXd::Ones(directions);
  result.signed_innovation = Eigen::VectorXd::Zero(directions);
  result.innovation_valid.assign(static_cast<std::size_t>(directions), 0);
  if (maximum_information > 0.0) {
    const double zero_threshold =
        parameters.zero_information_relative_tolerance * maximum_information;
    for (Eigen::Index axis = 0; axis < directions; ++axis) {
      const double information = result.conditional_information(axis);
      if (information <= zero_threshold) continue;
      const double score = std::max(0.0, std::min(1.0,
          information / maximum_information));
      result.score(axis) = score;
      result.gain(axis) = std::max(parameters.minimum_gain,
          std::min(1.0, score / parameters.relative_cutoff));
      result.signed_innovation(axis) =
          modes.col(axis).dot(original_residual) / std::sqrt(information);
      result.innovation_valid[static_cast<std::size_t>(axis)] =
          std::isfinite(result.signed_innovation(axis)) ? 1 : 0;
    }
  }

  Eigen::MatrixXd transformed_jacobian = original_jacobian;
  Eigen::VectorXd transformed_residual = original_residual;

  if (parameters.route_a_enabled &&
      parameters.route_a_direction < directions) {
    const Eigen::Index axis = parameters.route_a_direction;
    const double information = result.conditional_information(axis);
    const double zero_threshold = parameters.zero_information_relative_tolerance *
        std::max(1.0, maximum_information);
    if (information > zero_threshold) {
      const Eigen::VectorXd mode = modes.col(axis) / std::sqrt(information);
      result.route_a_information_to_prior_ratio =
          information * parameters.route_a_prior_variance;
      result.route_a_calibrated_ratio = parameters.route_a_calibration_gain *
          result.route_a_information_to_prior_ratio;
      const double confidence = result.route_a_calibrated_ratio /
          (1.0 + result.route_a_calibrated_ratio);
      const double floor = parameters.route_a_min_orthogonal_scale;
      const double scale = parameters.route_a_fixed_diagnostic ?
          parameters.route_a_fixed_orthogonal_scale :
          std::sqrt(std::max(floor * floor,
              1.0 - (1.0 - floor * floor) * confidence));
      result.route_a_orthogonal_scale = scale;
      if (scale < 1.0 - 1e-15) {
        // M = sI + (1-s)uu^T keeps the selected unit mode and scales its
        // orthogonal complement. Apply M to H and r together; M^T M <= I.
        // M 保留目标单位模态并软缩其正交补；必须同步变换 H 与残差，且 M^T M 不超过 I。
        const Eigen::RowVectorXd projection = mode.transpose() * transformed_jacobian;
        const double residual_projection = mode.dot(transformed_residual);
        transformed_jacobian *= scale;
        transformed_jacobian.noalias() +=
            (1.0 - scale) * mode * projection;
        transformed_residual *= scale;
        transformed_residual.noalias() +=
            (1.0 - scale) * mode * residual_projection;
        result.route_a_applied = true;
        result.applied = true;
      }
    }
  }

  if (!parameters.route_a_only && parameters.visual_priority_enabled &&
      result.score(parameters.priority_direction) >= parameters.priority_score_min &&
      result.whitened_residual_rms <= parameters.priority_residual_max &&
      parameters.priority_gain_max >= 1.0) {
    const double span = std::max(1e-12, 1.0 - parameters.priority_score_min);
    const double t = std::max(0.0, std::min(1.0,
        (result.score(parameters.priority_direction) -
         parameters.priority_score_min) / span));
    const double priority = 1.0 + (parameters.priority_gain_max - 1.0) * t;
    result.gain(parameters.priority_direction) =
        std::max(result.gain(parameters.priority_direction), priority);
    result.priority_gain = result.gain(parameters.priority_direction);
    result.priority_active = result.priority_gain > 1.0;
  }

  Eigen::MatrixXd information_before;
  Eigen::VectorXd rhs_before;
  if (parameters.collect_diagnostics) {
    information_before = original_jacobian.transpose() * original_jacobian;
    rhs_before = original_jacobian.transpose() * original_residual;
  }
  for (Eigen::Index axis = 0;
       axis < (parameters.route_a_only ? 0 : directions); ++axis) {
    if (result.gain(axis) == 1.0) continue;
    const double norm = modes.col(axis).norm();
    if (!(norm > 0.0)) continue;
    const Eigen::VectorXd mode = modes.col(axis) / norm;
    const double scale = std::sqrt(result.gain(axis));
    const Eigen::RowVectorXd projected_jacobian =
        mode.transpose() * transformed_jacobian;
    const double projected_residual = mode.dot(transformed_residual);
    transformed_jacobian.noalias() += (scale - 1.0) * mode * projected_jacobian;
    transformed_residual.noalias() += (scale - 1.0) * mode * projected_residual;
    result.applied = true;
  }
  if (!result.applied) return result;

  if (parameters.collect_diagnostics) {
    result.information_before = information_before;
    result.rhs_before = rhs_before;
    result.information_after =
        transformed_jacobian.transpose() * transformed_jacobian;
    result.rhs_after = transformed_jacobian.transpose() * transformed_residual;
    Eigen::MatrixXd loss = result.information_before - result.information_after;
    loss = 0.5 * (loss + loss.transpose()).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> loss_spectrum(loss);
    if (loss_spectrum.info() != Eigen::Success ||
        !loss_spectrum.eigenvalues().allFinite())
      throw std::runtime_error("DOA core information-loss eigensolver failed");
    result.information_loss_min_eigenvalue =
        loss_spectrum.eigenvalues().minCoeff();
    const double tolerance = 1e-10 *
        std::max(1.0, result.information_before.norm());
    result.information_loss_psd =
        result.information_loss_min_eigenvalue >= -tolerance;
  }

  jacobian = std::move(transformed_jacobian);
  residual = std::move(transformed_residual);
  result.output_is_whitened = true;
  return result;
}

}  // namespace

Result modulate(Eigen::MatrixXd &jacobian, Eigen::VectorXd &residual,
                const Eigen::VectorXd &diagonal_variance,
                const Eigen::MatrixXd &direction_map,
                const Eigen::MatrixXd &nuisance_map,
                const Parameters &parameters) {
  Result result;
  if (!parameters.enabled) return result;
  if (jacobian.rows() != residual.size() ||
      diagonal_variance.size() != jacobian.rows())
    throw std::invalid_argument("DOA core diagonal covariance dimension mismatch");
  if (!diagonal_variance.allFinite() || (diagonal_variance.array() <= 0.0).any())
    throw std::invalid_argument("DOA core diagonal covariance must be positive finite");

  // Whiten both sides of the observation with the same R^(-1/2) factor.
  // 对角协方差下，以相同的 R^(-1/2) 同步白化 Jacobian 和残差。
  Eigen::VectorXd inverse_sigma = diagonal_variance.array().sqrt().inverse();
  Eigen::MatrixXd whitened_jacobian = inverse_sigma.asDiagonal() * jacobian;
  Eigen::VectorXd whitened_residual = inverse_sigma.cwiseProduct(residual);
  result = modulateWhitened(whitened_jacobian, whitened_residual,
                            direction_map, nuisance_map, parameters);
  if (result.applied) {
    jacobian = std::move(whitened_jacobian);
    residual = std::move(whitened_residual);
    result.output_is_whitened = true;
  }
  return result;
}

Result modulate(Eigen::MatrixXd &jacobian, Eigen::VectorXd &residual,
                const Eigen::MatrixXd &covariance,
                const Eigen::MatrixXd &direction_map,
                const Eigen::MatrixXd &nuisance_map,
                const Parameters &parameters) {
  Result result;
  if (!parameters.enabled) return result;
  if (covariance.rows() != jacobian.rows() ||
      covariance.cols() != jacobian.rows() ||
      jacobian.rows() != residual.size())
    throw std::invalid_argument("DOA core covariance dimension mismatch");
  if (!covariance.allFinite())
    throw std::invalid_argument("DOA core covariance contains non-finite values");
  const double covariance_scale = std::max(1.0, covariance.norm());
  if ((covariance - covariance.transpose()).norm() > 1e-12 * covariance_scale)
    throw std::invalid_argument("DOA core covariance must be symmetric");

  const Eigen::MatrixXd symmetric_covariance =
      0.5 * (covariance + covariance.transpose()).eval();
  Eigen::LLT<Eigen::MatrixXd> llt(symmetric_covariance);
  if (llt.info() != Eigen::Success)
    throw std::invalid_argument("DOA core covariance must be positive definite");
  // LLT supplies L so L^(-1)R L^(-T)=I; both H and r use the same solve.
  // LLT 分解得到 L，并用同一个线性求解同步白化 H 与残差 r。
  Eigen::MatrixXd whitened_jacobian = llt.matrixL().solve(jacobian);
  Eigen::VectorXd whitened_residual = llt.matrixL().solve(residual);
  if (!whitened_jacobian.allFinite() || !whitened_residual.allFinite())
    throw std::runtime_error("DOA core whitening produced non-finite values");

  result = modulateWhitened(whitened_jacobian, whitened_residual,
                            direction_map, nuisance_map, parameters);
  if (result.applied) {
    jacobian = std::move(whitened_jacobian);
    residual = std::move(whitened_residual);
    result.output_is_whitened = true;
  }
  return result;
}

bool PriorityGate::observe(std::uint64_t observation_id, double score,
                           double whitened_residual, double signed_innovation,
                           bool innovation_valid, double reference_innovation,
                           bool reference_innovation_valid,
                           const PriorityGateParameters &parameters) {
  if (!(parameters.score_min >= 0.0 && parameters.score_min <= 1.0) ||
      !(parameters.residual_max >= 0.0) ||
      parameters.minimum_consecutive_frames < 1 ||
      parameters.release_consecutive_frames < 1 ||
      !(parameters.score_ema_alpha >= 0.0 && parameters.score_ema_alpha <= 1.0) ||
      !(parameters.stability_tolerance >= 0.0) ||
      !(parameters.innovation_zero_tolerance >= 0.0))
    throw std::invalid_argument("DOA priority gate parameter out of range");
  if (observation_seen_ && observation_id == last_observation_id_) return active_;
  observation_seen_ = true;
  last_observation_id_ = observation_id;

  if (!std::isfinite(score) || !std::isfinite(whitened_residual)) {
    good_streak_ = 0;
    ++bad_streak_;
    if (active_ && bad_streak_ >= parameters.release_consecutive_frames)
      active_ = false;
    history_valid_ = false;
    innovation_consistent_ = false;
    return active_;
  }

  score = std::max(0.0, std::min(1.0, score));
  const double alpha = parameters.score_ema_alpha;
  if (!history_valid_) {
    ema_score_ = score;
    ema_residual_ = whitened_residual;
    history_valid_ = true;
  } else {
    ema_score_ = alpha * score + (1.0 - alpha) * ema_score_;
    ema_residual_ = alpha * whitened_residual +
        (1.0 - alpha) * ema_residual_;
  }

  const bool lidar_near_zero = reference_innovation_valid &&
      std::isfinite(reference_innovation) &&
      std::abs(reference_innovation) <= parameters.innovation_zero_tolerance;
  const bool sign_match = reference_innovation_valid && innovation_valid &&
      std::isfinite(reference_innovation) && std::isfinite(signed_innovation) &&
      (reference_innovation == 0.0 || signed_innovation == 0.0 ||
       ((reference_innovation > 0.0) == (signed_innovation > 0.0)));
  innovation_consistent_ = innovation_valid &&
      std::isfinite(signed_innovation) && (sign_match || lidar_near_zero);

  const bool good = score >= parameters.score_min &&
      ema_score_ >= parameters.score_min &&
      whitened_residual <= parameters.residual_max &&
      ema_residual_ <= parameters.residual_max &&
      std::abs(score - ema_score_) <= parameters.stability_tolerance &&
      innovation_consistent_;
  if (good) {
    ++good_streak_;
    bad_streak_ = 0;
    if (!active_ && good_streak_ >= parameters.minimum_consecutive_frames)
      active_ = true;
  } else {
    good_streak_ = 0;
    ++bad_streak_;
    if (active_ && bad_streak_ >= parameters.release_consecutive_frames)
      active_ = false;
  }
  return active_;
}

void PriorityGate::reset() {
  active_ = false;
  history_valid_ = false;
  innovation_consistent_ = false;
  observation_seen_ = false;
  last_observation_id_ = 0;
  ema_score_ = 0.0;
  ema_residual_ = 0.0;
  good_streak_ = 0;
  bad_streak_ = 0;
}

}  // namespace sowa
