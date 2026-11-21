#include "sowa/core.h"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

bool near(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, double tolerance) {
  return (a - b).norm() <= tolerance * std::max(1.0, b.norm());
}

bool near(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double tolerance) {
  return (a - b).norm() <= tolerance * std::max(1.0, b.norm());
}

bool sameBytes(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  return a.size() == b.size() &&
      std::memcmp(a.data(), b.data(),
                  static_cast<std::size_t>(a.size()) * sizeof(double)) == 0;
}

bool sameBytes(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
  return a.size() == b.size() &&
      std::memcmp(a.data(), b.data(),
                  static_cast<std::size_t>(a.size()) * sizeof(double)) == 0;
}

void axisMaps(Eigen::MatrixXd &directions, Eigen::MatrixXd &nuisance,
              Eigen::Index state_size) {
  directions = Eigen::MatrixXd::Zero(state_size, 3);
  directions(0, 0) = 1.0;
  directions(1, 1) = 1.0;
  directions(2, 2) = 1.0;
  nuisance = Eigen::MatrixXd::Zero(state_size, state_size - 3);
  for (Eigen::Index i = 3; i < state_size; ++i)
    nuisance(i, i - 3) = 1.0;
}

void testDiagonalAndFullSpd() {
  const Eigen::Index rows = 12;
  const Eigen::Index state_size = 8;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, state_size);
  for (Eigen::Index i = 0; i < state_size; ++i)
    H(i, i) = i == 0 ? 0.1 : (i == 1 ? 3.0 : (i == 2 ? 2.0 : 1.0 + 0.1 * i));
  H(8, 0) = 0.02;
  H(9, 1) = -0.4;
  H(10, 2) = 0.3;
  H(11, 7) = 0.5;
  Eigen::VectorXd residual = Eigen::VectorXd::LinSpaced(rows, -0.4, 0.7);
  Eigen::VectorXd variance = Eigen::VectorXd::LinSpaced(rows, 0.7, 2.1);
  Eigen::MatrixXd D, N;
  axisMaps(D, N, state_size);

  sowa::Parameters parameters;
  parameters.enabled = true;
  parameters.relative_cutoff = 0.1;
  parameters.minimum_gain = 0.05;

  Eigen::MatrixXd H_diag = H;
  Eigen::VectorXd r_diag = residual;
  const sowa::Result diagonal =
      sowa::modulate(H_diag, r_diag, variance, D, N, parameters);
  require(diagonal.applied && diagonal.output_is_whitened,
          "diagonal covariance did not apply a modulation");
  require(diagonal.gain.size() == 3 && diagonal.gain(0) < 1.0 &&
              diagonal.gain(1) == 1.0 && diagonal.gain(2) == 1.0,
          "weak axis was not isolated from strong axes");
  require(diagonal.information_loss_psd &&
              diagonal.information_loss_min_eigenvalue > -1e-9,
          "diagonal covariance information loss is not PSD");
  require(near(diagonal.information_after, H_diag.transpose() * H_diag, 1e-12),
          "reported modulated information does not match H");
  require(near(diagonal.rhs_after, H_diag.transpose() * r_diag, 1e-12),
          "reported modulated rhs does not match H and residual");

  Eigen::MatrixXd H_fast = H;
  Eigen::VectorXd r_fast = residual;
  sowa::Parameters fast_parameters = parameters;
  fast_parameters.collect_diagnostics = false;
  const sowa::Result fast = sowa::modulate(
      H_fast, r_fast, variance, D, N, fast_parameters);
  require(fast.applied && !fast.diagnostics_collected,
          "diagnostic-off fast path reported diagnostics or skipped scheduling");
  require(sameBytes(H_fast, H_diag) && sameBytes(r_fast, r_diag),
          "diagnostics changed the modulated measurement output");

  Eigen::MatrixXd covariance = variance.asDiagonal();
  Eigen::MatrixXd H_full = H;
  Eigen::VectorXd r_full = residual;
  const sowa::Result full =
      sowa::modulate(H_full, r_full, covariance, D, N, parameters);
  require(full.applied && full.information_loss_psd,
          "full SPD covariance did not apply a valid modulation");
  require(near(H_full, H_diag, 2e-12) && near(r_full, r_diag, 2e-12),
          "diagonal and equivalent full SPD covariance disagree");
  require(near(full.information_after, diagonal.information_after, 2e-12) &&
              near(full.rhs_after, diagonal.rhs_after, 2e-12),
          "covariance representations produced different information");

  Eigen::MatrixXd L = Eigen::MatrixXd::Identity(rows, rows);
  for (Eigen::Index i = 1; i < rows; ++i) {
    L(i, i - 1) = 0.12;
    if (i > 1) L(i, i - 2) = -0.04;
  }
  const Eigen::MatrixXd correlated_covariance = L * L.transpose();
  Eigen::MatrixXd H_correlated = H;
  Eigen::VectorXd r_correlated = residual;
  const sowa::Result correlated = sowa::modulate(
      H_correlated, r_correlated, correlated_covariance, D, N, parameters);
  const Eigen::MatrixXd expected_information =
      H.transpose() * correlated_covariance.inverse() * H;
  const Eigen::VectorXd expected_rhs =
      H.transpose() * correlated_covariance.inverse() * residual;
  require(correlated.applied && correlated.output_is_whitened,
          "correlated SPD covariance did not apply a modulation");
  require(near(correlated.information_before, expected_information, 2e-11) &&
              near(correlated.rhs_before, expected_rhs, 2e-11),
          "full covariance whitening does not preserve the original normal equations");
  require(correlated.information_loss_psd,
          "correlated SPD covariance information loss is not PSD");
}

void testDisabledAndNeutralAreBitwiseNoOps() {
  const Eigen::Index rows = 8;
  const Eigen::Index state_size = 6;
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(rows, state_size);
  H(0, 0) = 0.2;
  Eigen::VectorXd residual = Eigen::VectorXd::LinSpaced(rows, 0.1, 0.8);
  Eigen::VectorXd variance = Eigen::VectorXd::Ones(rows);
  Eigen::MatrixXd D, N;
  axisMaps(D, N, state_size);
  const Eigen::MatrixXd H_original = H;
  const Eigen::VectorXd residual_original = residual;

  sowa::Parameters disabled;
  const sowa::Result disabled_result =
      sowa::modulate(H, residual, variance, D, N, disabled);
  require(!disabled_result.applied && !disabled_result.output_is_whitened,
          "disabled mode reported work");
  require(sameBytes(H, H_original) && sameBytes(residual, residual_original),
          "disabled mode changed input bytes");

  sowa::Parameters neutral;
  neutral.enabled = true;
  neutral.relative_cutoff = 0.01;
  const sowa::Result neutral_result =
      sowa::modulate(H, residual, variance, D, N, neutral);
  require(!neutral_result.applied && !neutral_result.output_is_whitened,
          "neutral schedule reported modulation");
  require(sameBytes(H, H_original) && sameBytes(residual, residual_original),
          "neutral schedule changed input bytes");
}

void testZeroAndRepeatedDirectionsDoNotInventInformation() {
  const Eigen::Index rows = 7;
  const Eigen::Index state_size = 5;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, state_size);
  H(0, 1) = 2.0;
  H(1, 2) = 3.0;
  H(2, 3) = 1.0;
  H(3, 4) = 1.0;
  Eigen::VectorXd residual = Eigen::VectorXd::LinSpaced(rows, -1.0, 1.0);
  Eigen::VectorXd variance = Eigen::VectorXd::Ones(rows);
  Eigen::MatrixXd D, N;
  axisMaps(D, N, state_size);
  const Eigen::MatrixXd H_before = H;
  const Eigen::VectorXd residual_before = residual;

  sowa::Parameters parameters;
  parameters.enabled = true;
  parameters.relative_cutoff = 0.1;
  const sowa::Result zero =
      sowa::modulate(H, residual, variance, D, N, parameters);
  require(zero.conditional_information(0) == 0.0 && zero.gain(0) == 1.0,
          "zero-information direction was assigned a non-neutral gain");
  require(!zero.applied && sameBytes(H, H_before) &&
              sameBytes(residual, residual_before),
          "zero-information direction changed the measurement");

  H.setZero();
  H(0, 0) = 2.0;
  H(0, 1) = 2.0;
  H(1, 2) = 2.0;
  H(2, 3) = 1.0;
  H(3, 4) = 1.0;
  const Eigen::MatrixXd repeated_before = H;
  const Eigen::VectorXd residual_repeated_before = residual;
  const sowa::Result repeated =
      sowa::modulate(H, residual, variance, D, N, parameters);
  require(repeated.conditional_information(0) < 1e-24 &&
              repeated.conditional_information(1) < 1e-24,
          "repeated measurement responses were counted as independent support");
  require(!repeated.applied && sameBytes(H, repeated_before) &&
              sameBytes(residual, residual_repeated_before),
          "repeated directions introduced a measurement-space update");
}

void testV076RouteAAndPriorityExtensions() {
  const Eigen::Index rows = 12;
  const Eigen::Index state_size = 7;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, state_size);
  H(0, 0) = 0.1;
  H(1, 1) = 3.0;
  H(2, 2) = 2.0;
  H(3, 3) = 1.0;
  H(4, 4) = 1.0;
  H(5, 5) = 1.0;
  H(6, 6) = 0.5;
  const Eigen::MatrixXd H_original = H;
  Eigen::VectorXd residual = Eigen::VectorXd::Constant(rows, 0.01);
  Eigen::VectorXd variance = Eigen::VectorXd::Ones(rows);
  Eigen::MatrixXd D, N;
  axisMaps(D, N, state_size);
  Eigen::MatrixXd D7(state_size, 3);
  D7.leftCols(3) = D;
  N = Eigen::MatrixXd::Zero(state_size, 4);
  for (Eigen::Index i = 3; i < state_size; ++i) N(i, i - 3) = 1.0;

  sowa::Parameters route;
  route.enabled = true;
  route.relative_cutoff = 0.01;
  route.route_a_enabled = true;
  route.route_a_only = true;
  route.route_a_direction = 0;
  route.route_a_prior_variance = 10.0;
  route.route_a_min_orthogonal_scale = 0.25;
  const Eigen::VectorXd route_mode = H_original * D7.col(0);
  Eigen::VectorXd mode_projected = route_mode;
  for (Eigen::Index axis = 1; axis < D7.cols(); ++axis) {
    const Eigen::VectorXd other = H_original * D7.col(axis);
    if (other.squaredNorm() > 0.0)
      mode_projected -= other * (other.dot(mode_projected) / other.squaredNorm());
  }
  for (Eigen::Index col = 0; col < N.cols(); ++col) {
    const Eigen::VectorXd other = H_original * N.col(col);
    if (other.squaredNorm() > 0.0)
      mode_projected -= other * (other.dot(mode_projected) / other.squaredNorm());
  }
  const Eigen::VectorXd normalized_mode = mode_projected.normalized();
  const sowa::Result route_result =
      sowa::modulate(H, residual, variance, D7, N, route);
  require(route_result.route_a_applied && route_result.route_a_orthogonal_scale < 1.0,
          "v0.7.6 Route A did not activate");
  require(!route_result.priority_active,
          "Route A-only mode unexpectedly entered visual priority gain");
  require(std::abs(normalized_mode.dot(H * D7.col(0)) -
                   normalized_mode.dot(H_original * D7.col(0))) < 1e-10,
          "Route A failed to preserve selected conditional mode");
  require(route_result.information_loss_psd,
          "Route A-only orthogonal contraction increased information");

  H = H_original;
  residual.setConstant(rows, 0.01);
  sowa::Parameters priority;
  priority.enabled = true;
  priority.relative_cutoff = 0.01;
  priority.minimum_gain = 0.05;
  priority.visual_priority_enabled = true;
  priority.priority_direction = 1;
  priority.priority_score_min = 0.55;
  priority.priority_residual_max = 1.0;
  priority.priority_gain_max = 1.5;
  const sowa::Result priority_result =
      sowa::modulate(H, residual, variance, D7, N, priority);
  require(priority_result.priority_active && priority_result.priority_gain > 1.0,
          "v0.7.6 positive visual priority path did not activate");
  require(!priority_result.information_loss_psd,
          "information increase from visual priority was misreported as PSD loss");
}

void testPriorityGateAdvancesOncePerObservation() {
  sowa::PriorityGate gate;
  sowa::PriorityGateParameters parameters;
  parameters.score_min = 0.5;
  parameters.residual_max = 1.0;
  parameters.minimum_consecutive_frames = 2;
  parameters.release_consecutive_frames = 2;
  parameters.score_ema_alpha = 1.0;
  parameters.stability_tolerance = 0.1;
  const bool first = gate.observe(1, 0.8, 0.2, 0.3, true, 0.4, true, parameters);
  const bool repeated = gate.observe(1, 0.8, 0.2, 0.3, true, 0.4, true, parameters);
  require(!first && !repeated && gate.goodStreak() == 1,
          "priority gate counted repeated solver iterations as frames");
  const bool second = gate.observe(2, 0.8, 0.2, 0.3, true, 0.4, true, parameters);
  require(second && gate.goodStreak() == 2,
          "priority gate did not admit stable matching observations");
  require(gate.observe(3, 0.8, 0.2, 0.3, true, 0.0, false, parameters),
          "priority gate released on first bad observation");
  require(!gate.observe(4, 0.8, 0.2, -0.3, true, 0.4, true, parameters),
          "priority gate failed to release after consecutive inconsistent observations");
  gate.reset();
  require(!gate.active() && gate.goodStreak() == 0 && gate.badStreak() == 0,
          "priority gate reset failed");
}

void testSyntheticSensorToStateDirectionMap() {
  const Eigen::Index rows = 7;
  const Eigen::Index state_size = 5;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, state_size);
  H(0, 0) = 3.0;
  H(1, 1) = 0.1;
  H(2, 2) = 2.0;
  H(3, 3) = 1.0;
  H(4, 4) = 1.0;
  Eigen::VectorXd residual = Eigen::VectorXd::LinSpaced(rows, -0.1, 0.2);
  Eigen::MatrixXd T_sensor_to_state = Eigen::MatrixXd::Identity(state_size, state_size);
  T_sensor_to_state(0, 0) = 0.0;
  T_sensor_to_state(1, 1) = 0.0;
  T_sensor_to_state(0, 1) = -1.0;
  T_sensor_to_state(1, 0) = 1.0;
  Eigen::MatrixXd sensor_basis = Eigen::MatrixXd::Zero(state_size, 3);
  sensor_basis.topRows(3).setIdentity();
  const Eigen::MatrixXd direction_map = T_sensor_to_state * sensor_basis;
  Eigen::MatrixXd nuisance_map = Eigen::MatrixXd::Zero(state_size, 2);
  nuisance_map(3, 0) = 1.0;
  nuisance_map(4, 1) = 1.0;
  Eigen::MatrixXd covariance_factor = Eigen::MatrixXd::Identity(rows, rows);
  for (Eigen::Index i = 1; i < rows; ++i)
    covariance_factor(i, i - 1) = 0.08;
  const Eigen::MatrixXd covariance = covariance_factor * covariance_factor.transpose();

  sowa::Parameters parameters;
  parameters.enabled = true;
  parameters.relative_cutoff = 0.05;
  parameters.minimum_gain = 0.05;
  const sowa::Result result = sowa::modulate(
      H, residual, covariance, direction_map, nuisance_map, parameters);
  require(result.applied && result.gain(0) < 1.0 && result.gain(1) == 1.0,
          "sensor-to-state rotation did not rotate the weak direction into sensor x");
  require(result.output_is_whitened && result.information_loss_psd,
          "mapped full-covariance modulation did not preserve PSD information loss");
}

}  // namespace

int main() {
  try {
    testDiagonalAndFullSpd();
    testDisabledAndNeutralAreBitwiseNoOps();
    testZeroAndRepeatedDirectionsDoNotInventInformation();
    testV076RouteAAndPriorityExtensions();
    testPriorityGateAdvancesOncePerObservation();
    testSyntheticSensorToStateDirectionMap();
    std::cout << "PASS: covariance whitening, arbitrary state maps, synchronized "
                 "measurement modulation, PSD information loss, and exact no-op modes\n";
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
