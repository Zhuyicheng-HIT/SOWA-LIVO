#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace sowa {

struct Parameters {
  bool enabled = false;
  bool collect_diagnostics = true;
  double relative_cutoff = 0.01;
  double minimum_gain = 0.05;
  double rank_relative_tolerance = 1e-10;
  double zero_information_relative_tolerance = 1e-12;

  // Optional v0.7.6 Route A: preserve one conditional mode and contract its
  // measurement-space orthogonal complement according to information/prior.
  bool route_a_enabled = false;
  bool route_a_only = false;
  Eigen::Index route_a_direction = 0;
  double route_a_min_orthogonal_scale = 0.25;
  bool route_a_fixed_diagnostic = false;
  double route_a_fixed_orthogonal_scale = 0.50;
  double route_a_prior_variance = 0.0;
  double route_a_calibration_gain = 1.0;

  // Optional v0.7.6 positive-gain path. Set enabled only after any desired
  // temporal stability gate has admitted the current sensor update.
  bool visual_priority_enabled = false;
  Eigen::Index priority_direction = 0;
  double priority_score_min = 0.55;
  double priority_residual_max = 1.0;
  double priority_gain_max = 1.50;
};

struct Result {
  bool applied = false;
  // True only when H and residual were replaced by their whitened, modulated
  // forms. In that case the caller must use identity measurement covariance.
  bool output_is_whitened = false;
  bool diagnostics_collected = false;
  Eigen::VectorXd conditional_information;
  Eigen::VectorXd score;
  Eigen::VectorXd gain;
  Eigen::VectorXd signed_innovation;
  std::vector<unsigned char> innovation_valid;
  double whitened_residual_rms = 0.0;
  bool route_a_applied = false;
  double route_a_orthogonal_scale = 1.0;
  double route_a_information_to_prior_ratio = 0.0;
  double route_a_calibrated_ratio = 0.0;
  bool priority_active = false;
  double priority_gain = 1.0;
  Eigen::MatrixXd information_before;
  Eigen::MatrixXd information_after;
  Eigen::VectorXd rhs_before;
  Eigen::VectorXd rhs_after;
  double information_loss_min_eigenvalue = 0.0;
  bool information_loss_psd = true;
};

// direction_map columns encode unit perturbations of the target state
// directions. nuisance_map spans all remaining state perturbations. Their
// combined columns must span the complete state space; either map may be
// empty. A transformed output is in whitened measurement coordinates.
Result modulate(Eigen::MatrixXd &jacobian, Eigen::VectorXd &residual,
                const Eigen::VectorXd &diagonal_variance,
                const Eigen::MatrixXd &direction_map,
                const Eigen::MatrixXd &nuisance_map,
                const Parameters &parameters);

Result modulate(Eigen::MatrixXd &jacobian, Eigen::VectorXd &residual,
                const Eigen::MatrixXd &covariance,
                const Eigen::MatrixXd &direction_map,
                const Eigen::MatrixXd &nuisance_map,
                const Parameters &parameters);

struct PriorityGateParameters {
  double score_min = 0.55;
  double residual_max = 1.0;
  int minimum_consecutive_frames = 3;
  int release_consecutive_frames = 2;
  double score_ema_alpha = 0.25;
  double stability_tolerance = 0.08;
  double innovation_zero_tolerance = 0.25;
};

// Sensor-agnostic temporal gate used by the v0.7.6 visual priority path.
// Call observe once per distinct sensor update; repeated IEKF iterations with
// the same observation_id do not advance its state.
class PriorityGate {
 public:
  bool observe(std::uint64_t observation_id, double score,
               double whitened_residual, double signed_innovation,
               bool innovation_valid, double reference_innovation,
               bool reference_innovation_valid,
               const PriorityGateParameters &parameters);
  bool active() const { return active_; }
  int goodStreak() const { return good_streak_; }
  int badStreak() const { return bad_streak_; }
  double scoreEma() const { return ema_score_; }
  double residualEma() const { return ema_residual_; }
  bool innovationConsistent() const { return innovation_consistent_; }
  void reset();

 private:
  bool active_ = false;
  bool history_valid_ = false;
  bool innovation_consistent_ = false;
  bool observation_seen_ = false;
  std::uint64_t last_observation_id_ = 0;
  double ema_score_ = 0.0;
  double ema_residual_ = 0.0;
  int good_streak_ = 0;
  int bad_streak_ = 0;
};

}  // namespace sowa
