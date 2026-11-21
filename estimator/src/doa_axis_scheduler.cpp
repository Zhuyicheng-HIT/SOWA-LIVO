#include "doa_axis_scheduler.h"
#include "sowa/core.h"
#include "sowa/v076_compat.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace doa_axis {
struct Scheduler::Impl {
  bool enabled = false;
  bool visual_role = false;
  bool reference_valid = false;
  Eigen::Vector3d reference_support = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_support = Eigen::Vector3d::Zero();
  Parameters parameters;
  bool priority_stable_gate_enabled = false;
  sowa::PriorityGateParameters priority_gate_parameters;
  sowa::PriorityGate priority_gate;
  double reference_signed_innovation_x = 0.0;
  bool reference_innovation_valid = false;
  unsigned long long priority_last_observation = std::numeric_limits<unsigned long long>::max();
  double last_signed_innovation_x = 0.0;
  bool last_innovation_valid = false;
  unsigned long long update_index = 0;
};
Scheduler::Scheduler(): impl_(new Impl) {}
Scheduler::~Scheduler() = default;
void Scheduler::configure(ros::NodeHandle &nh, bool visual_role) {
  impl_->visual_role = visual_role;
  if (visual_role) nh.param<bool>("sowa/visual_enabled", impl_->enabled, false);
  else nh.param<bool>("sowa/enabled", impl_->enabled, false);
  nh.param<double>("sowa/relative_cutoff", impl_->parameters.relative_cutoff, 0.01);
  nh.param<double>("sowa/rank_relative_tolerance", impl_->parameters.rank_relative_tolerance, 1e-10);
  nh.param<double>("sowa/zero_information_relative_tolerance", impl_->parameters.zero_information_relative_tolerance, 1e-12);
  if (visual_role) {
    nh.param<bool>("sowa/visual_priority_enabled", impl_->parameters.visual_priority_enabled, false);
    nh.param<double>("sowa/visual_priority_score_min", impl_->parameters.visual_priority_score_min, 0.55);
    nh.param<double>("sowa/visual_priority_residual_max", impl_->parameters.visual_priority_residual_max, 1.0);
    nh.param<double>("sowa/visual_priority_gain_max", impl_->parameters.visual_priority_gain_max, 1.50);
    nh.param<bool>("sowa/visual_route_a_enabled", impl_->parameters.visual_route_a_enabled, false);
    nh.param<bool>("sowa/visual_route_a_only", impl_->parameters.visual_route_a_only, false);
    nh.param<double>("sowa/visual_route_a_min_orthogonal_scale", impl_->parameters.visual_route_a_min_orthogonal_scale, 0.25);
    nh.param<bool>("sowa/visual_route_a_fixed_diagnostic", impl_->parameters.visual_route_a_fixed_diagnostic, false);
    nh.param<double>("sowa/visual_route_a_fixed_orthogonal_scale", impl_->parameters.visual_route_a_fixed_orthogonal_scale, 0.50);
    nh.param<double>("sowa/visual_route_a_calibration_gain", impl_->parameters.visual_route_a_calibration_gain, 1.0);
    nh.param<bool>("sowa/visual_reference_allocation_enabled", impl_->parameters.visual_reference_allocation_enabled, false);
    nh.param<double>("sowa/visual_reference_lidar_scale", impl_->parameters.visual_reference_lidar_scale, 3.0);
    nh.param<double>("sowa/visual_reference_floor", impl_->parameters.visual_reference_floor, 0.10);
    nh.param<bool>("sowa/visual_priority_stable_gate_enabled", impl_->priority_stable_gate_enabled, false);
    nh.param<int>("sowa/visual_priority_min_consecutive_frames", impl_->priority_gate_parameters.minimum_consecutive_frames, 3);
    nh.param<int>("sowa/visual_priority_release_consecutive_frames", impl_->priority_gate_parameters.release_consecutive_frames, 2);
    nh.param<double>("sowa/visual_priority_score_ema_alpha", impl_->priority_gate_parameters.score_ema_alpha, 0.25);
    nh.param<double>("sowa/visual_priority_stability_tolerance", impl_->priority_gate_parameters.stability_tolerance, 0.08);
    nh.param<double>("sowa/visual_priority_innovation_zero_tolerance", impl_->priority_gate_parameters.innovation_zero_tolerance, 0.25);
    impl_->priority_gate_parameters.score_min = impl_->parameters.visual_priority_score_min;
    impl_->priority_gate_parameters.residual_max = impl_->parameters.visual_priority_residual_max;
    impl_->priority_gate.reset();
  }
  impl_->parameters.collect_diagnostics = false;
  if (impl_->enabled) ROS_INFO("SOWA FLU conditional-axis %s weighting enabled; relative cutoff %.9g", visual_role ? "visual" : "LiDAR", impl_->parameters.relative_cutoff);
}
void Scheduler::setTime(double) { ++impl_->update_index; }
void Scheduler::setReferenceSupport(const Eigen::Vector3d &support) { if (support.allFinite()) { impl_->reference_support = support.cwiseMax(0.0).cwiseMin(1.0); impl_->reference_valid = true; } }
void Scheduler::setReferenceInnovation(double signed_innovation_x, bool valid) {
  impl_->reference_signed_innovation_x = signed_innovation_x;
  impl_->reference_innovation_valid = valid && std::isfinite(signed_innovation_x);
}
Eigen::Vector3d Scheduler::support() const { return impl_->last_support; }
double Scheduler::signedInnovation() const { return impl_->last_signed_innovation_x; }
bool Scheduler::signedInnovationValid() const { return impl_->last_innovation_valid; }
bool Scheduler::active() const { return impl_->enabled; }
bool Scheduler::observing() const { return impl_->enabled; }
void Scheduler::apply(Eigen::MatrixXd &H, Eigen::VectorXd &z, const Eigen::VectorXd &R_inv,
                      const Eigen::Matrix3d &Rwb, int iteration, const char *sensor,
                      const Eigen::Matrix3d *prior_position_covariance_world) {
  // Keep disabled scheduling as a true no-op: no allocation or matrix changes.
  // 关闭调度时保持严格空操作，不分配临时矩阵，也不改动观测数据。
  if (!impl_->enabled) return;
  (void)iteration;
  (void)sensor;
  if (impl_->visual_role && prior_position_covariance_world) {
    const Eigen::Matrix3d prior_body = Rwb.transpose() *
        (*prior_position_covariance_world) * Rwb;
    impl_->parameters.visual_route_a_prior_variance_x = std::max(0.0, prior_body(0,0));
  }
  const Eigen::Vector3d *ref =
      (impl_->visual_role && impl_->reference_valid) ? &impl_->reference_support : nullptr;
  Result probe;
  // Probe once per visual sensor update with the positive-gain branch disabled.
  // processFrame invokes apply several times (levels/iterations); counting
  // those calls as frames would make a temporal gate meaningless.
  // 一帧内多层、多次迭代只计一次，避免把迭代次数误当成连续帧。
  const bool use_stable_gate = impl_->visual_role &&
      impl_->priority_stable_gate_enabled && impl_->parameters.visual_priority_enabled;
  if (use_stable_gate && impl_->priority_last_observation != impl_->update_index) {
    Eigen::MatrixXd probe_H = H;
    Eigen::VectorXd probe_z = z;
    Parameters probe_parameters = impl_->parameters;
    probe_parameters.visual_priority_enabled = false;
    probe = contract(probe_H, probe_z, R_inv, Rwb, probe_parameters, ref);
    impl_->priority_gate.observe(
        impl_->update_index, probe.score(0), probe.raw_whitened_residual_rms,
        probe.signed_innovation_x, probe.innovation_x_valid,
        impl_->reference_signed_innovation_x, impl_->reference_innovation_valid,
        impl_->priority_gate_parameters);
    impl_->priority_last_observation = impl_->update_index;
  }
  Parameters active_parameters = impl_->parameters;
  if (use_stable_gate) active_parameters.visual_priority_enabled = impl_->priority_gate.active();
  const Result result = contract(H, z, R_inv, Rwb, active_parameters, ref);
  impl_->last_support = result.score;
  impl_->last_signed_innovation_x = result.signed_innovation_x;
  impl_->last_innovation_valid = result.innovation_x_valid;
}
} // namespace doa_axis
