#pragma once
#include <Eigen/Core>
#include <memory>
#include <ros/ros.h>

namespace doa_axis {
// Keep SVD implementation in a separate compilation unit so existing ROS/PCL
// translation units do not instantiate the numerical core repeatedly.
class Scheduler {
 public:
  Scheduler();
  ~Scheduler();
  void configure(ros::NodeHandle &nh, bool visual_role = false);
  void setTime(double sensor_time);
  void setReferenceSupport(const Eigen::Vector3d &support);
  void setReferenceInnovation(double signed_innovation_x, bool valid = true);
  Eigen::Vector3d support() const;
  double signedInnovation() const;
  bool signedInnovationValid() const;
  bool active() const;
  bool observing() const;
  void apply(Eigen::MatrixXd &H, Eigen::VectorXd &z, const Eigen::VectorXd &R_inv,
             const Eigen::Matrix3d &Rwb, int iteration, const char *sensor = "lidar",
             const Eigen::Matrix3d *prior_position_covariance_world = nullptr);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace doa_axis
