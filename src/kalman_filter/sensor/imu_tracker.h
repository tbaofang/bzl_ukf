#ifndef SENSOR_IMU_TRACKER_H_
#define SENSOR_IMU_TRACKER_H_

#include "Eigen/Geometry"

#include "customized_time.h"

namespace sensor {

class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, double time);
  ImuTracker(double imu_gravity_time_constant, common::Time time);


  void Advance(double time);
  void Advance(common::Time time);



  Eigen::Quaterniond orientation() const { return orientation_; }


  void AddImuLinearAccelerationObservation(const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(const Eigen::Vector3d& imu_angular_velocity);


 private:
  const double imu_gravity_time_constant_;
  double time_;
  common::Time time_i_;
  double last_linear_acceleration_time_;
  common::Time last_linear_acceleration_time_i_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;

};
}

#endif