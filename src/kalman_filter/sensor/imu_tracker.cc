#include "imu_tracker.h"
#include "transform.h"
#include "glog/logging.h"

#include <iostream>

namespace sensor{

ImuTracker::ImuTracker(const double imu_gravity_time_constant, const double time):
    imu_gravity_time_constant_(imu_gravity_time_constant),
    time_(time),
    last_linear_acceleration_time_(0.0),
    orientation_(Eigen::Quaterniond::Identity()),
    gravity_vector_(Eigen::Vector3d::UnitZ()),
    imu_angular_velocity_(Eigen::Vector3d::Zero())
{
    std::cout << "[ImuTracker::ImuTracker][imu_gravity_time_constant_:] " << imu_gravity_time_constant_ << std::endl;
    std::cout << "[ImuTracker::ImuTracker][time_:] " << time_ << std::endl;
    std::cout << "[ImuTracker::ImuTracker][last_linear_acceleration_time_:] " << last_linear_acceleration_time_ << std::endl;
    std::cout << "[ImuTracker::ImuTracker][orientation_:] " << orientation_.vec()  << std::endl;
    std::cout << "[ImuTracker::ImuTracker][orientation_.w():] " << orientation_.w() << std::endl;
    std::cout << "[ImuTracker::ImuTracker][gravity_vector_:] " << gravity_vector_ << std::endl;
    std::cout << "[ImuTracker::ImuTracker][imu_angular_velocity_:] " << imu_angular_velocity_ << std::endl;

}

void ImuTracker::Advance(const double time) 
{
    std::cout << "[ImuTracker::Advance]" << std::endl;
//  std::cout << "[][orientation_.vec():] " << orientation_.vec() << std::endl;
//  std::cout << "[][orientation_.w():] " << orientation_.w() << std::endl;


  CHECK_LE(time_, time);
  const double delta_t = time - time_;

  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(imu_angular_velocity_ * delta_t));

  orientation_ = (orientation_ * rotation).normalized();
  gravity_vector_ = rotation.inverse() * gravity_vector_;
  time_ = time;

  std::cout << "[ImuTracker::Advance][delta_t:] " << delta_t << std::endl;
  std::cout << "[ImuTracker::Advance][imu_angular_velocity_:] " << imu_angular_velocity_ << std::endl;
  std::cout << "[ImuTracker::Advance][orientation_.vec():] " << orientation_.vec() << std::endl;
  std::cout << "[ImuTracker::Advance][orientation_.w():] " << orientation_.w() << std::endl;
}




} //namespace sensor