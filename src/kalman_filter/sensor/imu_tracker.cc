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
    // std::cout << "[ImuTracker::ImuTracker][imu_gravity_time_constant_:] " << imu_gravity_time_constant_ << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][time_:] " << time_ << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][last_linear_acceleration_time_:] " << last_linear_acceleration_time_ << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][orientation_:] " << orientation_.vec()  << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][orientation_.w():] " << orientation_.w() << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][gravity_vector_:] " << gravity_vector_ << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][imu_angular_velocity_:] " << imu_angular_velocity_ << std::endl;

}

ImuTracker::ImuTracker(const double imu_gravity_time_constant, const common::Time time):
    imu_gravity_time_constant_(imu_gravity_time_constant),
    time_i_(time),
    last_linear_acceleration_time_i_(common::Time::min()),
    orientation_(Eigen::Quaterniond::Identity()),
    gravity_vector_(Eigen::Vector3d::UnitZ()),
    imu_angular_velocity_(Eigen::Vector3d::Zero())
{
    std::cout << "[ImuTracker::ImuTracker][imu_gravity_time_constant_:] " << imu_gravity_time_constant_ << std::endl;
    std::cout << "[ImuTracker::ImuTracker][time_:] " << time_ << std::endl;
    // std::cout << "[ImuTracker::ImuTracker][last_linear_acceleration_time_:] " << last_linear_acceleration_time_ << std::endl;
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

void ImuTracker::Advance(const common::Time time) 
{
  CHECK_LE(time_i_, time);
  const double delta_t = common::ToSeconds(time - time_i_);
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  orientation_ = (orientation_ * rotation).normalized();
  gravity_vector_ = rotation.inverse() * gravity_vector_;
  time_i_ = time;


  std::cout << "[ImuTracker::Advance][delta_t:] " << delta_t << std::endl;
}



void ImuTracker::AddImuLinearAccelerationObservation(const Eigen::Vector3d& imu_linear_acceleration) 
{
    std::cout << "[ImuTracker::AddImuLinearAccelerationObservation][std::numeric_limits<double>::infinity():] " << std::numeric_limits<double>::infinity() << std::endl;
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.


    // const double delta_t =
    //   last_linear_acceleration_time_ > common::Time::min()
    //       ? common::ToSeconds(time_ - last_linear_acceleration_time_)
    //       : std::numeric_limits<double>::infinity();


  const double delta_t =
      last_linear_acceleration_time_i_ > common::Time::min() ? 
      common::ToSeconds(time_i_ - last_linear_acceleration_time_i_): std::numeric_limits<double>::infinity();

  last_linear_acceleration_time_i_ = time_i_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ = (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.inverse() * Eigen::Vector3d::UnitZ());
      
  orientation_ = (orientation_ * rotation).normalized();

  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

void ImuTracker::AddImuAngularVelocityObservation(const Eigen::Vector3d& imu_angular_velocity) 
{  
   imu_angular_velocity_ = imu_angular_velocity;
}




} //namespace sensor