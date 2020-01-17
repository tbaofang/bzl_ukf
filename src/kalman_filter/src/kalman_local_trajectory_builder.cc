#include "kalman_local_trajectory_builder.h"

#include "make_unique.h"

namespace kalman_filter{

void KalmanLocalTrajectoryBuilder::CreateKalmanLocalTrajectoryBuilderOptions() 
{
    bool use_online_correlative_scan_matching =  false;
    double scan_matcher_variance = 2.34e-9;
    double odometer_translational_variance = 1e-7;
    double odometer_rotational_variance = 1e-7;

    double imu_gravity_variance = 0;
    int num_odometry_states = 1;
    double position_model_variance = 0.00654766;
    double orientation_model_variance = 5e-3;
    double velocity_model_variance = 0.53926;

  kalman_filter::proto::KalmanLocalTrajectoryBuilderOptions options;

  options.set_use_online_correlative_scan_matching(use_online_correlative_scan_matching);
  options.set_scan_matcher_variance(scan_matcher_variance);
  options.set_odometer_translational_variance(odometer_translational_variance);
  options.set_odometer_rotational_variance(odometer_rotational_variance);

  options.set_imu_gravity_variance(imu_gravity_variance);
  options.set_num_odometry_states(num_odometry_states);
  options.set_position_model_variance(position_model_variance);
  options.set_orientation_model_variance(orientation_model_variance);
  options.set_velocity_model_variance(velocity_model_variance);

//   *options.mutable_real_time_correlative_scan_matcher_options() =
//       mapping_2d::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
//           parameter_dictionary
//               ->GetDictionary("real_time_correlative_scan_matcher")
//               .get());
//   *options.mutable_pose_tracker_options() =
//       kalman_filter::CreatePoseTrackerOptions(
//           parameter_dictionary->GetDictionary("pose_tracker").get());

  // *options.mutable_real_time_correlative_scan_matcher_options() =
  //     mapping_2d::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
  //         parameter_dictionary
  //             ->GetDictionary("real_time_correlative_scan_matcher")
  //             .get());
  options_ = options;
}

KalmanLocalTrajectoryBuilder::KalmanLocalTrajectoryBuilder()
{
    CreateKalmanLocalTrajectoryBuilderOptions();

    // std::cout << "[KalmanLocalTrajectoryBuilder::KalmanLocalTrajectoryBuilder][options_.odometer_translational_variance():] " << options_.odometer_translational_variance() << std::endl;
    // bool use_online_correlative_scan_matching =  false;
    // double scan_matcher_variance = 2.34e-9;
    // double odometer_translational_variance = 1e-7;
    // double odometer_rotational_variance = 1e-7;

//   this->options_.set_use_online_correlative_scan_matching(use_online_correlative_scan_matching);
//   options_.set_scan_matcher_variance(scan_matcher_variance);
//   options_.set_odometer_translational_variance(odometer_translational_variance);
//   options_.set_odometer_rotational_variance(odometer_rotational_variance);
}

KalmanLocalTrajectoryBuilder::~KalmanLocalTrajectoryBuilder() {}


void KalmanLocalTrajectoryBuilder::AddOdometryData(double time, const transform::Rigid3d& pose)
{
    std::cout << "[AddOdometryData][time:] " << time << std::endl;
    // std::cout << "[AddOdometryData][pose.translation().x():] " << pose.translation().x() << std::endl;
    // std::cout << "[AddOdometryData][pose.translation().y():] " << pose.translation().y() << std::endl;
    // std::cout << "[AddOdometryData][pose.translation().z():] " << pose.translation().z() << std::endl;
    // std::cout << "[AddOdometryData][pose.rotation().x():] " << pose.rotation().x() << std::endl;
    // std::cout << "[AddOdometryData][pose.rotation().y():] " << pose.rotation().y() << std::endl;
    // std::cout << "[AddOdometryData][pose.rotation().z():] " << pose.rotation().z() << std::endl;
    // std::cout << "[AddOdometryData][pose.rotation().w():] " << pose.rotation().w() << std::endl;

    if (!pose_tracker_) 
    {
      pose_tracker_.reset(new kalman_filter::PoseTracker(options_,  time));
    }

    // std::cout << "[KalmanLocalTrajectoryBuilder::AddOdometryData][options_.odometer_translational_variance():] " << options_.odometer_translational_variance() << std::endl;

    // kalman_filter::BuildPoseCovariance(options_.odometer_translational_variance(),
    //                                      options_.odometer_rotational_variance());
    pose_tracker_->AddOdometerPoseObservation(time, pose,
      kalman_filter::BuildPoseCovariance(options_.odometer_translational_variance(), options_.odometer_rotational_variance()));

}


  void KalmanLocalTrajectoryBuilder::AddImuData(double time, const Eigen::Vector3d& linear_acceleration, 
                                                const Eigen::Vector3d& angular_velocity)
  {
    if (!pose_tracker_) {
    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(options_, time);
  }

  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  // transform::Rigid3d pose_estimate;
  // kalman_filter::PoseCovariance unused_covariance_estimate;
  // pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate, &unused_covariance_estimate);


  }                                                



}